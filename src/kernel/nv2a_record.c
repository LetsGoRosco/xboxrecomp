/*
 * RECOMP_RECORD=<file>: stream every presented frame into ffmpeg.
 *
 * The pushbuffer executor calls xbox_Nv2aRecordFrame at each flip with the
 * surface it just finished. The pixels go down a pipe to an ffmpeg process as
 * raw BGRA, so a recording needs no intermediate files and no second pass.
 *
 * Timing: by default each frame carries the wall-clock time it arrived at
 * (ffmpeg's -use_wallclock_as_timestamps), so the video plays at the speed
 * the title actually ran, however slow the rasteriser is.
 * RECOMP_RECORD_FPS=<n> instead plays every flip at a fixed rate, e.g. 50 to
 * see a PAL title at the speed it was meant to run.
 *
 * The output is fragmented MP4: a run that is killed (a time limit, a crash)
 * still leaves a playable file up to the last fragment. The frame size is
 * fixed by the first frame; later frames of another size are scaled to it.
 * ffmpeg must be on PATH; RECOMP_RECORD_FFMPEG names another binary, and
 * RECOMP_RECORD_ARGS replaces the encoder options (default: libx264,
 * ultrafast, crf 20).
 *
 * Sound: with wall-clock timing the APU's final mix (what goes to the
 * speakers, 48 kHz s16 stereo) is the second input, through a named pipe,
 * encoded as AAC (RECOMP_RECORD_AUDIO_ARGS replaces "-c:a aac -b:a 192k";
 * RECOMP_RECORD_AUDIO=0 records video only). The audio track runs on the
 * same clock as the frames: it starts when the first frame is recorded, a
 * writer thread hands ffmpeg the mix as it arrives, and it is padded with
 * silence when the mix stalls (a paused APU, a level load) and trimmed when
 * it runs ahead, so it never drifts from the wall clock by more than about
 * 0.1-0.25 s. Nothing here blocks the APU: the mix goes into a ring, and
 * the pipe is written non-blocking, so a stalled or dead ffmpeg costs
 * audio, never a frame of the title. With RECOMP_RECORD_FPS the video has
 * no wall clock to follow, so there is no sound. POSIX only.
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>

#ifndef _WIN32
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>
#define RECORD_AUDIO 1
#endif

#ifdef _WIN32
#define popen _popen
#define pclose _pclose
#define POPEN_MODE "wb"
#else
#define POPEN_MODE "w"
#endif

static FILE *s_pipe;
static int s_failed;
static uint32_t s_w, s_h;
static uint8_t *s_frame;
static unsigned long s_frames;

#ifdef RECORD_AUDIO
#define AUD_RATE   48000
#define AUD_RING   (AUD_RATE * 2)      /* frames: 2 s of the mix */
#define AUD_TICK_NS 10000000L          /* the writer wakes every 10 ms */
#define AUD_LAG    (AUD_RATE / 10)     /* pad with silence past 100 ms behind */
#define AUD_PAD_TO (AUD_RATE / 20)     /*   up to 50 ms behind */
#define AUD_LEAD   (AUD_RATE / 4)      /* trim the mix past 250 ms ahead */

static struct {
    pthread_mutex_t mu;
    pthread_t thread;
    int on;                 /* the APU's frames are taken */
    int quit, started;
    char dir[256], fifo[300];
    uint32_t head, count;   /* oldest frame, frames held in s_aud_ring */
    uint64_t t0_ns;         /* the wall-clock start of the track */
    uint64_t written;       /* frames ffmpeg has taken */
    uint64_t padded, trimmed, lost;
} s_aud = { .mu = PTHREAD_MUTEX_INITIALIZER };
static int16_t s_aud_ring[AUD_RING][2];   /* apart: the struct is in .data */

static uint64_t now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

/* The writer: opens the pipe as ffmpeg's reader opens it, then each tick
 * writes what the APU produced, keeping the track on the wall clock. */
static void *audio_main(void *arg)
{
    static int16_t chunk[AUD_RING + AUD_RATE][2];
    int fd = -1;
    (void)arg;

    /* ffmpeg opens its inputs in order; a reader appears within a moment
     * unless ffmpeg failed. Give it 30 s. */
    for (int i = 0; i < 3000 && fd < 0; i++) {
        fd = open(s_aud.fifo, O_WRONLY | O_NONBLOCK);
        if (fd < 0) {
            if (errno != ENXIO)
                break;
            struct timespec ts = { 0, AUD_TICK_NS };
            nanosleep(&ts, NULL);
            pthread_mutex_lock(&s_aud.mu);
            int quit = s_aud.quit;
            pthread_mutex_unlock(&s_aud.mu);
            if (quit)
                break;
        }
    }
    if (fd < 0) {
        pthread_mutex_lock(&s_aud.mu);
        int quit = s_aud.quit;
        pthread_mutex_unlock(&s_aud.mu);
        if (!quit)
            fprintf(stderr, "  [REC] ffmpeg never opened the audio pipe;"
                    " video only\n");
        pthread_mutex_lock(&s_aud.mu);
        s_aud.on = 0;
        pthread_mutex_unlock(&s_aud.mu);
        return NULL;
    }

    size_t pend = 0, pend_off = 0;  /* bytes of chunk not yet written */
    uint64_t bytes = 0;             /* bytes ffmpeg has taken */
    for (;;) {
        pthread_mutex_lock(&s_aud.mu);
        int quit = s_aud.quit;
        uint64_t target = (now_ns() - s_aud.t0_ns) * AUD_RATE / 1000000000ull;
        uint64_t ahead = s_aud.written + pend / 4 + s_aud.count;
        /* Ahead of the clock (the mix ran fast, or ffmpeg is slow to take
         * it): drop the oldest frames past the lead. */
        if (ahead > target + AUD_LEAD) {
            uint32_t drop = (uint32_t)(ahead - target - AUD_LEAD);
            if (drop > s_aud.count)
                drop = s_aud.count;
            s_aud.head = (s_aud.head + drop) % AUD_RING;
            s_aud.count -= drop;
            s_aud.trimmed += drop;
            ahead -= drop;
        }
        size_t n = 0;
        if (!pend) {
            while (s_aud.count) {
                uint32_t run = AUD_RING - s_aud.head;
                if (run > s_aud.count)
                    run = s_aud.count;
                memcpy(chunk[n], s_aud_ring[s_aud.head], (size_t)run * 4);
                n += run;
                s_aud.head = (s_aud.head + run) % AUD_RING;
                s_aud.count -= run;
            }
            /* Behind (the mix stalled): silence up to the clock. */
            if (ahead + AUD_LAG < target) {
                uint64_t pad = target - AUD_PAD_TO - ahead;
                if (pad > AUD_RATE)
                    pad = AUD_RATE;
                memset(chunk[n], 0, (size_t)pad * 4);
                n += (size_t)pad;
                s_aud.padded += pad;
            }
            pend = n * 4;
            pend_off = 0;
        }
        pthread_mutex_unlock(&s_aud.mu);

        while (pend) {
            ssize_t w = write(fd, (uint8_t *)chunk + pend_off, pend);
            if (w > 0) {
                pend -= (size_t)w;
                pend_off += (size_t)w;
                bytes += (uint64_t)w;
                continue;
            }
            if (w < 0 && errno == EAGAIN)
                break;      /* ffmpeg is behind: the rest next tick */
            quit = 1;       /* EPIPE: ffmpeg is gone */
            break;
        }
        pthread_mutex_lock(&s_aud.mu);
        s_aud.written = bytes / 4;
        pthread_mutex_unlock(&s_aud.mu);
        if (quit)
            break;
        struct timespec ts = { 0, AUD_TICK_NS };
        nanosleep(&ts, NULL);
    }
    close(fd);
    pthread_mutex_lock(&s_aud.mu);
    s_aud.on = 0;
    pthread_mutex_unlock(&s_aud.mu);
    return NULL;
}

/* A fifo in a fresh temporary directory, for ffmpeg's second input. */
static int audio_prepare(void)
{
    const char *tmp = getenv("TMPDIR");
    if (!tmp || !*tmp)
        tmp = "/tmp";
    snprintf(s_aud.dir, sizeof s_aud.dir, "%s/recomp-rec-XXXXXX", tmp);
    if (!mkdtemp(s_aud.dir))
        return 0;
    snprintf(s_aud.fifo, sizeof s_aud.fifo, "%s/audio.pcm", s_aud.dir);
    if (mkfifo(s_aud.fifo, 0600) != 0) {
        rmdir(s_aud.dir);
        return 0;
    }
    return 1;
}

/* Before ffmpeg starts: ffmpeg blocks opening the pipe until a writer
 * opens it, and reads no frame until then, so without a writer the first
 * frame would never go down. No thread, no sound track. */
static int audio_start(void)
{
    pthread_mutex_lock(&s_aud.mu);
    s_aud.t0_ns = now_ns();
    s_aud.on = 1;
    pthread_mutex_unlock(&s_aud.mu);
    if (pthread_create(&s_aud.thread, NULL, audio_main, NULL) == 0) {
        s_aud.started = 1;
        return 1;
    }
    s_aud.on = 0;
    return 0;
}

static void audio_stop(void)
{
    if (!s_aud.fifo[0])
        return;
    pthread_mutex_lock(&s_aud.mu);
    s_aud.quit = 1;
    s_aud.on = 0;
    pthread_mutex_unlock(&s_aud.mu);
    if (s_aud.started) {
        pthread_join(s_aud.thread, NULL);
        s_aud.started = 0;
        fprintf(stderr, "  [REC] audio: %.1f s to ffmpeg (%.1f s silence"
                " padded, %.1f s trimmed, %.1f s lost)\n",
                s_aud.written / (double)AUD_RATE,
                s_aud.padded / (double)AUD_RATE,
                s_aud.trimmed / (double)AUD_RATE,
                s_aud.lost / (double)AUD_RATE);
    }
    unlink(s_aud.fifo);
    rmdir(s_aud.dir);
    s_aud.fifo[0] = 0;
}
#endif /* RECORD_AUDIO */

/* The APU's mix, as it hands it to the device: `frames` stereo s16 frames.
 * Called on the APU's frame thread; never blocks on ffmpeg. */
void xbox_Nv2aRecordAudio(const int16_t *samples, int frames)
{
#ifdef RECORD_AUDIO
    if (!s_aud.on || frames <= 0)
        return;
    pthread_mutex_lock(&s_aud.mu);
    if (s_aud.on) {
        for (int i = 0; i < frames; i++) {
            if (s_aud.count == AUD_RING) {
                /* The writer is stuck: lose the oldest. */
                s_aud.head = (s_aud.head + 1) % AUD_RING;
                s_aud.count--;
                s_aud.lost++;
            }
            uint32_t at = (s_aud.head + s_aud.count) % AUD_RING;
            s_aud_ring[at][0] = samples[2 * i];
            s_aud_ring[at][1] = samples[2 * i + 1];
            s_aud.count++;
        }
    }
    pthread_mutex_unlock(&s_aud.mu);
#else
    (void)samples;
    (void)frames;
#endif
}

static void record_close(void)
{
#ifdef RECORD_AUDIO
    audio_stop();
#endif
    if (s_pipe) {
        pclose(s_pipe);
        s_pipe = NULL;
        fprintf(stderr, "  [REC] closed after %lu frames\n", s_frames);
    }
}

static int record_open(const char *path, uint32_t w, uint32_t h)
{
    const char *ffmpeg = getenv("RECOMP_RECORD_FFMPEG");
    const char *fps = getenv("RECOMP_RECORD_FPS");
    const char *args = getenv("RECOMP_RECORD_ARGS");
    char cmd[3072], rate[64], audio_in[512] = "", audio_out[512] = "";

    if (!ffmpeg || !*ffmpeg)
        ffmpeg = "ffmpeg";
    if (!args || !*args)
        args = "-c:v libx264 -preset ultrafast -crf 20 -pix_fmt yuv420p";
    if (fps && *fps)
        snprintf(rate, sizeof rate, "-framerate %s", fps);
    else
        snprintf(rate, sizeof rate, "-use_wallclock_as_timestamps 1");

    /* H.264 with yuv420p wants even dimensions */
    s_w = w & ~1u;
    s_h = h & ~1u;
    if (!s_w || !s_h)
        return 0;

#ifdef RECORD_AUDIO
    const char *aud = getenv("RECOMP_RECORD_AUDIO");
    const char *aargs = getenv("RECOMP_RECORD_AUDIO_ARGS");
    if (!aargs || !*aargs)
        aargs = "-c:a aac -b:a 192k";
    if (!(fps && *fps) && !(aud && aud[0] == '0') && audio_prepare() &&
        audio_start()) {
        snprintf(audio_in, sizeof audio_in,
                 "-thread_queue_size 512 -f s16le -ar %d -ac 2 -i \"%s\" ",
                 AUD_RATE, s_aud.fifo);
        snprintf(audio_out, sizeof audio_out, "-map 0:v -map 1:a %s ", aargs);
    }
#endif

    snprintf(cmd, sizeof cmd,
             "\"%s\" -hide_banner -loglevel error -y "
             "-thread_queue_size 64 -f rawvideo -pix_fmt bgra -s %ux%u %s -i - "
             "%s"
             "%s %s%s -movflags +frag_keyframe+empty_moov+default_base_moof "
             "\"%s\"",
             ffmpeg, s_w, s_h, rate, audio_in, args, audio_out,
             (fps && *fps) ? "" : "-fps_mode:v passthrough", path);

#ifndef _WIN32
    /* If ffmpeg exits, a write must fail rather than kill the title */
    signal(SIGPIPE, SIG_IGN);
#endif
    s_pipe = popen(cmd, POPEN_MODE);
    if (!s_pipe) {
#ifdef RECORD_AUDIO
        audio_stop();
#endif
        return 0;
    }
    s_frame = (uint8_t *)malloc((size_t)s_w * s_h * 4);
    if (!s_frame) {
#ifdef RECORD_AUDIO
        audio_stop();
#endif
        pclose(s_pipe);
        s_pipe = NULL;
        return 0;
    }
    atexit(record_close);
    fprintf(stderr, "  [REC] recording %ux%u to %s (%s%s)\n", s_w, s_h, path,
            (fps && *fps) ? rate : "wall-clock timing",
            audio_in[0] ? ", with the APU's mix" : "");
    return 1;
}

/*
 * base points at the surface's top-left visible pixel, pitch is in bytes and
 * bpp is 2 (R5G6B5) or 4 (X8R8G8B8 / A8R8G8B8, stored as BGRA in memory).
 */
void xbox_Nv2aRecordFrame(const uint8_t *base, uint32_t w, uint32_t h,
                          uint32_t pitch, uint32_t bpp)
{
    static const char *path = (const char *)-1;
    uint32_t x, y;

    if (path == (const char *)-1) {
        path = getenv("RECOMP_RECORD");
        if (path && !*path)
            path = NULL;
    }
    if (!path || s_failed || !base || !w || !h || (bpp != 2 && bpp != 4))
        return;
    if (!s_pipe && !record_open(path, w, h)) {
        fprintf(stderr, "  [REC] could not start ffmpeg; recording off\n");
        s_failed = 1;
        return;
    }

    /* Nearest-neighbour into the fixed output size (a copy when equal) */
    for (y = 0; y < s_h; y++) {
        const uint8_t *row = base + (size_t)(y * h / s_h) * pitch;
        uint8_t *out = s_frame + (size_t)y * s_w * 4;
        for (x = 0; x < s_w; x++) {
            uint32_t sx = x * w / s_w;
            if (bpp == 4) {
                memcpy(out, row + (size_t)sx * 4, 3);
            } else {
                uint16_t v = ((const uint16_t *)row)[sx];
                out[0] = (uint8_t)((v & 0x1F) << 3);
                out[1] = (uint8_t)(((v >> 5) & 0x3F) << 2);
                out[2] = (uint8_t)(((v >> 11) & 0x1F) << 3);
            }
            out[3] = 0xFF;
            out += 4;
        }
    }

    if (fwrite(s_frame, 1, (size_t)s_w * s_h * 4, s_pipe) !=
        (size_t)s_w * s_h * 4) {
        fprintf(stderr, "  [REC] ffmpeg stopped taking frames; recording off\n");
        s_failed = 1;
#ifdef RECORD_AUDIO
        audio_stop();
#endif
        pclose(s_pipe);
        s_pipe = NULL;
        return;
    }
    s_frames++;
}

int xbox_Nv2aRecordWanted(void)
{
    static int want = -1;
    if (want < 0) {
        const char *r = getenv("RECOMP_RECORD");
        want = r && *r;
    }
    return want && !s_failed;
}
