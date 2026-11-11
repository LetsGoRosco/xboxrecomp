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
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>

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

static void record_close(void)
{
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
    char cmd[2048], rate[64];

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

    snprintf(cmd, sizeof cmd,
             "\"%s\" -hide_banner -loglevel error -y "
             "-f rawvideo -pix_fmt bgra -s %ux%u %s -i - "
             "%s %s -movflags +frag_keyframe+empty_moov+default_base_moof "
             "\"%s\"",
             ffmpeg, s_w, s_h, rate, args,
             (fps && *fps) ? "" : "-fps_mode passthrough", path);

#ifndef _WIN32
    /* If ffmpeg exits, a write must fail rather than kill the title */
    signal(SIGPIPE, SIG_IGN);
#endif
    s_pipe = popen(cmd, POPEN_MODE);
    if (!s_pipe)
        return 0;
    s_frame = (uint8_t *)malloc((size_t)s_w * s_h * 4);
    if (!s_frame) {
        pclose(s_pipe);
        s_pipe = NULL;
        return 0;
    }
    atexit(record_close);
    fprintf(stderr, "  [REC] recording %ux%u to %s (%s)\n", s_w, s_h, path,
            (fps && *fps) ? rate : "wall-clock timing");
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
        pclose(s_pipe);
        s_pipe = NULL;
        return;
    }
    s_frames++;
}
