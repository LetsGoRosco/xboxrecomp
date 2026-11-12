/*
 * MCPX APU DSP: the GP and EP on the APU (RECOMP_APU_DSP=1).
 *
 * Built only with the dsp56300 JIT (XBOXRECOMP_APU_DSP), which also defines
 * XBOXRECOMP_APU_DSP_DEVICE so the key table lets RECOMP_APU_DSP=1 through.
 * apu_dsp_emu.c is one core; this file is what the APU does with the two of
 * them each frame, as xemu's mcpx_apu_dsp_frame and gp_ep.c do it: hand the
 * GP the VP's mixbins, run it, take the stereo output from its mixbuf, and
 * run the EP once an EP period. The system memory side (scatter-gather
 * scratch, the circular FIFOs) follows gp_ep.c.
 *
 * Copyright (c) 2012 espes
 * Copyright (c) 2015 espes
 * Copyright (c) 2018-2019 Jannik Vogel
 * Copyright (c) 2019-2025 Matt Borgerson
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 */
#include "apu_state.h"
#include "apu_regs.h"
#include "apu_dsp_emu.h"
#include "apu_dsp_warm.h"
#include "apu_gp_native.h"
#include "apu_lockstat.h"
#include "fpconv.h"
#include "recomp_env.h"
#include "../kernel/xbox_memory_layout.h"
#include "../kernel/guest_det_device.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* One APU frame is MCPX_APU_GP_FRAME_CYCLES of a DSP's clock (apu_state.h).
 * A program gets twice that before the frame gives up on it: one that never
 * halts (waiting on something this model lacks) must not stall the frame
 * thread. The EP's frame is an EP period, eight APU frames. */
#define GP_FRAME_CYCLES MCPX_APU_GP_FRAME_CYCLES
#define GP_CAP          (2u * GP_FRAME_CYCLES)
#define EP_CAP          (2u * 8u * GP_FRAME_CYCLES)

/* The GP's stereo output: its mixbuf bins 0 and 1 (xemu's monitor point
 * GP). Only the front pair: DirectSound's GP image hands all six speaker
 * channels to the EP, and the EP makes the stereo mix. The speakers hear
 * the GP only until the EP delivers (see ep_out_period below). */
#define GP_OUT_LEFT  APU_DSP_MIXBUF_BASE
#define GP_OUT_RIGHT (APU_DSP_MIXBUF_BASE + NUM_SAMPLES_PER_FRAME)

/* After a bootstrap a core's first frames compile its whole program
 * (apu_dsp_warm.h): on the Deck the GP's first frame took 522 ms and the EP's
 * first burst 16.6 ms, each enough to starve the audio device run on the APU
 * thread. For about a second after each bootstrap (1500 GP frames, 188 EP
 * periods) a core runs on its worker, and the APU thread waits for each frame
 * up to WARM_WAIT_NS. A GP frame that misses it is silent, and so are the
 * frames that find the worker still busy, which are skipped and counted.
 *
 * A skipped frame is only skipped by the APU thread: the worker owes it and
 * runs it (catch-up) as soon as it is done with the frame it has, before the
 * core is the APU thread's again, so each program still runs every frame, in
 * order. They have to. The GP writes the six speaker channels into rings in
 * scratch memory that the EP reads a period at a time, and each program keeps
 * its own ring position in its DMA descriptors, which move only when it runs.
 * Dropping frames left the two out of step for good: each dropped GP frame
 * put the GP a frame behind the EP and each dropped EP period put the EP
 * eight behind, and once the EP read a half of the 512-sample ring the GP had
 * not finished, the first 1-3 frames of every period it played were 512
 * samples newer than the rest (a 187.5 Hz splice in everything heard, for the
 * session; Enter the Matrix, 94-157 GP frames and 4-8 EP periods dropped
 * after its logo movies, half its runs). Catch-up frames get silent mixbins
 * and their EP stereo output is dropped: what they produce is too late to
 * hear. The APU thread never waits for them. */
#define GP_WARM_FRAMES  1500u
#define EP_WARM_PERIODS 188u
/* Owed frames past this (a second's worth) are dropped after all: a worker
 * that never catches up must not owe without bound. Counted and logged. */
#define GP_OWED_MAX     GP_WARM_FRAMES
#define EP_OWED_MAX     EP_WARM_PERIODS
/* Above the GP's slowest steady frames (1.8 ms on the Deck), well below the
 * device's buffer. */
#define WARM_WAIT_NS    3000000u

#define STATS_FRAMES    1500u      /* one second of APU frames */
#define CAP_LOG_FRAMES  15000u     /* ten seconds */

enum { GP = 0, EP = 1 };
static const char *const k_name[2] = { "GP", "EP" };
static const int k_ids[2] = { GP, EP };

typedef struct RunWindow {
    uint64_t frames, cycles, ns, ns_max;
    uint32_t cycles_max;
} RunWindow;

/* A core's warm-up worker. busy is set by the APU thread when it hands over
 * a frame (or owes one to an idle worker) and cleared by the worker once it
 * has run that frame and every frame owed meanwhile; the core is the
 * worker's alone while it is set. */
typedef struct Worker {
    QemuThread thread;
    QemuMutex mu;
    QemuCond cv;
    bool up, busy, job, quit, done;
    bool frame;                     /* the job has a frame from dispatch */
    int catching;                   /* running an owed frame (atomic) */
    uint32_t cycles;
    uint64_t ns;
    ApuDspWarm warm;
    uint64_t skipped_shown, late_shown;
    uint64_t owed;                  /* skipped frames still to run */
    uint64_t cu_frames, cu_cycles, cu_ns;   /* owed frames run, not folded */
    uint64_t caught_up, dropped, caught_up_shown, dropped_shown;
} Worker;

static struct {
    MCPXAPUState *d;
    ApuDspEmu *emu[2];
    bool created;
    int on;                         /* -1: RECOMP_APU_DSP not read yet */
    int stats;
    uint64_t frames;
    RunWindow win[2];
    uint64_t capped_shown[2];
    uint64_t next_cap_log;
    Worker wk[2];
    ApuDspEmuStats st[2];           /* each core's, as of its last idle frame */
    /* RECOMP_DEBUG=apu_gp: the native GP beside the JIT (shadow) or in
     * its place (native). */
    ApuGpNative *gpn;
    int gp_mode;                    /* -1 unread, else GPM_* */
    /* shadow: the GP window's X as the JIT left it, so a word that differs
     * before the next frame is the title's write */
    uint32_t gp_x_snap[0xC00];
    bool gp_x_snapped;
    int ep_native;                  /* -1 unread */
    struct {
        uint64_t frames, differ, bin_differ[APU_GPN_BINS];
        uint32_t bin_max[APU_GPN_BINS];
        int logged;
        uint64_t shown_frames;
    } sh;
    /* The EP's stereo output (FIFO 0, the AC97 stream): the period its last
     * frame wrote, whether that is new since the APU thread took one, and
     * how many EP periods ago it last wrote one. Written by whichever thread
     * runs the EP frame; the worker's mutex orders it before idle() sees the
     * frame done. */
    int16_t ep_pcm[NUM_SAMPLES_PER_FRAME * 8][2];
    bool ep_fresh;
    uint32_t ep_quiet;              /* EP periods since FIFO 0 was written */
    bool period_ep;                 /* this period's output is the EP's */
    int out_mode;                   /* -1 unread, else OUT_* */
    uint64_t ep_periods, ep_delivered, ep_used;
} s = { .on = -1, .out_mode = -1, .gp_mode = -1, .ep_native = -1,
        .ep_quiet = UINT32_MAX };

/* RECOMP_DEBUG=apu_gp: which GP runs. */
enum { GPM_JIT = 0, GPM_SHADOW = 1, GPM_NATIVE = 2 };
/* What the JIT measures this program at a frame, charged for a native
 * frame so the sample limit keeps the pace it has with the JIT. */
#define GPN_FRAME_CYCLES 12895u

static int gp_mode(void)
{
    if (s.gp_mode < 0) {
        const char *e = recomp_env(RENV_APU_GP);
        s.gp_mode = GPM_JIT;
        if (e && !strcmp(e, "shadow"))
            s.gp_mode = GPM_SHADOW;
        else if (e && !strcmp(e, "native"))
            s.gp_mode = GPM_NATIVE;
        else if (e && *e && strcmp(e, "jit"))
            fprintf(stderr, "[APU GPN] apu_gp=%s: not jit, shadow or native;"
                    " jit\n", e);
    }
    return s.gp_mode;
}

/* RECOMP_DEBUG=apu_ep=native (with apu_gp=native): the EP's downmix here. */
static bool ep_native(void)
{
    if (s.ep_native < 0) {
        const char *e = recomp_env(RENV_APU_EP);
        s.ep_native = e && !strcmp(e, "native");
        if (s.ep_native && gp_mode() != GPM_NATIVE) {
            fprintf(stderr, "[APU GPN] apu_ep=native needs apu_gp=native;"
                    " the EP stays on the JIT\n");
            s.ep_native = 0;
        }
    }
    return s.ep_native;
}

/* The EP program's stereo downmix of the GP's speaker channels (bins 0-5:
 * L, R, C, LFE, Ls, Rs), fitted from apu_ep_tap over the New Game FMV
 * (etm scripts/epfit.py): one accumulator, F << 24 plus (C + S) times
 * 0x5A8279 (0.7071), rounded at bit 23, the top 16 bits. Equal to the
 * EP's FIFO 0 on all but 40 of 2.1 million samples, which are 1 LSB out on
 * rounding ties. LFE is not in it. */
#define EP_DOWNMIX_K 0x5A8279
static inline int16_t ep_downmix(uint32_t f, uint32_t c, uint32_t sur)
{
    int64_t F = (int32_t)(f << 8) >> 8, C = (int32_t)(c << 8) >> 8;
    int64_t S = (int32_t)(sur << 8) >> 8;
    int64_t acc = F * ((int64_t)1 << 24) + (C + S) * EP_DOWNMIX_K * 2 +
                  ((int64_t)1 << 23);
    int64_t v = acc >> 32;
    return (int16_t)(v > 32767 ? 32767 : v < -32768 ? -32768 : v);
}

/* apu_ep=native: the EP program does not run. What DirectSound could see of
 * it is its period counter, P:0007 (the program's kernel adds one each
 * period), kept moving here; and its request words, X:0xBC1 (a request) and
 * X:0xBC3 (a mode, 2 in Enter the Matrix), which the program would act on:
 * a request, or a mode change, is logged once, as nothing here answers it. */
static void ep_native_period(uint8_t *win)
{
    uint32_t *p = (uint32_t *)(win + APU_DSP_WIN_P);
    const uint32_t *x = (const uint32_t *)(win + APU_DSP_WIN_X);
    static uint32_t mode0 = UINT32_MAX;
    static bool told;
    p[7] = (p[7] + 1) & 0xFFFFFF;
    uint32_t req = x[0xBC1] & 0xFFFFFF, mode = x[0xBC3] & 0xFFFFFF;
    if (mode0 == UINT32_MAX)
        mode0 = mode;
    if (!told && (req || mode != mode0)) {
        told = true;
        fprintf(stderr, "[APU GPN] the title asked the EP for something"
                " (X:0BC1 %06X, X:0BC3 %06X, was %06X); with apu_ep=native"
                " no EP program answers it\n", req, mode, mode0);
    }
}

/* RECOMP_APU_OUT: which core the speakers hear. */
enum { OUT_AUTO = 0, OUT_EP = 1, OUT_GP = 2 };
/* auto keeps the EP's output while the EP has written FIFO 0 within this
 * many periods (21 ms), so one late warm-up period does not hand the
 * speakers back to the GP. */
#define EP_QUIET_MAX 4u

static int out_mode(void)
{
    if (s.out_mode < 0) {
        const char *e = recomp_env(RENV_APU_OUT);
        s.out_mode = OUT_AUTO;
        if (e && !strcmp(e, "ep"))
            s.out_mode = OUT_EP;
        else if (e && !strcmp(e, "gp"))
            s.out_mode = OUT_GP;
        else if (e && *e && strcmp(e, "auto"))
            fprintf(stderr, "[APU DSP] RECOMP_APU_OUT=%s: not auto, ep or gp;"
                    " auto\n", e);
    }
    return s.out_mode;
}

bool mcpx_apu_dsp_device_on(void)
{
    if (s.on < 0) {
        s.on = recomp_env(RENV_APU_DSP) != NULL;
        s.stats = recomp_env_on(RENV_APU_DSP_STATS) > 0;
        /* The title writes GPRST through the MCPX window. Without it the GP
         * never starts, nothing answers the doorbell and audio init waits
         * for ever: keep the stub for the session (apu_dsp.c's ack then
         * takes its default). */
        if (s.on && !xbox_McpxHost()) {
            fprintf(stderr, "[APU DSP] RECOMP_APU_DSP=1 but no MCPX window:"
                    " the stub\n");
            s.on = 0;
        }
    }
    return s.on;
}

/* ---- system memory: scatter-gather pages and circular FIFOs ---- */

/* An SGE table: 8 bytes an entry, the physical page in the first dword. */
static void sg_rw(uint32_t sge_base, uint32_t max_sge, uint8_t *ptr,
                  uint32_t addr, size_t len, bool dir)
{
    uint32_t pe = addr / 4096, off = addr % 4096;
    size_t n = 4096 - off;

    while (len > 0) {
        if (pe > max_sge) {
            /* Both cores' DMA can land here, the GP's on its worker. */
            static long logs;
            if (qatomic_inc(&logs) <= 4)
                fprintf(stderr, "[APU DSP] SGE %u past the table's %u;"
                        " %zu bytes dropped\n", pe, max_sge, len);
            return;
        }
        uint32_t page = *(const uint32_t *)mcpx_apu_phys(sge_base + pe * 8);
        uint8_t *host = mcpx_apu_phys(page + off);
        if (n > len)
            n = len;
        if (dir)
            memcpy(host, ptr, n);
        else
            memcpy(ptr, host, n);
        ptr += n;
        len -= n;
        pe++;
        n = 4096;
        off = 0;
    }
}

static uint32_t reg(uint32_t a)
{
    return qatomic_read(&s.d->regs[a]);
}

static void scratch_rw(void *opaque, uint8_t *ptr, uint32_t addr, size_t len,
                       bool dir)
{
    int k = *(const int *)opaque;
    if (k == GP)
        sg_rw(reg(NV_PAPU_GPSADDR), reg(NV_PAPU_GPSMAXSGE), ptr, addr, len,
              dir);
    else
        sg_rw(reg(NV_PAPU_EPSADDR), reg(NV_PAPU_EPSMAXSGE), ptr, addr, len,
              dir);
}

static void fifo_rw(void *opaque, uint8_t *ptr, unsigned index, size_t len,
                    bool dir)
{
    int k = *(const int *)opaque;
    uint32_t grp = k == GP ? NV_PAPU_GPOFBASE0 : NV_PAPU_EPOFBASE0;
    uint32_t base, end, cur_reg;

    /* EP FIFO 0 is the AC97 stream: one period of s16 stereo, the EP's mix
     * of all six speaker channels, which xemu's ep_sink_samples hands the
     * monitor instead of the title's buffer (the buffer itself only ever
     * feeds the AC97 DMA, which nothing here runs). Taking the GP's front
     * pair instead, as this did, lost every channel but front left and
     * right: the centre is where a 5.1 movie keeps its dialogue. */
    if (k == EP && dir && index == 0 &&
        len == sizeof(s.d->monitor.frame_buf)) {
        /* An owed period's output is too late for its period: drop it. */
        if (!qatomic_read(&s.wk[EP].catching)) {
            memcpy(s.ep_pcm, ptr, sizeof(s.ep_pcm));
            s.ep_fresh = true;
        }
        return;
    }
    if (dir) {
        base = reg(grp + 0x10 * index) & 0xFFFFFF;
        end = reg(grp + 0x04 + 0x10 * index) & 0xFFFFFF;
        cur_reg = grp + 0x08 + 0x10 * index;
    } else {
        grp += NV_PAPU_GPIFBASE0 - NV_PAPU_GPOFBASE0;
        base = reg(grp + 0x10 * index) & 0xFFFFFF;
        end = reg(grp + 0x04 + 0x10 * index) & 0xFFFFFF;
        cur_reg = grp + 0x08 + 0x10 * index;
    }
    if (end <= base)
        return;
    uint32_t cur = reg(cur_reg) & 0xFFFFFF;
    if (cur < base || cur >= end)
        cur = base;
    uint32_t sge = k == GP ? reg(NV_PAPU_GPFADDR) : reg(NV_PAPU_EPFADDR);
    uint32_t max = k == GP ? reg(NV_PAPU_GPFMAXSGE) : reg(NV_PAPU_EPFMAXSGE);
    while (len > 0) {
        size_t n = end - cur;
        if (n > len)
            n = len;
        sg_rw(sge, max, ptr, cur, n, dir);
        ptr += n;
        len -= n;
        cur += (uint32_t)n;
        if (cur >= end)
            cur = base;
    }
    qatomic_set(&s.d->regs[cur_reg], (reg(cur_reg) & ~0xFFFFFFu) | cur);
}

/* ---- the warm-up workers ---- */

/* One owed frame, on the worker: as the frame would have run, with silent
 * mixbins for the GP (the VP's were for a frame already gone by). */
static uint32_t catch_up_frame(int k, uint32_t cap)
{
    static const uint32_t silent[NUM_MIXBINS * NUM_SAMPLES_PER_FRAME];
    if (k == GP)
        apu_dsp_emu_write_run(s.emu[k], APU_DSP_SPACE_X, APU_DSP_MIXBUF_BASE,
                              silent, NUM_MIXBINS * NUM_SAMPLES_PER_FRAME);
    return apu_dsp_emu_run_frame(s.emu[k], cap);
}

static void *worker_main(void *arg)
{
    int k = *(const int *)arg;
    Worker *w = &s.wk[k];
    uint32_t cap = k == GP ? GP_CAP : EP_CAP;
    uint64_t owed_max = k == GP ? GP_OWED_MAX : EP_OWED_MAX;

    qemu_mutex_lock(&w->mu);
    for (;;) {
        while (!w->job && !w->quit)
            qemu_cond_wait(&w->cv, &w->mu);
        if (w->quit)
            break;
        bool frame = w->frame;
        w->job = false;
        w->frame = false;
        if (frame) {
            qemu_mutex_unlock(&w->mu);
            uint64_t t0 = apu_lockstat_now_ns();
            uint32_t cycles = apu_dsp_emu_run_frame(s.emu[k], cap);
            uint64_t ns = apu_lockstat_now_ns() - t0;
            qemu_mutex_lock(&w->mu);
            w->cycles = cycles;
            w->ns = ns;
            w->done = true;
        }
        /* The frames the APU thread skipped meanwhile, oldest first, before
         * it can hand over the next one. */
        while (w->owed && !w->quit) {
            if (w->owed > owed_max) {
                w->dropped += w->owed - owed_max;
                w->owed = owed_max;
            }
            w->owed--;
            qatomic_set(&w->catching, 1);
            qemu_mutex_unlock(&w->mu);
            uint64_t t0 = apu_lockstat_now_ns();
            uint32_t cycles = catch_up_frame(k, cap);
            uint64_t ns = apu_lockstat_now_ns() - t0;
            qemu_mutex_lock(&w->mu);
            qatomic_set(&w->catching, 0);
            w->cu_frames++;
            w->cu_cycles += cycles;
            w->cu_ns += ns;
        }
        w->busy = false;
        qemu_cond_signal(&w->cv);
    }
    qemu_mutex_unlock(&w->mu);
    return NULL;
}

static void count(int k, uint32_t cycles, uint64_t ns)
{
    /* Every GP frame that ran, inline or on its worker, is lag accounted on
     * the frame thread. A skipped warm-up frame is the host's delay, not the
     * GP's: it adds none. Nor does a frame stopped at the cap: a GP that
     * never halts waits on something this model lacks, so its cycles say
     * nothing of the program's pace, and counting them would raise the bit
     * every 64 frames and run the title's GP restart for a gap of ours. The
     * cap hit is already counted (capped=) and logged. */
    if (k == GP && cycles < GP_CAP)
        mcpx_apu_gp_frame_ran(s.d, cycles);
    RunWindow *w = &s.win[k];
    w->frames++;
    w->cycles += cycles;
    if (cycles > w->cycles_max)
        w->cycles_max = cycles;
    w->ns += ns;
    if (ns > w->ns_max)
        w->ns_max = ns;
}

/* True when core k is the APU thread's: no frame of it is on the worker,
 * after waiting up to wait_ns for the worker to finish one. Folds a finished
 * worker frame into the stats. */
static bool idle_within(int k, uint64_t wait_ns)
{
    Worker *w = &s.wk[k];
    if (!w->up)
        return true;
    qemu_mutex_lock(&w->mu);
    if (w->busy && wait_ns) {
        uint64_t end = apu_lockstat_now_ns() + wait_ns;
        while (w->busy && apu_lockstat_now_ns() < end)
            qemu_cond_timedwait(&w->cv, &w->mu, 1);
    }
    bool idle_now = !w->busy;
    if (w->done) {
        count(k, w->cycles, w->ns);
        w->done = false;
    }
    if (w->cu_frames) {
        /* Owed frames ran as they would have: their cycles count too. */
        uint64_t n = w->cu_frames;
        for (uint64_t i = 0; i < n; i++)
            count(k, (uint32_t)(w->cu_cycles / n), w->cu_ns / n);
        w->caught_up += n;
        w->cu_frames = w->cu_cycles = w->cu_ns = 0;
    }
    qemu_mutex_unlock(&w->mu);
    return idle_now;
}

static bool idle(int k)
{
    return idle_within(k, 0);
}

static void dispatch(int k)
{
    static const char *const thread_name[2] = { "mcpx.apu_gp", "mcpx.apu_ep" };
    Worker *w = &s.wk[k];
    if (!w->up) {
        qemu_mutex_init(&w->mu);
        qemu_cond_init(&w->cv);
        w->up = true;
        qemu_thread_create(&w->thread, thread_name[k], worker_main,
                           (void *)&k_ids[k], QEMU_THREAD_JOINABLE);
    }
    qemu_mutex_lock(&w->mu);
    w->busy = true;
    w->job = true;
    w->frame = true;
    qemu_cond_signal(&w->cv);
    qemu_mutex_unlock(&w->mu);
}

/* Core k's frame was skipped (its worker had the core): the worker owes it.
 * If the worker finished after idle() looked, wake it for just that one, so
 * it still runs before the next frame is handed over. */
static void owe(int k)
{
    Worker *w = &s.wk[k];
    if (!w->up)
        return;
    qemu_mutex_lock(&w->mu);
    w->owed++;
    if (!w->busy) {
        w->busy = true;
        w->job = true;
        qemu_cond_signal(&w->cv);
    }
    qemu_mutex_unlock(&w->mu);
}

/* Hands core k's frame to its worker and waits for it up to the deadline:
 * true when it finished in time. */
static bool run_on_worker(int k)
{
    dispatch(k);
    bool in_time = idle_within(k, WARM_WAIT_NS);
    apu_dsp_warm_ran(&s.wk[k].warm, in_time);
    return in_time;
}

/* ---- setup ---- */

static void create(MCPXAPUState *d)
{
    uint8_t *mcpx = xbox_McpxHost();

    s.created = true;
    s.d = d;
    for (int k = 0; k < 2; k++) {
        /* The title writes the cores' memory and GPRST/EPRST through these
         * pages untrapped, so the cores run on them (the window is there:
         * mcpx_apu_dsp_device_on keeps the stub without it). */
        ApuDspEmuConfig c = {
            .name = k_name[k], .is_gp = k == GP,
            .window = mcpx + (k == GP ? 0x30000 : 0x50000),
            .opaque = (void *)&k_ids[k],
            .scratch_rw = scratch_rw, .fifo_rw = fifo_rw };
        s.emu[k] = apu_dsp_emu_create(&c);
    }
    if (gp_mode() != GPM_JIT) {
        s.gpn = apu_gpn_create(scratch_rw, (void *)&k_ids[GP]);
        /* native: the translated core runs on the GP's window, so the
         * title's reads and writes of GP memory reach it */
        if (s.gpn && gp_mode() == GPM_NATIVE)
            apu_gpn_set_window(s.gpn, mcpx + 0x30000);
        fprintf(stderr, "[APU GPN] native GP %s\n",
                gp_mode() == GPM_SHADOW ? "beside the JIT (shadow)"
                                        : "in place of the JIT's");
    }
    if (!s.emu[GP] || !s.emu[EP])
        fprintf(stderr, "[APU DSP] creating the %s failed; its frames are"
                " silent\n", !s.emu[GP] ? "GP" : "EP");
    else
        fprintf(stderr, "[APU DSP] GP and EP on the dsp56300 JIT (cap %u/%u"
                " cycles a frame)\n", GP_CAP, EP_CAP);
}

void mcpx_apu_dsp_run_finalize(void)
{
    for (int k = 0; k < 2; k++) {
        Worker *w = &s.wk[k];
        if (!w->up)
            continue;
        qemu_mutex_lock(&w->mu);
        w->quit = true;
        qemu_cond_signal(&w->cv);
        qemu_mutex_unlock(&w->mu);
        qemu_thread_join(&w->thread);
        qemu_cond_destroy(&w->cv);
        qemu_mutex_destroy(&w->mu);
    }
    for (int k = 0; k < 2; k++)
        if (s.emu[k])
            apu_dsp_emu_destroy(s.emu[k]);
    int on = s.on, stats = s.stats;
    memset(&s, 0, sizeof(s));
    s.on = on;
    s.stats = stats;
    s.out_mode = -1;
    s.ep_quiet = UINT32_MAX;
}

/* ---- reports ---- */

static void report(void)
{
    /* A core on its worker is writing its stats: keep the last snapshot. */
    for (int k = 0; k < 2; k++)
        if (idle(k))
            apu_dsp_emu_get_stats(s.emu[k], &s.st[k]);
    const ApuDspEmuStats *st = s.st;

    if (s.stats && s.frames % STATS_FRAMES == 0) {
        fprintf(stderr, "[APU DSP] t=%llus",
                (unsigned long long)(s.frames / STATS_FRAMES));
        for (int k = 0; k < 2; k++) {
            const RunWindow *w = &s.win[k];
            fprintf(stderr, " %s frames=%llu cyc avg=%llu max=%u us avg=%.1f"
                    " max=%.1f capped=%llu dma=%llu rej=%llu skipped=%llu"
                    " late=%llu",
                    k_name[k],
                    (unsigned long long)w->frames,
                    (unsigned long long)(w->frames ? w->cycles / w->frames : 0),
                    w->cycles_max,
                    w->frames ? (double)w->ns / (double)w->frames / 1e3 : 0.0,
                    (double)w->ns_max / 1e3,
                    (unsigned long long)st[k].capped,
                    (unsigned long long)st[k].dma_blocks,
                    (unsigned long long)st[k].dma_rejected,
                    (unsigned long long)s.wk[k].warm.skipped,
                    (unsigned long long)s.wk[k].warm.late);
        }
        fprintf(stderr, " ssl=%u lag=%llu", s.d->ssl_raised,
                (unsigned long long)(s.d->gp_lag_cycles
                                     * NUM_SAMPLES_PER_FRAME
                                     / MCPX_APU_GP_FRAME_CYCLES));
        fputc('\n', stderr);
        memset(s.win, 0, sizeof(s.win));
    }

    /* Cap hits and skipped warm-up frames are never silent: whatever the
     * key, a line every ten seconds while there are new ones. */
    if (s.frames < s.next_cap_log)
        return;
    if (st[GP].capped != s.capped_shown[GP] ||
        st[EP].capped != s.capped_shown[EP] ||
        s.wk[GP].warm.skipped != s.wk[GP].skipped_shown ||
        s.wk[EP].warm.skipped != s.wk[EP].skipped_shown ||
        s.wk[GP].warm.late != s.wk[GP].late_shown ||
        s.wk[EP].warm.late != s.wk[EP].late_shown ||
        s.wk[GP].caught_up != s.wk[GP].caught_up_shown ||
        s.wk[EP].caught_up != s.wk[EP].caught_up_shown ||
        s.wk[GP].dropped != s.wk[GP].dropped_shown ||
        s.wk[EP].dropped != s.wk[EP].dropped_shown) {
        fprintf(stderr, "[APU DSP] cycle cap hit: GP %llu, EP %llu frames;"
                " warm-up skipped/late: GP %llu/%llu frames,"
                " EP %llu/%llu periods; skipped run later (caught up): GP"
                " %llu, EP %llu; dropped: GP %llu, EP %llu\n",
                (unsigned long long)st[GP].capped,
                (unsigned long long)st[EP].capped,
                (unsigned long long)s.wk[GP].warm.skipped,
                (unsigned long long)s.wk[GP].warm.late,
                (unsigned long long)s.wk[EP].warm.skipped,
                (unsigned long long)s.wk[EP].warm.late,
                (unsigned long long)s.wk[GP].caught_up,
                (unsigned long long)s.wk[EP].caught_up,
                (unsigned long long)s.wk[GP].dropped,
                (unsigned long long)s.wk[EP].dropped);
        for (int k = 0; k < 2; k++) {
            s.wk[k].caught_up_shown = s.wk[k].caught_up;
            s.wk[k].dropped_shown = s.wk[k].dropped;
            s.capped_shown[k] = st[k].capped;
            s.wk[k].skipped_shown = s.wk[k].warm.skipped;
            s.wk[k].late_shown = s.wk[k].warm.late;
        }
        s.next_cap_log = s.frames + CAP_LOG_FRAMES;
    }
}

/* ---- one APU frame ---- */

static void silence(MCPXAPUState *d, int off)
{
    memset(d->monitor.frame_buf[off], 0,
           sizeof(d->monitor.frame_buf[0]) * NUM_SAMPLES_PER_FRAME);
}

/* The GP's stereo output, 24-bit words whose top 16 bits are the sample,
 * into the monitor's frame slot. */
static void gp_output(MCPXAPUState *d, ApuDspEmu *gp, int off)
{
    uint32_t l[NUM_SAMPLES_PER_FRAME], r[NUM_SAMPLES_PER_FRAME];
    apu_dsp_emu_read_run(gp, APU_DSP_SPACE_X, GP_OUT_LEFT, l,
                         NUM_SAMPLES_PER_FRAME);
    apu_dsp_emu_read_run(gp, APU_DSP_SPACE_X, GP_OUT_RIGHT, r,
                         NUM_SAMPLES_PER_FRAME);
    for (int i = 0; i < NUM_SAMPLES_PER_FRAME; i++) {
        d->monitor.frame_buf[off + i][0] = (int16_t)(l[i] >> 8);
        d->monitor.frame_buf[off + i][1] = (int16_t)(r[i] >> 8);
    }
}

/* The native GP's front pair, as gp_output reads the JIT's. */
static void gpn_output(MCPXAPUState *d, const uint32_t *mix, int off)
{
    for (int i = 0; i < NUM_SAMPLES_PER_FRAME; i++) {
        d->monitor.frame_buf[off + i][0] = (int16_t)(mix[0 * 32 + i] >> 8);
        d->monitor.frame_buf[off + i][1] = (int16_t)(mix[1 * 32 + i] >> 8);
    }
}

/* Shadow mode: the JIT's mixbuf after its frame against the native GP's
 * after the same frame's input. */
static void shadow_compare(ApuDspEmu *gp, const uint32_t *nat)
{
    uint32_t jit[APU_GPN_BINS * APU_GPN_SAMPLES];
    apu_dsp_emu_read_run(gp, APU_DSP_SPACE_X, APU_DSP_MIXBUF_BASE, jit,
                         APU_GPN_BINS * APU_GPN_SAMPLES);
    bool differ = false;
    for (int b = 0; b < APU_GPN_BINS; b++) {
        uint32_t worst = 0;
        int at = -1;
        for (int i = 0; i < APU_GPN_SAMPLES; i++) {
            int32_t dlt = (int32_t)((jit[b * 32 + i] & 0xFFFFFF) << 8) / 256 -
                          (int32_t)((nat[b * 32 + i] & 0xFFFFFF) << 8) / 256;
            uint32_t m = (uint32_t)(dlt < 0 ? -dlt : dlt);
            if (m > worst) {
                worst = m;
                at = i;
            }
        }
        if (worst) {
            differ = true;
            s.sh.bin_differ[b]++;
            if (worst > s.sh.bin_max[b])
                s.sh.bin_max[b] = worst;
            if (s.sh.logged < 12) {
                s.sh.logged++;
                fprintf(stderr, "[APU GPN] frame %llu bin %d differs: sample"
                        " %d jit %06X native %06X (max |d| %u); X:73B/75B"
                        " jit %06X/%06X native %06X/%06X\n",
                        (unsigned long long)s.frames, b, at,
                        jit[b * 32 + at] & 0xFFFFFF,
                        nat[b * 32 + at] & 0xFFFFFF, worst,
                        apu_dsp_emu_read(gp, APU_DSP_SPACE_X, 0x73B),
                        apu_dsp_emu_read(gp, APU_DSP_SPACE_X, 0x75B),
                        apu_gpn_x(s.gpn, 0x73B), apu_gpn_x(s.gpn, 0x75B));
            }
        }
    }
    if (differ && s.sh.differ == 0) {
        /* the first frame that differs: where the two X memories part */
        int shown = 0;
        for (uint32_t a = 0; a < 0xC00 && shown < 24; a++) {
            uint32_t j = apu_dsp_emu_read(gp, APU_DSP_SPACE_X, a) & 0xFFFFFF;
            uint32_t n = apu_gpn_x(s.gpn, a) & 0xFFFFFF;
            if (j != n) {
                fprintf(stderr, "[APU GPN]   X:%04X jit %06X native %06X\n",
                        a, j, n);
                shown++;
            }
        }
    }
    s.sh.frames++;
    if (differ)
        s.sh.differ++;
}

static void shadow_report(void)
{
    if (!s.gpn || s.frames % STATS_FRAMES != 0 ||
        s.sh.frames == s.sh.shown_frames)
        return;
    s.sh.shown_frames = s.sh.frames;
    fprintf(stderr, "[APU GPN] t=%llus image=%s compared=%llu differ=%llu",
            (unsigned long long)(s.frames / STATS_FRAMES),
            apu_gpn_known(s.gpn) ? "known"
                                 : apu_gpn_loaded(s.gpn) ? "unknown" : "none",
            (unsigned long long)s.sh.frames,
            (unsigned long long)s.sh.differ);
    for (int b = 0; b < APU_GPN_BINS; b++)
        if (s.sh.bin_differ[b])
            fprintf(stderr, " bin%d=%llu(max %u)", b,
                    (unsigned long long)s.sh.bin_differ[b], s.sh.bin_max[b]);
    fputc('\n', stderr);
}

static void gp_frame(MCPXAPUState *d, ApuDspEmu *gp,
                     float mixbins[NUM_MIXBINS][NUM_SAMPLES_PER_FRAME],
                     int off, bool out)
{
    Worker *w = &s.wk[GP];
    ApuDspWarmStep step = apu_dsp_warm_step(&w->warm, idle(GP));

    g_dbg.gp.cycles = 0;
    if (step == APU_DSP_WARM_SKIP) {
        /* Shadow: the native GP takes every frame, in order, as the JIT's
         * will: a skipped frame the worker runs later on silent mixbins. */
        if (s.gpn && gp_mode() == GPM_SHADOW) {
            uint32_t silent[NUM_MIXBINS * NUM_SAMPLES_PER_FRAME];
            memset(silent, 0, sizeof silent);
            apu_gpn_frame(s.gpn, silent, false);
        }
        owe(GP);        /* the worker runs it after the one it has */
        if (out)
            silence(d, off);
        return;
    }

    uint32_t words[NUM_MIXBINS * NUM_SAMPLES_PER_FRAME];
    for (int b = 0; b < NUM_MIXBINS; b++)
        for (int i = 0; i < NUM_SAMPLES_PER_FRAME; i++)
            words[b * NUM_SAMPLES_PER_FRAME + i] = float_to_24b(mixbins[b][i]);
    /* Shadow: the native GP takes the frame first, without acknowledging
     * DirectSound's command, so the JIT's GP still answers it. Compared
     * only when the JIT runs it here and now (not on its warm-up worker). */
    uint32_t nat[NUM_MIXBINS * NUM_SAMPLES_PER_FRAME];
    bool shadow = s.gpn && gp_mode() == GPM_SHADOW;
    if (shadow) {
        /* the title's writes to GP X memory since the JIT's last frame */
        const uint32_t *win = (const uint32_t *)apu_dsp_emu_window(gp);
        if (s.gp_x_snapped)
            for (uint32_t a = 0; a < 0xC00; a++)
                if ((win[a] & 0xFFFFFF) != s.gp_x_snap[a])
                    apu_gpn_poke_x(s.gpn, a, win[a]);
        memcpy(nat, words, sizeof nat);
        apu_gpn_frame(s.gpn, nat, false);
    }
    apu_dsp_emu_write_run(gp, APU_DSP_SPACE_X, APU_DSP_MIXBUF_BASE, words,
                          NUM_MIXBINS * NUM_SAMPLES_PER_FRAME);
    if (step == APU_DSP_WARM_WORKER) {
        bool in_time = run_on_worker(GP);
        if (out) {
            if (in_time)
                gp_output(d, gp, off);
            else
                silence(d, off);
        }
        return;
    }
    uint64_t t0 = apu_lockstat_now_ns();
    uint32_t cycles = apu_dsp_emu_run_frame(gp, GP_CAP);
    count(GP, cycles, apu_lockstat_now_ns() - t0);
    g_dbg.gp.cycles = (int)cycles;
    if (shadow) {
        shadow_compare(gp, nat);
        const uint32_t *win = (const uint32_t *)apu_dsp_emu_window(gp);
        for (uint32_t a = 0; a < 0xC00; a++)
            s.gp_x_snap[a] = win[a] & 0xFFFFFF;
        s.gp_x_snapped = true;
    }
    if (out)
        gp_output(d, gp, off);
}

/* The native GP in the JIT's place: every frame, on this thread. */
static void gpn_frame(MCPXAPUState *d,
                      float mixbins[NUM_MIXBINS][NUM_SAMPLES_PER_FRAME],
                      int off, bool out)
{
    uint32_t words[NUM_MIXBINS * NUM_SAMPLES_PER_FRAME];
    for (int b = 0; b < NUM_MIXBINS; b++)
        for (int i = 0; i < NUM_SAMPLES_PER_FRAME; i++)
            words[b * NUM_SAMPLES_PER_FRAME + i] = float_to_24b(mixbins[b][i]);
    uint64_t t0 = apu_lockstat_now_ns();
    apu_gpn_frame(s.gpn, words, true);
    count(GP, GPN_FRAME_CYCLES, apu_lockstat_now_ns() - t0);
    /* the host's view of the mixbuf, as apu_dsp_emu_run_frame keeps it */
    memcpy(xbox_McpxHost() + 0x30000 + APU_DSP_WIN_MIXBUF, words,
           sizeof words);
    g_dbg.gp.cycles = (int)GPN_FRAME_CYCLES;
    if (out && ep_native()) {
        for (int i = 0; i < NUM_SAMPLES_PER_FRAME; i++) {
            d->monitor.frame_buf[off + i][0] =
                ep_downmix(words[0 * 32 + i], words[2 * 32 + i],
                           words[4 * 32 + i]);
            d->monitor.frame_buf[off + i][1] =
                ep_downmix(words[1 * 32 + i], words[2 * 32 + i],
                           words[5 * 32 + i]);
        }
    } else if (out) {
        gpn_output(d, words, off);
    }
}

static void ep_frame(ApuDspEmu *ep)
{
    switch (apu_dsp_warm_step(&s.wk[EP].warm, idle(EP))) {
    case APU_DSP_WARM_SKIP:
        owe(EP);        /* the worker runs it after the one it has */
        break;
    case APU_DSP_WARM_WORKER:
        run_on_worker(EP);
        break;
    case APU_DSP_WARM_INLINE: {
        uint64_t t0 = apu_lockstat_now_ns();
        uint32_t cycles = apu_dsp_emu_run_frame(ep, EP_CAP);
        count(EP, cycles, apu_lockstat_now_ns() - t0);
        g_dbg.ep.cycles = (int)cycles;
        break;
    }
    }
}

/* After an EP frame: take the period it wrote to FIFO 0, if any. An EP
 * frame still on its warm-up worker has not written it yet; idle() then
 * says so and the period stays silent (frame_buf is cleared after each
 * period is sent), as a late GP warm-up frame does. The EP's output goes
 * out at the end of the same period, as on xemu: its frame runs at the
 * period's first APU frame and the monitor sends the buffer at the last. */
/* RECOMP_DEBUG=apu_ep_tap=path[@s]: per EP period that delivered, a
 * record of: u64 APU frame, the six GP rings (scratch 0x8000 + 0x800*ch,
 * 512 u32 each), the period's FIFO 0 output (256 x 2 s16). */
#define EP_TAP_PERIODS 4096u
static void ep_tap(void)
{
    static FILE *f;
    static int state;           /* 0 unread, 1 waiting/on, 2 done/off */
    static uint64_t from, n;
    if (state == 0) {
        state = 2;
        const char *e = recomp_env(RENV_APU_EP_TAP);
        if (e && *e) {
            char path[1024];
            snprintf(path, sizeof path, "%s", e);
            char *at = strrchr(path, '@');
            if (at) {
                *at = 0;
                from = strtoull(at + 1, NULL, 10) * 1500;
            }
            f = fopen(path, "wb");
            state = f ? 1 : 2;
            if (!f)
                fprintf(stderr, "[APU DSP] ep tap: cannot write %s\n", path);
        }
    }
    if (state != 1 || s.frames < from)
        return;
    uint32_t rings[6 * 512];
    for (int ch = 0; ch < 6; ch++)
        scratch_rw((void *)&k_ids[GP], (uint8_t *)&rings[ch * 512],
                   0x8000 + 0x800 * ch, 0x800, false);
    uint64_t fr = s.frames;
    fwrite(&fr, sizeof fr, 1, f);
    fwrite(rings, sizeof rings, 1, f);
    fwrite(s.ep_pcm, sizeof s.ep_pcm, 1, f);
    if (++n >= EP_TAP_PERIODS) {
        fclose(f);
        state = 2;
        fprintf(stderr, "[APU DSP] ep tap: %u periods written\n",
                EP_TAP_PERIODS);
    }
}

static void ep_out_period(MCPXAPUState *d, bool out)
{
    s.ep_periods++;
    bool fresh = idle(EP) && s.ep_fresh;
    if (fresh) {
        ep_tap();
        s.ep_fresh = false;
        s.ep_quiet = 0;
        s.ep_delivered++;
    } else if (s.ep_quiet < UINT32_MAX) {
        s.ep_quiet++;
    }
    if (!out || !s.period_ep)
        return;
    if (fresh) {
        memcpy(d->monitor.frame_buf, s.ep_pcm, sizeof(d->monitor.frame_buf));
        s.ep_used++;
    } else {
        memset(d->monitor.frame_buf, 0, sizeof(d->monitor.frame_buf));
    }
    if (s.ep_used == 1 && fresh)
        fprintf(stderr, "[APU DSP] output: the EP's AC97 FIFO (EP period"
                " %llu)\n", (unsigned long long)s.ep_periods);
}

/* RECOMP_DEBUG=apu_dsp_dump=<dir>: the cores' windows every 5 s of frames,
 * each taken while its core is off the worker so the copy is one frame's. */
static void dump_windows(void)
{
    static const char *dir;
    static int looked;
    if (!looked) {
        looked = 1;
        dir = recomp_env(RENV_APU_DSP_DUMP);
        if (dir && !*dir)
            dir = NULL;
    }
    if (!dir || s.frames % (5 * 1500) != 0)
        return;
    static const char *const name[2] = { "gp", "ep" };
    for (int k = 0; k < 2; k++) {
        if (!s.emu[k] || !idle(k))
            continue;
        char path[1024];
        snprintf(path, sizeof path, "%s/%s-%04llu.win", dir, name[k],
                 (unsigned long long)(s.frames / 1500));
        FILE *f = fopen(path, "wb");
        if (!f) {
            fprintf(stderr, "[APU DSP] dump: cannot write %s\n", path);
            continue;
        }
        fwrite(apu_dsp_emu_window(s.emu[k]), 1, APU_DSP_WINDOW_BYTES, f);
        fclose(f);
        /* The core's whole scratch space, as its DMA sees it (<core>-<s>.scr). */
        uint32_t sges = reg(k == GP ? NV_PAPU_GPSMAXSGE : NV_PAPU_EPSMAXSGE);
        size_t len = ((size_t)sges + 1) * 4096;
        uint8_t *buf = malloc(len);
        if (!buf)
            continue;
        scratch_rw((void *)&k_ids[k], buf, 0, len, false);
        snprintf(path, sizeof path, "%s/%s-%04llu.scr", dir, name[k],
                 (unsigned long long)(s.frames / 1500));
        f = fopen(path, "wb");
        if (f) {
            fwrite(buf, 1, len, f);
            fclose(f);
        }
        free(buf);
    }
}

void mcpx_apu_dsp_run_frame(MCPXAPUState *d,
                            float mixbins[NUM_MIXBINS][NUM_SAMPLES_PER_FRAME])
{
    if (!s.created)
        create(d);
    ApuDspEmu *gp = s.emu[GP], *ep = s.emu[EP];
    int off = (d->ep_frame_div % 8) * NUM_SAMPLES_PER_FRAME;
    bool out = d->monitor.point != MCPX_APU_DEBUG_MON_VP;

    if (!gp || !ep) {
        if (out)
            silence(d, off);
        return;
    }

    /* GPRST and EPRST come through the untrapped window: act on a change,
     * when the core is not on its worker. A bootstrap starts the warm-up.
     * An EPRST write restarts the EP period, as xemu's ep_frame_div = 0. */
    if (idle(GP) && apu_dsp_emu_poll_rst(gp)) {
        d->gp_lag_cycles = 0;
        if (s.gpn)
            apu_gpn_reset(s.gpn);
        s.gp_x_snapped = false;
        /* Deterministic mode: no warm-up worker. A frame that missed the
         * worker's deadline would be silent or skipped by host time; the
         * frame thread is not on the device's pace there anyway. */
        if (apu_dsp_emu_running(gp) && !xbox_GuestDetOn())
            apu_dsp_warm_start(&s.wk[GP].warm, GP_WARM_FRAMES);
    }
    if (idle(EP) && apu_dsp_emu_poll_rst(ep)) {
        d->ep_frame_div = 0;
        off = 0;
        if (apu_dsp_emu_running(ep) && !xbox_GuestDetOn())
            apu_dsp_warm_start(&s.wk[EP].warm, EP_WARM_PERIODS);
    }

    /* A running GP program sets the sample limit's pace: its lag, not the
     * stub's once-per-start tick (mcpx_apu_ssl_tick). */
    d->gp_emulated = apu_dsp_emu_running(gp);

    /* Which core this period's output comes from, chosen at its first
     * frame and kept for all eight, so a period is never half one core's.
     * xemu's MON_GP_OR_EP takes the EP's whenever EPRST lets it run; auto
     * waits until the EP has actually written FIFO 0, so an EP program that
     * never does (or one still warming up) leaves the GP's front pair. */
    if (d->ep_frame_div % 8 == 0) {
        int mode = out_mode();
        bool ep_live = apu_dsp_emu_running(ep) &&
                       s.ep_quiet <= EP_QUIET_MAX;
        s.period_ep = !ep_native() && (mode == OUT_EP ||
                      (mode == OUT_AUTO && ep_live));
    }
    bool gp_out = out && !s.period_ep;

    if (d->gp_emulated && s.gpn && gp_mode() == GPM_NATIVE) {
        gpn_frame(d, mixbins, off, gp_out);
    } else if (d->gp_emulated) {
        gp_frame(d, gp, mixbins, off, gp_out);
    } else {
        /* No GP program, no output: the hardware's speakers hear the GP. */
        g_dbg.gp.cycles = 0;
        if (gp_out)
            silence(d, off);
    }

    g_dbg.ep.cycles = 0;
    if (ep_native()) {
        /* the downmix went out with the GP's frame; the EP program itself
         * does not run, so keep what DirectSound could see of it */
        if (d->ep_frame_div % 8 == 0 && apu_dsp_emu_running(ep))
            ep_native_period(apu_dsp_emu_window(ep));
    } else if (d->ep_frame_div % 8 == 0 && apu_dsp_emu_running(ep)) {
        ep_frame(ep);
        ep_out_period(d, out);
    } else if (d->ep_frame_div % 8 == 0 && out && s.period_ep) {
        /* Forced to the EP (apu_out=ep) with no EP running: silence. */
        memset(d->monitor.frame_buf, 0, sizeof(d->monitor.frame_buf));
    }

    s.frames++;
    report();
    shadow_report();
    dump_windows();
}
