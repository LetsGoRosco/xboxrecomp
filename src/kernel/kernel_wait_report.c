/*
 * kernel_wait_report.c - RECOMP_DEBUG=wait_report=<ms>: where guest threads
 * waited during a slow flip.
 *
 * Every place a guest thread sleeps (kernel_signpost.h: lowered spin loops,
 * the guest CPU queue, kernel waits and sleeps) adds its time to the calling
 * thread's table, keyed by kind and site. At each flip the tables are
 * cleared; first, if the flip took longer than the threshold, they are
 * printed: what each thread was blocked on while that frame was late. The
 * Time Profiler sees only running threads, and a frame where the game thread
 * mostly waited (Halo 2's checkpoint reload: 19 ms running of 100) needs
 * this. Unlocked: a diagnostic, and a torn count names a wait that happened.
 */
#include "kernel.h"
#include "kernel_signpost.h"
#include "recomp_env.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

volatile int g_xbox_wait_report;
static int s_resolved;
static double s_threshold_ms;
static uint64_t s_last_flip_ns;

#define WR_THREADS 16
#define WR_SITES   24
typedef struct { int kind; uint32_t site; uint64_t ns; uint32_t n; } WrSite;
typedef struct { unsigned long tid; WrSite site[WR_SITES]; } WrThread;
static WrThread s_thr[WR_THREADS];

uint64_t xbox_wait_report_now(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

static void resolve(void)
{
    const char *e = recomp_env(RENV_WAIT_REPORT);
    s_resolved = 1;
    s_threshold_ms = e && e[0] ? atof(e) : 0.0;
    g_xbox_wait_report = s_threshold_ms > 0.0;
}

void xbox_wait_report_add(int kind, uint32_t site, uint64_t ns)
{
    unsigned long tid = (unsigned long)GetCurrentThreadId();
    WrThread *t = NULL;
    int i;
    for (i = 0; i < WR_THREADS; i++) {
        if (s_thr[i].tid == tid) { t = &s_thr[i]; break; }
        if (!s_thr[i].tid) { s_thr[i].tid = tid; t = &s_thr[i]; break; }
    }
    if (!t)
        return;
    for (i = 0; i < WR_SITES; i++) {
        WrSite *s = &t->site[i];
        if (s->n && s->kind == kind && s->site == site) { s->ns += ns; s->n++; return; }
        if (!s->n) { s->kind = kind; s->site = site; s->ns = ns; s->n = 1; return; }
    }
}

static const char *kind_name(int k)
{
    return k == XBOX_WAIT_SPIN ? "spin" : k == XBOX_WAIT_CPU ? "cpu" : "kwait";
}

static int by_ns_desc(const void *a, const void *b)
{
    const WrSite *x = (const WrSite *)a, *y = (const WrSite *)b;
    return x->ns < y->ns ? 1 : x->ns > y->ns ? -1 : 0;
}

void xbox_wait_report_flip(uint32_t flip)
{
    uint64_t now;
    double gap;
    int i, j;
    if (!s_resolved)
        resolve();
    if (!g_xbox_wait_report)
        return;
    now = xbox_wait_report_now();
    gap = s_last_flip_ns ? (now - s_last_flip_ns) / 1e6 : 0.0;
    s_last_flip_ns = now;
    if (gap > s_threshold_ms) {
        fprintf(stderr, "[WAITS] flip %u took %.1f ms:\n", flip, gap);
        for (i = 0; i < WR_THREADS && s_thr[i].tid; i++) {
            WrSite all[WR_SITES];
            int n = 0;
            double tot = 0;
            for (j = 0; j < WR_SITES && s_thr[i].site[j].n; j++) {
                all[n++] = s_thr[i].site[j];
                tot += s_thr[i].site[j].ns / 1e6;
            }
            if (tot < 1.0)
                continue;
            qsort(all, (size_t)n, sizeof all[0], by_ns_desc);
            fprintf(stderr, "  tid %lu waited %.1f ms:", s_thr[i].tid, tot);
            for (j = 0; j < n && j < 6; j++)
                fprintf(stderr, " %s 0x%08X %.1fms/%u", kind_name(all[j].kind),
                        all[j].site, all[j].ns / 1e6, all[j].n);
            fprintf(stderr, "\n");
        }
        fflush(stderr);
    }
    for (i = 0; i < WR_THREADS; i++)
        memset(s_thr[i].site, 0, sizeof s_thr[i].site);
}
