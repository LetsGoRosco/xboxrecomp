/*
 * kernel_signpost.h - Instruments intervals around the places a guest thread
 * sleeps (macOS os_signpost; nothing elsewhere).
 *
 * The Time Profiler samples what runs. A guest thread that waits -- a lowered
 * spin loop asleep, a queue for the guest CPU, a kernel wait -- is not
 * sampled, so a slow frame where the game thread mostly waited showed only
 * its few running milliseconds (Halo 2's checkpoint reload: 19 ms of 100).
 * These intervals, on the "xboxrecomp" Points of Interest log, put each
 * wait on the timeline with what it waited for. os_signpost_enabled() makes
 * them free unless Instruments is recording. The interval names are
 * literals (os_signpost requires it); the message says which site.
 */
#ifndef KERNEL_SIGNPOST_H
#define KERNEL_SIGNPOST_H

#include <stdint.h>

/* RECOMP_DEBUG=wait_report=<ms> (kernel_wait_report.c): each wait below also
 * adds its time to the calling thread's table, by kind and site, and a flip
 * longer than ms prints the tables. g_xbox_wait_report is 0 when off, so a
 * wait pays one load for it. */
enum { XBOX_WAIT_SPIN = 0, XBOX_WAIT_CPU = 1, XBOX_WAIT_KERNEL = 2 };
extern volatile int g_xbox_wait_report;
uint64_t xbox_wait_report_now(void);
void xbox_wait_report_add(int kind, uint32_t site, uint64_t ns);
void xbox_wait_report_flip(uint32_t flip);

#if defined(__APPLE__)
#include <os/log.h>
#include <os/signpost.h>

static inline os_log_t xbox_sp_log(void)
{
    static os_log_t log;
    if (!log)
        log = os_log_create("xboxrecomp", OS_LOG_CATEGORY_POINTS_OF_INTEREST);
    return log;
}

#define XBOX_SP_BEGIN(name, kind, site, fmt, ...)                           \
    os_signpost_id_t xbox_sp_id_ = OS_SIGNPOST_ID_NULL;                     \
    int xbox_sp_kind_ = (kind); uint32_t xbox_sp_site_ = (uint32_t)(site);  \
    uint64_t xbox_sp_t0_ = g_xbox_wait_report ? xbox_wait_report_now() : 0; \
    if (os_signpost_enabled(xbox_sp_log())) {                               \
        xbox_sp_id_ = os_signpost_id_generate(xbox_sp_log());               \
        os_signpost_interval_begin(xbox_sp_log(), xbox_sp_id_, name, fmt,   \
                                   ##__VA_ARGS__);                          \
    }
#define XBOX_SP_END(name)                                                   \
    do {                                                                    \
        if (xbox_sp_id_ != OS_SIGNPOST_ID_NULL)                             \
            os_signpost_interval_end(xbox_sp_log(), xbox_sp_id_, name);     \
        if (xbox_sp_t0_)                                                    \
            xbox_wait_report_add(xbox_sp_kind_, xbox_sp_site_,              \
                                 xbox_wait_report_now() - xbox_sp_t0_);     \
    } while (0)
#else
#define XBOX_SP_BEGIN(name, kind, site, fmt, ...)                           \
    int xbox_sp_kind_ = (kind); uint32_t xbox_sp_site_ = (uint32_t)(site);  \
    uint64_t xbox_sp_t0_ = g_xbox_wait_report ? xbox_wait_report_now() : 0;
#define XBOX_SP_END(name)                                                   \
    do {                                                                    \
        if (xbox_sp_t0_)                                                    \
            xbox_wait_report_add(xbox_sp_kind_, xbox_sp_site_,              \
                                 xbox_wait_report_now() - xbox_sp_t0_);     \
    } while (0)
#endif

#endif /* KERNEL_SIGNPOST_H */
