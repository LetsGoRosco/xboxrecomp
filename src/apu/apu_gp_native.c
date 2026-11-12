/*
 * The APU's GP in C. See apu_gp_native.h, and etm's docs/gp-effects-chain.md
 * for the program this follows (Enter the Matrix's matrixDSP.bin on
 * DirectSound 5344: P:0x000-0x170 the kernel, P:0x171-0x71E the effects).
 *
 * Arithmetic is the DSP56300's, on 56-bit accumulators held as int64 in
 * units of 2^-47 (a2:a1:a0): a memory word w loads as sext24(w) << 24, a
 * product of two words is (x * y) << 1, a move to memory takes a1 through
 * the limiter. The effects run with SR's SM bit set (arithmetic saturation
 * at 48 bits), so every result here saturates there.
 */
#include "apu_gp_native.h"
#include "apu_gpx.h"
#include "recomp_env.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define X_WORDS        0x1000u
#define MIXBUF_ALIAS   0xC00u   /* the mixbuf: X:0xC00-0xFFF ... */
#define MIXBUF_BASE    0x1400u  /* ... also seen at X:0x1400-0x17FF */
#define MIXBUF_WORDS   0x400u

/* The kernel's command block, six words at scratch 0x800, read into X:0-5:
 * +0 state offset (from X:0x80), +1 code words, +2 scratch source of the
 * state, +3 state words, +4 flags (bit 0: code, bit 1: state; the kernel
 * clears it and writes the block back, which is DirectSound's doorbell at
 * scratch 0x810). */
#define CMD_SCRATCH    0x800u
#define CMD_WORDS      6u
#define CODE_SCRATCH   0x818u
#define CODE_P         0x171u
#define CODE_MAX       (0x800u - CODE_P)
#define STATE_BASE     0x80u

/* The output: six 0x800-byte circular rings at scratch 0x8000 + 0x800*ch,
 * one 32-word frame each, from mixbins 0, 2, 1, 4, 5, 3 (AC-3 order: L, C,
 * R, Ls, Rs, LFE); then the FX sends (bins 11-30) to scratch 0xB000. */
#define RING_BASE      0x8000u
#define RING_BYTES     0x800u
#define FXSEND_X       0x1560u
#define FXSEND_WORDS   0x280u
#define FXSEND_SCRATCH 0xB000u
static const uint8_t k_ring_bin[6] = { 0, 2, 1, 4, 5, 3 };

/* SR as the mixers' stale x0 reads it: EMR CP1/CP0 (0xC0), MR I1/I0 (0x03),
 * then the CCR this file tracks. */
#define SR_HIGH        0xC00300u
#define SR_SM          0x100000u
enum { CCR_C = 0x01, CCR_V = 0x02, CCR_Z = 0x04, CCR_N = 0x08,
       CCR_U = 0x10, CCR_E = 0x20, CCR_L = 0x40, CCR_S = 0x80 };

typedef enum {
    FX_I3DL2,           /* stock I3DL2 reverb (not yet run: TODO) */
    FX_XTALK,           /* stock 4-channel crosstalk */
    FX_MIX2,            /* stock 2-input mixer */
} FxKind;

typedef struct {
    FxKind kind;
    uint32_t state;     /* its state block in X */
} Fx;

typedef struct {
    const char *name;
    uint32_t words;     /* code words downloaded */
    uint64_t hash;      /* FNV-1a over those words, 24 bits each */
    int nfx;
    Fx fx[8];
} Image;

/* Enter the Matrix (Europe), matrixDSP.bin. */
static const Image k_images[] = {
    { "etm matrixDSP", 0x5AE, 0x5239d79240291e08ull, 5, {
        { FX_I3DL2, 0x080 }, { FX_XTALK, 0x2AA }, { FX_MIX2, 0x307 },
        { FX_MIX2, 0x323 }, { FX_MIX2, 0x33F } } },
};

#define SCR_PAGE      4096u
#define SCR_PAGES     256u      /* 1 MB of scratch, more than the GP maps */

struct ApuGpNative {
    ApuGpnScratchRw rw;
    void *opaque;
    /* A translated program (apu_gpx.h), when one matches the kernel. */
    GpxCore *gpx;
    const GpxProgram *prog;
    bool gpx_off;               /* RECOMP_DEBUG=apu_gpx=0 */
    unsigned budget_logs;
    bool gpx_failed;
    bool commit;                /* this frame's */
    /* Shadow: the native core's scratch writes go to private pages, and
     * its acknowledgement of DirectSound's command is kept here, not
     * written, so the real GP still answers it. */
    uint8_t *page[SCR_PAGES];
    uint32_t consumed[CMD_WORDS];
    bool have_consumed;
    uint32_t x[X_WORDS];
    uint32_t code[CODE_MAX];
    uint32_t code_words;
    const Image *image;
    bool loaded;
    uint32_t ring_off;  /* the six rings move together */
    uint8_t ccr;
    bool warned;
    bool pending;       /* shadow: last_cmd seen and not yet answered */
    uint32_t last_cmd[CMD_WORDS];
};

/* ---- DSP56300 arithmetic ---- */

#define ACC_MAX48 ((int64_t)0x7FFFFFFFFFFF)
#define ACC_MIN48 (-(int64_t)0x800000000000)
#define M56 ((uint64_t)0x00FFFFFFFFFFFFFF)

static inline int64_t sx24(uint32_t w)
{
    return (int64_t)((int32_t)(w << 8) >> 8);
}

static inline int64_t ld(uint32_t w)        /* move x:,a */
{
    return sx24(w) * ((int64_t)1 << 24);
}

static inline int64_t sat48(int64_t a)      /* SR.SM */
{
    return a > ACC_MAX48 ? ACC_MAX48 : a < ACC_MIN48 ? ACC_MIN48 : a;
}

static inline int64_t mpy(uint32_t x, uint32_t y)
{
    return sat48(sx24(x) * sx24(y) * 2);
}

/* move a,x: through the limiter: a1, or the extreme of its sign. */
static inline uint32_t st(ApuGpNative *g, int64_t a)
{
    if (a > ACC_MAX48) {
        g->ccr |= CCR_L;
        return 0x7FFFFF;
    }
    if (a < ACC_MIN48) {
        g->ccr |= CCR_L;
        return 0x800000;
    }
    return (uint32_t)(a >> 24) & 0xFFFFFF;
}

/* The CCR an add leaves (S and L sticky; no scaling mode). */
static void ccr_add(ApuGpNative *g, int64_t d, int64_t s, int64_t r)
{
    uint8_t c = g->ccr & (CCR_S | CCR_L);
    if (((((uint64_t)d & M56) + ((uint64_t)s & M56)) >> 56) & 1)
        c |= CCR_C;
    if (r == 0)
        c |= CCR_Z;
    if (r < 0)
        c |= CCR_N;
    if (((r >> 47) & 1) == ((r >> 46) & 1))
        c |= CCR_U;
    if (r > ACC_MAX48 || r < ACC_MIN48)
        c |= CCR_E;
    g->ccr = c;
}

/* ---- X memory ---- */

static inline uint32_t *xp(ApuGpNative *g, uint32_t a)
{
    if (a >= MIXBUF_BASE && a < MIXBUF_BASE + MIXBUF_WORDS)
        a -= MIXBUF_BASE - MIXBUF_ALIAS;
    return &g->x[a & (X_WORDS - 1)];
}

#define X(a) (*xp(g, (a)))

/* ---- the effects ---- */

/* XTalk, P:0x62B: two passes over a pair of channels, each sample
 *   sB = y0*sB + y1*D1[n]     sA = y0*sA + y1*D2[n]
 *   a  = sA + in1             b  = sB + in2
 *   D1[n] = a                 D2[n] = b
 *   out1 += a (in1 with bit 1 clear), out2 += b (in2)
 * with n going round 11 slots. State: +5..+8 inputs, +9..+12 outputs,
 * +13/+25 and +38/+50 the delay lines, +24/+36 and +49/+61 the one-poles,
 * +37/+62 n, +63/+64 y0/y1 (written on the first frame). */
static void xtalk_pass(ApuGpNative *g, uint32_t r5, int pass)
{
    uint32_t r1 = X(r5 + (pass ? 7 : 5)), r2 = X(r5 + (pass ? 8 : 6));
    uint32_t r6 = X(r5 + (pass ? 11 : 9)), r7 = X(r5 + (pass ? 12 : 10));
    uint32_t r3 = r5 + (pass ? 0x26 : 0x0D), r4 = r5 + (pass ? 0x32 : 0x19);
    uint32_t r0 = r5 + (pass ? 49 : 24);
    uint32_t nslot = r5 + (pass ? 62 : 37);
    uint32_t n = X(nslot);
    uint32_t y1 = X(r5 + 64);

    for (int i = 0; i < APU_GPN_SAMPLES; i++) {
        uint32_t y0 = X(r5 + 63);
        int64_t b = sat48(mpy(y0, X(r0)) + mpy(y1, X(r3 + n)));
        X(r0) = st(g, b);
        int64_t a = sat48(mpy(y0, X(r0 + 12)) + mpy(y1, X(r4 + n)));
        X(r0 + 12) = st(g, a);
        uint32_t x0 = X(r1++), x1 = X(r2++);
        a = sat48(a + ld(x0));
        b = sat48(b + ld(x1));
        X(r3 + n) = st(g, a);
        X(r4 + n) = st(g, b);
        if (!(X(r5 + 4) & 2)) {
            a = ld(x0);
            b = ld(x1);
        }
        a = sat48(a + ld(X(r6)));
        b = sat48(b + ld(X(r7)));
        X(r6++) = st(g, a);
        X(r7++) = st(g, b);
        /* move n3,a / add #1,a / cmp #11,a / blt / clr a */
        int64_t an = ld(n) + ld(1);
        int64_t k = ld(11);
        ccr_add(g, an, -k, an - k);
        if (an - k >= 0) {
            an = 0;
            g->ccr = (g->ccr & (CCR_S | CCR_L | CCR_C)) | CCR_Z | CCR_U;
        }
        n = st(g, an);
    }
    X(nslot) = n;
}

static void fx_xtalk(ApuGpNative *g, uint32_t r5)
{
    uint32_t f = X(r5 + 4);
    if (!(f & 1)) {
        X(r5 + 4) = f | 1;
        X(r5 + 63) = 0x7645A1;
        X(r5 + 64) = 0xF88313;
    }
    xtalk_pass(g, r5, 0);
    xtalk_pass(g, r5, 1);
    /* bclr #20,sr: C takes the bit, which was set */
    g->ccr |= CCR_C;
}

/* The 2-input mixer, P:0x69F: out = (bit 1 ? out : 0) + y0*in0 + y1*in1.
 * With bit 1 clear the program skips the load of in0 too, so the product
 * takes the x0 the entry left: SR, read with movec just before. That is a
 * constant y0 * SR on the output (Enter the Matrix's LFE: about -0.19). */
static void fx_mix2(ApuGpNative *g, uint32_t r5)
{
    uint32_t f = X(r5 + 4);
    if (!(f & 1))
        X(r5 + 4) = f | 1;
    uint32_t x0 = (SR_HIGH | g->ccr) | SR_SM;     /* movec sr,x0; bset */
    uint32_t r0 = X(r5 + 5), r1 = X(r5 + 6), r2 = X(r5 + 7);
    uint32_t y0 = X(r5 + 8), y1 = X(r5 + 9);

    for (int i = 0; i < APU_GPN_SAMPLES; i++) {
        int64_t b = 0;
        if (X(r5 + 4) & 2) {
            b = ld(X(r2));
            x0 = X(r0++);
        }
        int64_t a = mpy(y0, x0);
        uint32_t x1 = X(r1++);
        int64_t r = sat48(a + b);
        ccr_add(g, b, a, a + b);
        b = r;
        a = mpy(y1, x1);
        r = sat48(a + b);
        ccr_add(g, b, a, a + b);
        b = r;
        X(r2++) = st(g, b);
    }
}

/* The I3DL2 reverb, P:0x171. TODO: not run yet; the first-frame flag and
 * the parameter flag (bit 2) are handled so the state matches otherwise. */
static void fx_i3dl2(ApuGpNative *g, uint32_t r5)
{
    uint32_t f = X(r5 + 4);
    if (!(f & 1))
        f |= 1;
    f &= ~4u;
    X(r5 + 4) = f;
}

/* ---- scratch as the translated core sees it ---- */

static void real_rw(ApuGpNative *g, uint8_t *p, uint32_t a, size_t n, bool d)
{
    g->rw(g->opaque, p, a, n, d);
}

static void gpx_scratch(void *opaque, uint8_t *ptr, uint32_t addr, size_t len,
                        bool dir)
{
    ApuGpNative *g = opaque;
    if (g->commit) {
        real_rw(g, ptr, addr, len, dir);
        return;
    }
    /* the command block: reads through, the native ack withheld */
    if (addr == CMD_SCRATCH && len == CMD_WORDS * 4) {
        uint32_t real[CMD_WORDS];
        real_rw(g, (uint8_t *)real, addr, len, false);
        if (dir) {
            memcpy(g->consumed, real, sizeof real);
            g->have_consumed = true;
            return;
        }
        if (g->have_consumed) {
            if (!real[4] || memcmp(real, g->consumed, sizeof real))
                g->have_consumed = false;   /* answered, or a new one */
            else
                real[4] = 0;
        }
        memcpy(ptr, real, len);
        return;
    }
    while (len > 0) {
        uint32_t pg = addr / SCR_PAGE, off = addr % SCR_PAGE;
        size_t n = SCR_PAGE - off;
        if (n > len)
            n = len;
        if (pg >= SCR_PAGES) {
            if (!dir)
                memset(ptr, 0, n);
        } else if (dir) {
            if (!g->page[pg]) {
                g->page[pg] = malloc(SCR_PAGE);
                if (!g->page[pg])
                    return;
                real_rw(g, g->page[pg], pg * SCR_PAGE, SCR_PAGE, false);
            }
            memcpy(g->page[pg] + off, ptr, n);
        } else if (g->page[pg]) {
            memcpy(ptr, g->page[pg] + off, n);
        } else {
            real_rw(g, ptr, addr, n, false);
        }
        ptr += n;
        addr += (uint32_t)n;
        len -= n;
    }
}

static void drop_pages(ApuGpNative *g)
{
    for (unsigned i = 0; i < SCR_PAGES; i++) {
        free(g->page[i]);
        g->page[i] = NULL;
    }
    g->have_consumed = false;
}

/* One frame of the translated program: the VP's mixbins in, START_FRAME,
 * run to the frame halt, the mixbuf out. */
static void gpx_frame(ApuGpNative *g, uint32_t *mixbuf, bool commit)
{
    GpxCore *c = g->gpx;
    g->commit = commit;
    memcpy(&c->X[MIXBUF_ALIAS], mixbuf, MIXBUF_WORDS * 4);
    c->interrupts |= 1u << 1;
    c->budget = 1000000;
    c->p_written = false;
    int rc = g->prog->run(c);
    /* -1: the budget ran out; like the JIT at its cycle cap, the next frame
     * goes on from c->pc. -2: a PC outside the translation: stop. */
    if (rc == -1 && g->budget_logs++ < 4)
        fprintf(stderr, "[APU GPX] %s: frame budget spent at P:%04X\n",
                g->prog->name, c->pc);
    if (rc == -2 && !g->gpx_failed) {
        g->gpx_failed = true;
        fprintf(stderr, "[APU GPX] %s: P:%04X is outside the translation;"
                " the GP passes its bins through from here\n", g->prog->name,
                c->pc);
    }
    if (c->p_written)
        fprintf(stderr, "[APU GPX] effects code downloaded: %s\n",
                c->code_ok ? "the translated program's"
                           : "not the translated program's; effects off");
    memcpy(mixbuf, &c->X[MIXBUF_ALIAS], MIXBUF_WORDS * 4);
}

/* ---- the kernel ---- */

static uint64_t fnv(const uint32_t *w, uint32_t n)
{
    uint64_t h = 0xcbf29ce484222325ull;
    for (uint32_t i = 0; i < n; i++)
        for (int b = 0; b < 3; b++) {
            h ^= (w[i] >> (8 * b)) & 0xFF;
            h *= 0x100000001b3ull;
        }
    return h;
}

static void scratch_words(ApuGpNative *g, uint32_t *w, uint32_t addr,
                          uint32_t n, bool dir)
{
    g->rw(g->opaque, (uint8_t *)w, addr, n * 4, dir);
}

static void load_code(ApuGpNative *g, uint32_t words)
{
    if (words > CODE_MAX)
        words = CODE_MAX;
    scratch_words(g, g->code, CODE_SCRATCH, words, false);
    for (uint32_t i = 0; i < words; i++)
        g->code[i] &= 0xFFFFFF;
    g->code_words = words;
    g->loaded = true;
    uint64_t h = fnv(g->code, words);
    g->image = NULL;
    for (size_t i = 0; i < sizeof k_images / sizeof k_images[0]; i++)
        if (k_images[i].words == words && k_images[i].hash == h)
            g->image = &k_images[i];
    fprintf(stderr, "[APU GPN] effects image: %u words, hash %016llx: %s\n",
            words, (unsigned long long)h,
            g->image ? g->image->name : "unknown");
}

/* P:0x05C: DirectSound's command. */
static void command(ApuGpNative *g, bool commit)
{
    uint32_t cmd[CMD_WORDS];
    scratch_words(g, cmd, CMD_SCRATCH, CMD_WORDS, false);
    for (unsigned i = 0; i < CMD_WORDS; i++)
        X(i) = cmd[i] & 0xFFFFFF;
    uint32_t flags = X(4);
    if (!flags) {
        g->pending = false;
        return;
    }
    /* Shadow: the block stays set until the real GP answers it; take it
     * once. */
    if (!commit) {
        if (g->pending && !memcmp(cmd, g->last_cmd, sizeof cmd))
            return;
        g->pending = true;
        memcpy(g->last_cmd, cmd, sizeof cmd);
    }
    if (flags & 1) {
        /* clear X:0x80 for 0xF80 words (the mixbuf with it) */
        memset(&g->x[STATE_BASE], 0, (X_WORDS - STATE_BASE) * 4);
        load_code(g, X(1));
    }
    if (flags & 2) {
        uint32_t off = STATE_BASE + X(0), n = X(3);
        if (off + n > X_WORDS)
            n = off < X_WORDS ? X_WORDS - off : 0;
        uint32_t buf[X_WORDS];
        scratch_words(g, buf, X(2), n, false);
        for (uint32_t i = 0; i < n; i++)
            g->x[off + i] = buf[i] & 0xFFFFFF;
        X(4) = 0;
        if (commit) {
            uint32_t back[CMD_WORDS];
            for (unsigned i = 0; i < CMD_WORDS; i++)
                back[i] = X(i);
            scratch_words(g, back, CMD_SCRATCH, CMD_WORDS, true);
        }
    }
}

static void output(ApuGpNative *g)
{
    for (int ch = 0; ch < 6; ch++)
        scratch_words(g, xp(g, MIXBUF_ALIAS + k_ring_bin[ch] * 32),
                      RING_BASE + RING_BYTES * ch + g->ring_off,
                      APU_GPN_SAMPLES, true);
    g->ring_off = (g->ring_off + APU_GPN_SAMPLES * 4) % RING_BYTES;
    scratch_words(g, xp(g, FXSEND_X), FXSEND_SCRATCH, FXSEND_WORDS, true);
}

void apu_gpn_frame(ApuGpNative *g, uint32_t mixbuf[APU_GPN_BINS *
                                                   APU_GPN_SAMPLES],
                   bool commit)
{
    if (g->prog) {
        if (!g->gpx_failed)
            gpx_frame(g, mixbuf, commit);
        return;
    }
    memcpy(&g->x[MIXBUF_ALIAS], mixbuf, MIXBUF_WORDS * 4);
    command(g, commit);
    if (g->image) {
        for (int i = 0; i < g->image->nfx; i++) {
            const Fx *fx = &g->image->fx[i];
            switch (fx->kind) {
            case FX_I3DL2: fx_i3dl2(g, fx->state); break;
            case FX_XTALK: fx_xtalk(g, fx->state); break;
            case FX_MIX2:  fx_mix2(g, fx->state);  break;
            }
        }
    } else if (g->loaded && !g->warned) {
        g->warned = true;
        fprintf(stderr, "[APU GPN] unknown effects image: bins pass"
                " through\n");
    }
    if (commit)
        output(g);
    memcpy(mixbuf, &g->x[MIXBUF_ALIAS], MIXBUF_WORDS * 4);
}

void apu_gpn_reset(ApuGpNative *g)
{
    memset(g->x, 0, sizeof g->x);
    g->code_words = 0;
    g->image = NULL;
    g->loaded = false;
    g->ring_off = 0;
    g->ccr = CCR_S;
    g->warned = false;
    g->pending = false;
    /* A translated program, if the boot ROM's copy (the first 0x800 words
     * of scratch into P) is a kernel one was made from. */
    g->prog = NULL;
    g->gpx_failed = false;
    drop_pages(g);
    if (g->gpx && !g->gpx_off) {
        apu_gpx_reset(g->gpx);
        uint32_t boot[0x800];
        g->rw(g->opaque, (uint8_t *)boot, 0, sizeof boot, false);
        for (unsigned i = 0; i < 0x800; i++)
            g->gpx->P[i] = boot[i] & 0xFFFFFF;
        g->prog = apu_gpx_find_kernel(g->gpx->P, 0x800);
        if (g->prog) {
            g->gpx->code_at = g->prog->code_at;
            g->gpx->code_words = g->prog->code_words;
            g->gpx->code_hash = g->prog->code_hash;
        }
        fprintf(stderr, "[APU GPX] GP kernel %016llx: %s\n",
                (unsigned long long)apu_gpx_hash(g->gpx->P, 0x171),
                g->prog ? g->prog->name : "no translated program; the"
                          " hand-written effects");
    }
}

ApuGpNative *apu_gpn_create(ApuGpnScratchRw rw, void *opaque)
{
    ApuGpNative *g = calloc(1, sizeof *g);
    if (!g)
        return NULL;
    g->rw = rw;
    g->opaque = opaque;
    const char *e = recomp_env(RENV_APU_GPX);
    g->gpx_off = e && !strcmp(e, "0");
    g->gpx = calloc(1, sizeof *g->gpx);
    if (g->gpx) {
        g->gpx->rw = gpx_scratch;
        g->gpx->opaque = g;
    }
    apu_gpn_reset(g);
    return g;
}

void apu_gpn_destroy(ApuGpNative *g)
{
    drop_pages(g);
    free(g->gpx);
    free(g);
}

bool apu_gpn_known(const ApuGpNative *g)
{
    return g->prog ? !g->gpx_failed : g->image != NULL;
}

bool apu_gpn_loaded(const ApuGpNative *g)
{
    return g->prog ? true : g->loaded;
}

void apu_gpn_set_window(ApuGpNative *g, uint8_t *window)
{
    if (g->gpx)
        apu_gpx_set_memory(g->gpx, window);
}

void apu_gpn_poke_x(ApuGpNative *g, uint32_t addr, uint32_t value)
{
    if (g->prog && addr < GPX_X_WORDS)
        g->gpx->X[addr] = value & 0xFFFFFF;
}

uint32_t apu_gpn_x(const ApuGpNative *g, uint32_t addr)
{
    if (g->prog)
        return gpx_xr(g->gpx, addr);
    return *xp((ApuGpNative *)g, addr);
}
