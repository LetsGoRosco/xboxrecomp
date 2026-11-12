/*
 * The runtime for a GP program translated to C by tools/dsp2c.py: one
 * DSP56300 core's registers and memory, the peripheral page (the frame
 * halt, the interrupt status, the DMA engine), and the arithmetic the
 * translated instructions call, all inline.
 *
 * Semantics: the DSP56300 family manual, as xemu's interpreter implements
 * it (hw/xbox/mcpx/apu/dsp/interp), except where the dsp56300 JIT this
 * replaces differs and the JIT is the reference: SR's upper byte survives
 * an ALU instruction and a DO loop's end restores only LF. The DMA engine
 * and the peripheral page follow apu_dsp_emu.c.
 *
 * Accumulators are int64 holding the 56-bit value sign-extended (a2:a1:a0
 * = bits 55..48, 47..24, 23..0); data registers and memory words are the
 * 24-bit pattern in a uint32.
 */
#ifndef APU_GPX_H
#define APU_GPX_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GPX_X_WORDS   0x1000u
#define GPX_Y_WORDS   0x800u
#define GPX_P_WORDS   0x1000u
#define GPX_STACK     16

/* SR bits */
#define GPX_C   (1u << 0)
#define GPX_V   (1u << 1)
#define GPX_Z   (1u << 2)
#define GPX_N   (1u << 3)
#define GPX_U   (1u << 4)
#define GPX_E   (1u << 5)
#define GPX_L   (1u << 6)
#define GPX_S   (1u << 7)
#define GPX_SM  (1u << 20)
#define GPX_LF  (1u << 15)
#define GPX_SR_RESET 0xC00300u

typedef void (*GpxScratchRw)(void *opaque, uint8_t *ptr, uint32_t addr,
                             size_t len, bool dir);

typedef struct GpxDma {
    uint32_t configuration, control, start_block, next_block;
    bool eol;
    uint32_t read_count;
} GpxDma;

typedef struct GpxCore {
    int64_t a, b;
    uint32_t x0, x1, y0, y1;
    uint32_t r[8], n[8], m[8];
    uint32_t sr, la, lc;
    uint32_t ssh[GPX_STACK], ssl[GPX_STACK];
    int sp;
    uint32_t pc;                /* where the next run starts */
    /* Memory: the core's own (Xown...) or the MCPX window's pages, which
     * the title reads and writes too (one u32 a word, the top byte not
     * the core's: reads mask it). The mixbuf is X:0xC00-0xFFF. */
    uint32_t *X, *Y, *P;
    uint32_t interrupts;
    GpxDma dma;
    bool halt;
    bool p_written;             /* a DMA wrote P this run */
    /* The downloaded effects code is the translated program's: until a DMA
     * puts it there (its hash checked as the DMA ends), a call to code_at
     * returns at once instead of running a translation of code that is not
     * in P. */
    bool code_ok;
    uint32_t code_at, code_words;
    uint64_t code_hash;
    unsigned refused_logs;
    int64_t budget;             /* branches and loop turns left this run */
    GpxScratchRw rw;
    void *opaque;
    uint32_t words[0x1800];     /* DMA staging */
    uint8_t xfer[0x1800 * 4];
    uint32_t Xown[GPX_X_WORDS], Yown[GPX_Y_WORDS], Pown[GPX_P_WORDS];
} GpxCore;

/* A translated program: run from c->pc until the frame halt (0), or until
 * the budget runs out (-1); either way c->pc is where to go on. */
typedef int (*GpxRunFn)(GpxCore *c);

typedef struct GpxProgram {
    const char *name;
    uint32_t kernel_words;      /* P:0 up to the downloaded code */
    uint64_t kernel_hash;       /* FNV-1a, 3 bytes a word */
    uint32_t code_at, code_words;
    uint64_t code_hash;
    GpxRunFn run;
} GpxProgram;

/* The game registers what it translated (before the APU starts). */
void apu_gpx_register(const GpxProgram *p);
const GpxProgram *apu_gpx_find_kernel(const uint32_t *p, uint32_t words);
uint64_t apu_gpx_hash(const uint32_t *w, uint32_t n);
/* The hash of P over [code_at, code_at + code_words), 24 bits a word. */
uint64_t apu_gpx_hash_p(const GpxCore *c);

/* Registers and peripherals to their reset state; memory is kept. */
void apu_gpx_reset(GpxCore *c);
/* Run on a window (APU_DSP_WINDOW layout: X at +0, Y at +0x6000, P at
 * +0xA000), or NULL for the core's own memory. */
void apu_gpx_set_memory(GpxCore *c, uint8_t *window);
void apu_gpx_periph_write(GpxCore *c, uint32_t addr, uint32_t v);
uint32_t apu_gpx_periph_read(GpxCore *c, uint32_t addr);

/* ---- arithmetic ---- */

#define GPX_M56 ((uint64_t)0x00FFFFFFFFFFFFFFull)

static inline int64_t gpx_n56(int64_t v)
{
    return (int64_t)((uint64_t)v << 8) >> 8;
}

static inline int64_t gpx_sx24(uint32_t w)
{
    return (int64_t)((int32_t)(w << 8) >> 8);
}

static inline int64_t gpx_ld(uint32_t w)        /* 24 bits into a1 */
{
    return gpx_sx24(w) * ((int64_t)1 << 24);
}

static inline void gpx_eunz(GpxCore *c, int64_t r)
{
    uint32_t sr = c->sr & ~(GPX_E | GPX_U | GPX_N | GPX_Z);
    uint32_t a2 = (uint32_t)(r >> 48) & 0xFF, a1 = (uint32_t)(r >> 24) &
                  0xFFFFFF;
    uint32_t e = (a2 << 1) | (a1 >> 23);
    if (e != 0 && e != 0x1FF)
        sr |= GPX_E;
    if ((a1 & 0xC00000) == 0 || (a1 & 0xC00000) == 0xC00000)
        sr |= GPX_U;
    if (r < 0)
        sr |= GPX_N;
    if (r == 0)
        sr |= GPX_Z;
    c->sr = sr;
}

/* SR.SM: results saturate at 48 bits (sets V and L). */
static inline int64_t gpx_sat(GpxCore *c, int64_t r, uint32_t *flags)
{
    if (!(c->sr & GPX_SM))
        return r;
    const int64_t hi = ((int64_t)1 << 47) - 1, lo = -((int64_t)1 << 47);
    if (r > hi || r < lo) {
        *flags |= GPX_V | GPX_L;
        return r > hi ? hi : lo;
    }
    return r;
}

/* d + s: C (carry out of bit 55), V (overflow), L |= V. */
static inline int64_t gpx_add(GpxCore *c, int64_t d, int64_t s,
                              uint32_t *flags)
{
    uint64_t u = ((uint64_t)d & GPX_M56) + ((uint64_t)s & GPX_M56);
    int64_t r = gpx_n56((int64_t)u);
    uint32_t f = (uint32_t)((u >> 56) & 1) ? GPX_C : 0;
    if ((d < 0) == (s < 0) && (r < 0) != (d < 0))
        f |= GPX_V | GPX_L;
    r = gpx_sat(c, r, &f);
    *flags = f;
    return r;
}

static inline int64_t gpx_sub(GpxCore *c, int64_t d, int64_t s,
                              uint32_t *flags)
{
    uint64_t u = ((uint64_t)d & GPX_M56) - ((uint64_t)s & GPX_M56);
    int64_t r = gpx_n56((int64_t)u);
    uint32_t f = (uint32_t)((u >> 56) & 1) ? GPX_C : 0;
    if ((d < 0) != (s < 0) && (r < 0) != (d < 0))
        f |= GPX_V | GPX_L;
    r = gpx_sat(c, r, &f);
    *flags = f;
    return r;
}

static inline int64_t gpx_mul(uint32_t x, uint32_t y, bool neg)
{
    int64_t p = gpx_sx24(x) * gpx_sx24(y) * 2;
    return gpx_n56(neg ? -p : p);
}

/* Convergent rounding at bit 23 (no scaling); the add's flags are not
 * the instruction's (xemu's dsp_rnd56). */
static inline int64_t gpx_rnd(GpxCore *c, int64_t v)
{
    uint32_t f;
    int64_t r = gpx_add(c, v, (int64_t)1 << 23, &f);
    if ((r & 0xFFFFFF) == 0)
        r &= ~((int64_t)1 << 24);
    return r & ~(int64_t)0xFFFFFF;
}

/* An accumulator read as a 24-bit word: the limiter, and the sticky S bit
 * (no scaling: set when bits 46 and 45 differ, a value of a quarter or
 * more), which xemu leaves out and the JIT has. */
static inline uint32_t gpx_acc24(GpxCore *c, int64_t v)
{
    const int64_t hi = ((int64_t)1 << 47) - 1, lo = -((int64_t)1 << 47);
    if (((v >> 46) ^ (v >> 45)) & 1)
        c->sr |= GPX_S;
    if (v > hi) {
        c->sr |= GPX_L;
        return 0x7FFFFF;
    }
    if (v < lo) {
        c->sr |= GPX_L;
        return 0x800000;
    }
    return (uint32_t)(v >> 24) & 0xFFFFFF;
}

static inline uint32_t gpx_a2(int64_t v) { return (uint32_t)(v >> 48) & 0xFF; }
static inline uint32_t gpx_a1(int64_t v) { return (uint32_t)(v >> 24) & 0xFFFFFF; }
static inline uint32_t gpx_a0(int64_t v) { return (uint32_t)v & 0xFFFFFF; }
static inline int64_t gpx_set_a1(int64_t v, uint32_t w)
{
    return gpx_n56((v & ~((int64_t)0xFFFFFF << 24)) |
                   ((int64_t)(w & 0xFFFFFF) << 24));
}
static inline int64_t gpx_set_a0(int64_t v, uint32_t w)
{
    return (v & ~(int64_t)0xFFFFFF) | (w & 0xFFFFFF);
}

/* ---- memory ---- */

static inline uint32_t gpx_xr(GpxCore *c, uint32_t a)
{
    a &= 0xFFFFFF;
    if (a < GPX_X_WORDS)
        return c->X[a] & 0xFFFFFF;
    if (a >= 0x1400 && a < 0x1800)
        return c->X[a - 0x800] & 0xFFFFFF;
    if (a >= 0xFFFF80)
        return apu_gpx_periph_read(c, a);
    return 0;
}

static inline void gpx_xw(GpxCore *c, uint32_t a, uint32_t v)
{
    a &= 0xFFFFFF;
    v &= 0xFFFFFF;
    if (a < GPX_X_WORDS)
        c->X[a] = v;
    else if (a >= 0x1400 && a < 0x1800)
        c->X[a - 0x800] = v;
    else if (a >= 0xFFFF80)
        apu_gpx_periph_write(c, a, v);
}

static inline uint32_t gpx_yr(GpxCore *c, uint32_t a)
{
    a &= 0xFFFFFF;
    return a < GPX_Y_WORDS ? c->Y[a] & 0xFFFFFF : 0;
}

static inline void gpx_yw(GpxCore *c, uint32_t a, uint32_t v)
{
    a &= 0xFFFFFF;
    if (a < GPX_Y_WORDS)
        c->Y[a] = v & 0xFFFFFF;
}

static inline uint32_t gpx_pr(GpxCore *c, uint32_t a)
{
    return c->P[a & (GPX_P_WORDS - 1)] & 0xFFFFFF;
}

/* ---- the system stack ---- */

static inline void gpx_push(GpxCore *c, uint32_t hi, uint32_t lo)
{
    c->sp = (c->sp + 1) & (GPX_STACK - 1);
    c->ssh[c->sp] = hi;
    c->ssl[c->sp] = lo;
}

static inline void gpx_pop(GpxCore *c, uint32_t *hi, uint32_t *lo)
{
    *hi = c->ssh[c->sp];
    *lo = c->ssl[c->sp];
    c->sp = (c->sp - 1) & (GPX_STACK - 1);
}

#ifdef __cplusplus
}
#endif

#endif /* APU_GPX_H */
