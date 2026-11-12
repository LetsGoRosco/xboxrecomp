/*
 * Lockstep test for a dsp2c translation against the dsp56300 JIT.
 *
 * Both cores start from one GP dump (RECOMP_DEBUG=apu_dsp_dump: a .win for
 * X/Y/P and the .scr beside it for scratch), with the same registers at the
 * kernel's resume point, and get the same random mixbins every frame. The
 * translation is compiled with GPX_TRACE defined to call trace() after each
 * instruction; trace() steps the JIT one instruction and compares the
 * register files. The first difference is printed with the instruction.
 *
 * build: see tools/dsp2c/README.md
 * usage: lockstep <gp-NNNN.win> <frames> [seed]
 */
#include "apu_gpx.h"
#include "dsp56300.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void trace(GpxCore *c, uint32_t addr);
#define GPX_TRACE(c, addr) trace((c), (addr))
#include GPX_PROGRAM_C

static GpxCore *N, *J;
static Dsp56300Jit *jit;
static uint8_t *scr[2];
static size_t scr_len;
static uint64_t steps, frame;

static void scr_rw(void *opaque, uint8_t *p, uint32_t a, size_t n, bool dir)
{
    uint8_t *s = opaque;
    if (a + n > scr_len) {
        if (!dir)
            memset(p, 0, n);
        return;
    }
    if (dir)
        memcpy(s + a, p, n);
    else
        memcpy(p, s + a, n);
}

static uint32_t periph_read(void *o, uint32_t a) { return apu_gpx_periph_read(o, a); }
static void periph_write(void *o, uint32_t a, uint32_t v) { apu_gpx_periph_write(o, a, v); }

static const char *names[64];
static uint32_t prev_rn[64];
static Dsp56300State prev_st;

static void regs_native(const GpxCore *c, uint32_t *r)
{
    memset(r, 0, 64 * 4);
    r[DSP56300_REG_X0] = c->x0; r[DSP56300_REG_X1] = c->x1;
    r[DSP56300_REG_Y0] = c->y0; r[DSP56300_REG_Y1] = c->y1;
    r[DSP56300_REG_A2] = gpx_a2(c->a); r[DSP56300_REG_A1] = gpx_a1(c->a);
    r[DSP56300_REG_A0] = gpx_a0(c->a);
    r[DSP56300_REG_B2] = gpx_a2(c->b); r[DSP56300_REG_B1] = gpx_a1(c->b);
    r[DSP56300_REG_B0] = gpx_a0(c->b);
    for (int i = 0; i < 8; i++) {
        r[DSP56300_REG_R0 + i] = c->r[i];
        r[DSP56300_REG_N0 + i] = c->n[i];
    }
    r[DSP56300_REG_SR] = c->sr;
    r[DSP56300_REG_LA] = c->la;
    r[DSP56300_REG_LC] = c->lc;
    r[DSP56300_REG_SP] = (uint32_t)c->sp;
}

static const int cmp_regs[] = {
    DSP56300_REG_X0, DSP56300_REG_X1, DSP56300_REG_Y0, DSP56300_REG_Y1,
    DSP56300_REG_A2, DSP56300_REG_A1, DSP56300_REG_A0,
    DSP56300_REG_B2, DSP56300_REG_B1, DSP56300_REG_B0,
    DSP56300_REG_R0, DSP56300_REG_R1, DSP56300_REG_R2, DSP56300_REG_R3,
    DSP56300_REG_R4, DSP56300_REG_R5, DSP56300_REG_R6, DSP56300_REG_R7,
    DSP56300_REG_N0, DSP56300_REG_N1, DSP56300_REG_N2, DSP56300_REG_N3,
    DSP56300_REG_N4, DSP56300_REG_N5, DSP56300_REG_N6, DSP56300_REG_N7,
    DSP56300_REG_SR, DSP56300_REG_LA, DSP56300_REG_LC, DSP56300_REG_SP,
};

static void fail(const char *why, uint32_t addr, const uint32_t *rn,
                 const Dsp56300State *st)
{
    printf("MISMATCH (%s) frame %llu step %llu after P:%04X\n", why,
           (unsigned long long)frame, (unsigned long long)steps, addr);
    printf("       before (both)   after: native  jit\n");
    for (size_t i = 0; i < sizeof cmp_regs / sizeof cmp_regs[0]; i++) {
        int k = cmp_regs[i];
        uint32_t m = k == DSP56300_REG_SP ? 0xF : 0xFFFFFF;
        uint32_t a = rn[k] & m, b = st->registers[k] & m;
        printf("  %-3s %06X          %06X  %06X%s\n", names[k],
               prev_rn[k] & m, a, b, a != b ? "  <--" : "");
    }
    exit(1);
}

void trace(GpxCore *c, uint32_t addr)
{
    Dsp56300State st;
    dsp56300_get_state(jit, &st);
    uint32_t rn[64];
    if (st.pc != addr) {
        regs_native(c, rn);
        printf("pc: jit at P:%04X, native ran P:%04X\n", st.pc, addr);
        fail("pc", addr, rn, &st);
    }
    prev_st = st;
    dsp56300_step(jit);
    steps++;
    dsp56300_get_state(jit, &st);
    regs_native(c, rn);
    for (size_t i = 0; i < sizeof cmp_regs / sizeof cmp_regs[0]; i++) {
        int k = cmp_regs[i];
        uint32_t m = k == DSP56300_REG_SP ? 0xF : 0xFFFFFF;
        if (k == DSP56300_REG_SR && getenv("LOCKSTEP_SR_MASK"))
            m &= ~(uint32_t)strtoul(getenv("LOCKSTEP_SR_MASK"), NULL, 0);
        if ((rn[k] & m) != (st.registers[k] & m))
            fail(names[k], addr, rn, &st);
    }
    memcpy(prev_rn, rn, sizeof prev_rn);
}

static uint32_t rnd24(void)
{
    return ((uint32_t)rand() ^ ((uint32_t)rand() << 12)) & 0xFFFFFF;
}

int main(int argc, char **argv)
{
    if (argc < 3) {
        fprintf(stderr, "usage: lockstep <gp-NNNN.win> <frames> [seed]\n");
        return 2;
    }
    names[DSP56300_REG_X0] = "x0"; names[DSP56300_REG_X1] = "x1";
    names[DSP56300_REG_Y0] = "y0"; names[DSP56300_REG_Y1] = "y1";
    names[DSP56300_REG_A2] = "a2"; names[DSP56300_REG_A1] = "a1";
    names[DSP56300_REG_A0] = "a0"; names[DSP56300_REG_B2] = "b2";
    names[DSP56300_REG_B1] = "b1"; names[DSP56300_REG_B0] = "b0";
    static const char *rr[] = { "r0","r1","r2","r3","r4","r5","r6","r7" };
    static const char *nn[] = { "n0","n1","n2","n3","n4","n5","n6","n7" };
    for (int i = 0; i < 8; i++) {
        names[DSP56300_REG_R0 + i] = rr[i];
        names[DSP56300_REG_N0 + i] = nn[i];
    }
    names[DSP56300_REG_SR] = "sr"; names[DSP56300_REG_LA] = "la";
    names[DSP56300_REG_LC] = "lc"; names[DSP56300_REG_SP] = "sp";

    /* lockstep probe <op> <a2> <a1> <a0> <x0> <x1> <y0> <y1> <sr>: one
     * instruction on the JIT, for pinning down its flags */
    if (!strcmp(argv[1], "probe") && argc >= 11) {
        J = calloc(1, sizeof *J);
        apu_gpx_reset(J);
        uint32_t v[9];
        for (int i = 0; i < 9; i++)
            v[i] = (uint32_t)strtoul(argv[i + 2], NULL, 16);
        J->P[0] = v[0];
        for (int i = 1; i < 16; i++)
            J->P[i] = 0x00000C;
        Dsp56300MemoryRegion xr[2] = {
            { .start = 0, .end = 0x1000, .kind = DSP56300_REGION_BUFFER,
              .data.buffer = { J->X, 0 } },
            { .start = 0xFFFF80, .end = 0x1000000,
              .kind = DSP56300_REGION_CALLBACK,
              .data.callback = { J, periph_read, periph_write } } };
        Dsp56300MemoryRegion yr = { .start = 0, .end = 0x800,
            .kind = DSP56300_REGION_BUFFER, .data.buffer = { J->Y, 0 } };
        Dsp56300MemoryRegion pr = { .start = 0, .end = 0x1000,
            .kind = DSP56300_REGION_BUFFER, .data.buffer = { J->P, 0 } };
        Dsp56300CreateInfo ci = { .memory_map = { xr, 2, &yr, 1, &pr, 1 } };
        jit = dsp56300_create(&ci);
        dsp56300_reset(jit);
        Dsp56300State st;
        dsp56300_get_state(jit, &st);
        st.registers[DSP56300_REG_A2] = v[1];
        st.registers[DSP56300_REG_A1] = v[2];
        st.registers[DSP56300_REG_A0] = v[3];
        st.registers[DSP56300_REG_X0] = v[4];
        st.registers[DSP56300_REG_X1] = v[5];
        st.registers[DSP56300_REG_Y0] = v[6];
        st.registers[DSP56300_REG_Y1] = v[7];
        st.registers[DSP56300_REG_SR] = v[8];
        for (int i = 0; i < 8; i++)
            st.registers[DSP56300_REG_M0 + i] = 0xFFFFFF;
        st.pc = 0;
        dsp56300_set_state(jit, &st);
        dsp56300_step(jit);
        dsp56300_get_state(jit, &st);
        printf("a=%02X:%06X:%06X sr=%06X\n", st.registers[DSP56300_REG_A2],
               st.registers[DSP56300_REG_A1], st.registers[DSP56300_REG_A0],
               st.registers[DSP56300_REG_SR]);
        return 0;
    }
    long frames = strtol(argv[2], NULL, 0);
    srand(argc > 3 ? (unsigned)atoi(argv[3]) : 1);

    FILE *f = fopen(argv[1], "rb");
    static uint32_t win[16384];
    if (!f || fread(win, 4, 16384, f) != 16384) {
        perror(argv[1]);
        return 1;
    }
    fclose(f);
    char sp[1024];
    snprintf(sp, sizeof sp, "%.*s.scr", (int)(strlen(argv[1]) - 4), argv[1]);
    f = fopen(sp, "rb");
    if (!f) {
        perror(sp);
        return 1;
    }
    fseek(f, 0, SEEK_END);
    scr_len = (size_t)ftell(f);
    fseek(f, 0, SEEK_SET);
    for (int k = 0; k < 2; k++) {
        scr[k] = malloc(scr_len);
        fseek(f, 0, SEEK_SET);
        if (fread(scr[k], 1, scr_len, f) != scr_len)
            return 1;
    }
    fclose(f);

    N = calloc(1, sizeof *N);
    J = calloc(1, sizeof *J);
    N->rw = scr_rw; N->opaque = scr[0];
    J->rw = scr_rw; J->opaque = scr[1];
    apu_gpx_reset(N);
    apu_gpx_reset(J);
    for (GpxCore *c = N; c; c = c == N ? J : NULL) {
        for (unsigned i = 0; i < 0x1000; i++)
            c->X[i] = win[i] & 0xFFFFFF;
        for (unsigned i = 0; i < 0x800; i++)
            c->Y[i] = win[0x1800 + i] & 0xFFFFFF;
        for (unsigned i = 0; i < 0x1000; i++)
            c->P[i] = win[0x2800 + i] & 0xFFFFFF;
        c->pc = 0x2B;
        /* the dump is from after the effects code was downloaded */
        c->code_ok = true;
    }

    Dsp56300MemoryRegion xr[3] = {
        { .start = 0, .end = 0x1000, .kind = DSP56300_REGION_BUFFER,
          .data.buffer = { J->X, 0 } },
        { .start = 0x1400, .end = 0x1800, .kind = DSP56300_REGION_BUFFER,
          .data.buffer = { J->X, 0xC00 } },
        { .start = 0xFFFF80, .end = 0x1000000, .kind = DSP56300_REGION_CALLBACK,
          .data.callback = { J, periph_read, periph_write } },
    };
    Dsp56300MemoryRegion yr[1] = {
        { .start = 0, .end = 0x800, .kind = DSP56300_REGION_BUFFER,
          .data.buffer = { J->Y, 0 } } };
    Dsp56300MemoryRegion pr[1] = {
        { .start = 0, .end = 0x1000, .kind = DSP56300_REGION_BUFFER,
          .data.buffer = { J->P, 0 } } };
    Dsp56300CreateInfo ci = { .memory_map = { xr, 3, yr, 1, pr, 1 } };
    jit = dsp56300_create(&ci);
    dsp56300_reset(jit);
    Dsp56300State st;
    dsp56300_get_state(jit, &st);
    memset(st.registers, 0, sizeof st.registers);
    memset(st.stack, 0, sizeof st.stack);
    st.registers[DSP56300_REG_SR] = N->sr;
    for (int i = 0; i < 8; i++)
        st.registers[DSP56300_REG_M0 + i] = 0xFFFFFF;
    st.pc = 0x2B;
    dsp56300_set_state(jit, &st);

    for (frame = 0; frame < (uint64_t)frames; frame++) {
        for (unsigned i = 0; i < 0x400; i++) {
            /* bins 0-10 carry signal at about -12 dB, the rest silence */
            uint32_t v = i < 11 * 32 ? (uint32_t)((int32_t)(rnd24() << 8) >> 10)
                                       & 0xFFFFFF : 0;
            N->X[0xC00 + i] = J->X[0xC00 + i] = v;
        }
        N->interrupts |= 2;
        J->interrupts |= 2;
        N->budget = 1000000;
        int rc = k_program.run(N);
        if (rc) {
            printf("native run returned %d at P:%04X\n", rc, N->pc);
            return 1;
        }
        if (memcmp(N->X, J->X, GPX_X_WORDS * 4)) {
            for (unsigned i = 0; i < 0x1000; i++)
                if (N->X[i] != J->X[i]) {
                    printf("X MISMATCH frame %llu: X:%04X native %06X jit"
                           " %06X\n", (unsigned long long)frame, i, N->X[i],
                           J->X[i]);
                    return 1;
                }
        }
        if (memcmp(scr[0], scr[1], scr_len)) {
            for (size_t i = 0; i < scr_len; i += 4)
                if (memcmp(scr[0] + i, scr[1] + i, 4)) {
                    printf("SCRATCH MISMATCH frame %llu at %06zx\n",
                           (unsigned long long)frame, i);
                    return 1;
                }
        }
    }
    printf("OK: %ld frames, %llu instructions in lockstep\n", frames,
           (unsigned long long)steps);
    return 0;
}
