/*
 * Time a dsp2c translation: frames from one GP dump (as lockstep.c loads
 * it), random input in mixbins 0-10, no JIT. Build like lockstep.c but
 * without the JIT archive, at the game's optimisation (-O2).
 * usage: bench <gp-NNNN.win> <frames>
 */
#include "apu_gpx.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include GPX_PROGRAM_C

static uint8_t *scr;
static size_t scr_len;

static void scr_rw(void *o, uint8_t *p, uint32_t a, size_t n, bool dir)
{
    (void)o;
    if (a + n > scr_len) {
        if (!dir)
            memset(p, 0, n);
        return;
    }
    if (dir)
        memcpy(scr + a, p, n);
    else
        memcpy(p, scr + a, n);
}

int main(int argc, char **argv)
{
    if (argc < 3)
        return 2;
    static uint32_t win[16384];
    FILE *f = fopen(argv[1], "rb");
    if (!f || fread(win, 4, 16384, f) != 16384)
        return 1;
    fclose(f);
    char sp[1024];
    snprintf(sp, sizeof sp, "%.*s.scr", (int)(strlen(argv[1]) - 4), argv[1]);
    f = fopen(sp, "rb");
    if (!f)
        return 1;
    fseek(f, 0, SEEK_END);
    scr_len = (size_t)ftell(f);
    fseek(f, 0, SEEK_SET);
    scr = malloc(scr_len);
    if (fread(scr, 1, scr_len, f) != scr_len)
        return 1;
    fclose(f);
    GpxCore *c = calloc(1, sizeof *c);
    c->rw = scr_rw;
    apu_gpx_reset(c);
    for (unsigned i = 0; i < 0x1000; i++)
        c->X[i] = win[i] & 0xFFFFFF;
    for (unsigned i = 0; i < 0x800; i++)
        c->Y[i] = win[0x1800 + i] & 0xFFFFFF;
    for (unsigned i = 0; i < 0x1000; i++)
        c->P[i] = win[0x2800 + i] & 0xFFFFFF;
    c->pc = 0x2B;
    c->code_ok = true;
    long frames = strtol(argv[2], NULL, 0);
    srand(1);
    struct timespec t0, t1;
    clock_gettime(CLOCK_MONOTONIC, &t0);
    for (long fr = 0; fr < frames; fr++) {
        for (unsigned i = 0; i < 0x400; i++)
            c->X[0xC00 + i] = i < 11 * 32
                ? (uint32_t)((int32_t)((rand() ^ (rand() << 12)) << 8) >> 10)
                      & 0xFFFFFF
                : 0;
        c->interrupts |= 2;
        c->budget = 1000000;
        if (k_program.run(c))
            return 1;
    }
    clock_gettime(CLOCK_MONOTONIC, &t1);
    double us = ((t1.tv_sec - t0.tv_sec) * 1e9 + (t1.tv_nsec - t0.tv_nsec)) /
                1e3 / frames;
    printf("%.2f us a frame (%ld frames)\n", us, frames);
    uint64_t h = apu_gpx_hash(c->X, 0x1000);
    printf("X hash %016llx\n", (unsigned long long)h);
    return 0;
}
