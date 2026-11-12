/*
 * The runtime for translated GP programs (apu_gpx.h): the registry, reset,
 * and the peripheral page with the DMA engine, as apu_dsp_emu.c models
 * them for the JIT (which follows xemu's dsp_dma.c).
 */
#include "apu_gpx.h"

#include <stdio.h>
#include <string.h>

#define MAX_PROGRAMS 8
static const GpxProgram *s_programs[MAX_PROGRAMS];
static int s_nprograms;

void apu_gpx_register(const GpxProgram *p)
{
    if (s_nprograms < MAX_PROGRAMS)
        s_programs[s_nprograms++] = p;
}

uint64_t apu_gpx_hash(const uint32_t *w, uint32_t n)
{
    uint64_t h = 0xcbf29ce484222325ull;
    for (uint32_t i = 0; i < n; i++)
        for (int b = 0; b < 3; b++) {
            h ^= (w[i] >> (8 * b)) & 0xFF;
            h *= 0x100000001b3ull;
        }
    return h;
}

const GpxProgram *apu_gpx_find_kernel(const uint32_t *p, uint32_t words)
{
    for (int i = 0; i < s_nprograms; i++) {
        const GpxProgram *g = s_programs[i];
        if (g->kernel_words <= words &&
            apu_gpx_hash(p, g->kernel_words) == g->kernel_hash)
            return g;
    }
    return NULL;
}

void apu_gpx_reset(GpxCore *c)
{
    c->a = c->b = 0;
    c->x0 = c->x1 = c->y0 = c->y1 = 0;
    memset(c->r, 0, sizeof c->r);
    memset(c->n, 0, sizeof c->n);
    for (int i = 0; i < 8; i++)
        c->m[i] = 0xFFFFFF;
    c->sr = GPX_SR_RESET;
    c->la = c->lc = 0;
    memset(c->ssh, 0, sizeof c->ssh);
    memset(c->ssl, 0, sizeof c->ssl);
    c->sp = 0;
    c->pc = 0;
    c->interrupts = 0;
    memset(&c->dma, 0, sizeof c->dma);
    c->halt = c->p_written = false;
    c->code_ok = false;
    if (!c->X)
        apu_gpx_set_memory(c, NULL);
}

void apu_gpx_set_memory(GpxCore *c, uint8_t *window)
{
    if (window) {
        c->X = (uint32_t *)window;
        c->Y = (uint32_t *)(window + 0x6000);
        c->P = (uint32_t *)(window + 0xA000);
    } else {
        c->X = c->Xown;
        c->Y = c->Yown;
        c->P = c->Pown;
    }
}

/* ---- the DMA engine ---- */

#define REG_HALT     0x44u
#define REG_INT      0x45u
#define REG_DMA      0x54u
#define INT_DMA_EOL  (1u << 7)
#define ACT_START    1u
#define ACT_STOP     2u
#define ACT_FREEZE   3u
#define ACT_UNFREEZE 4u
#define FROZEN       (1u << 3)
#define RUNNING      (1u << 4)
#define STOPPED      (1u << 5)
#define EOL          (1u << 14)
#define PTR          0x3fffu

static bool p_dirty;

static uint32_t *dma_mem(GpxCore *c, uint32_t a, uint32_t n)
{
    if (a < 0x1800)
        return a + n <= GPX_X_WORDS ? &c->X[a]
               : (a >= 0x1400 && a + n <= 0x1800) ? &c->X[a - 0x800] : NULL;
    if (a >= 0x1800 && a + n <= 0x2000)
        return &c->Y[a - 0x1800];
    if (a >= 0x2800 && a + n <= 0x3800) {
        p_dirty = true;
        return &c->P[a - 0x2800];
    }
    return NULL;
}

static void circular(GpxCore *c, uint32_t base, uint32_t *off, uint32_t size,
                     size_t len, uint8_t *buf, bool dir)
{
    if (*off >= size)
        *off = 0;
    size_t done = 0;
    while (len > 0) {
        size_t n = size - *off;
        if (n > len)
            n = len;
        c->rw(c->opaque, buf + done, base + *off, n, dir);
        *off += (uint32_t)n;
        if (*off >= size)
            *off = 0;
        len -= n;
        done += n;
    }
}

static void dma_run(GpxCore *c)
{
    GpxDma *s = &c->dma;
    if (!(s->control & RUNNING) || (s->control & FROZEN))
        return;
    uint32_t walked = 0;
    while (!(s->next_block & EOL)) {
        uint32_t addr = s->next_block & PTR;
        uint32_t *d = dma_mem(c, addr, 7);
        p_dirty = false;            /* a descriptor in P is not a P write */
        if (!d || walked++ == 0x800) {
            s->eol = true;
            s->next_block |= EOL;
            return;
        }
        uint32_t next = d[0] & 0xFFFFFF, control = d[1] & 0xFFFFFF;
        uint32_t count = d[2] & 0xFFFFFF, dsp = d[3] & 0xFFFFFF;
        uint32_t soff = d[4] & 0xFFFFFF, sbase = d[5] & 0xFFFFFF;
        uint32_t ssize = (d[6] & 0xFFFFFF) + 1;
        s->next_block = next;
        if (next & EOL)
            s->eol = true;
        bool inter = control & 1, dir = control & 2;
        bool wb = (control >> 4) & 1;
        uint32_t buf_id = (control >> 5) & 0xF, format = (control >> 10) & 7;
        unsigned item = format == 1 ? 2 : (format == 2 || format == 6) ? 4 : 0;
        uint32_t *m = count <= 0x1800 ? dma_mem(c, dsp, count) : NULL;
        bool scratch = buf_id >= 0xE;
        if (!item || ((control >> 2) & 3) || ((control >> 13) & 1) ||
            !scratch || inter || !m) {
            /* Only what the GP programs seen use: scratch, plain or
             * circular, 16- or 32-bit items, no interleave. */
            if (c->refused_logs++ < 4)
                fprintf(stderr, "[APU GPX] dma descriptor %04x refused"
                        " (control %06x)\n", addr, control);
            s->eol = true;
            s->next_block |= EOL;
            return;
        }
        size_t len = (size_t)count * item;
        uint8_t *buf = c->xfer;
        if (dir) {
            for (uint32_t i = 0; i < count; i++) {
                uint32_t w = m[i] & 0xFFFFFF;
                if (item == 2) {
                    uint16_t h = (uint16_t)(w >> 8);
                    memcpy(buf + i * 2, &h, 2);
                } else {
                    memcpy(buf + i * 4, &w, 4);
                }
            }
            if (buf_id == 0xE)
                circular(c, sbase, &soff, ssize, len, buf, true);
            else
                c->rw(c->opaque, buf, sbase + soff, len, true);
        } else {
            if (buf_id == 0xE)
                circular(c, sbase, &soff, ssize, len, buf, false);
            else
                c->rw(c->opaque, buf, sbase + soff, len, false);
            for (uint32_t i = 0; i < count; i++) {
                uint32_t v;
                if (item == 2) {
                    uint16_t h;
                    memcpy(&h, buf + i * 2, 2);
                    v = (uint32_t)h << 8;
                } else {
                    memcpy(&v, buf + i * 4, 4);
                    v &= 0xFFFFFF;
                }
                m[i] = v;
            }
        }
        if (wb)
            d[4] = soff;
        if (p_dirty && !dir) {
            c->p_written = true;
            if (c->code_words) {
                bool ok = apu_gpx_hash_p(c) == c->code_hash;
                c->code_ok = ok;
            }
        }
        p_dirty = false;
    }
}

uint64_t apu_gpx_hash_p(const GpxCore *c)
{
    uint64_t h = 0xcbf29ce484222325ull;
    for (uint32_t i = c->code_at; i < c->code_at + c->code_words &&
                                  i < GPX_P_WORDS; i++)
        for (int b = 0; b < 3; b++) {
            h ^= ((c->P[i] & 0xFFFFFF) >> (8 * b)) & 0xFF;
            h *= 0x100000001b3ull;
        }
    return h;
}

uint32_t apu_gpx_periph_read(GpxCore *c, uint32_t addr)
{
    uint32_t off = addr - 0xFFFF80u;
    if (off - REG_DMA < 4) {
        GpxDma *s = &c->dma;
        switch (off - REG_DMA) {
        case 3: return s->configuration;
        case 2:
            if (s->control & RUNNING && ++s->read_count > 2) {
                s->control = (s->control & ~RUNNING) | STOPPED;
                s->read_count = 0;
            }
            return s->control;
        case 1: return s->start_block;
        default: return s->next_block;
        }
    }
    if (off == REG_INT)
        return c->interrupts | (c->dma.eol ? INT_DMA_EOL : 0);
    return 0;
}

void apu_gpx_periph_write(GpxCore *c, uint32_t addr, uint32_t v)
{
    uint32_t off = addr - 0xFFFF80u;
    if (off - REG_DMA < 4) {
        GpxDma *s = &c->dma;
        switch (off - REG_DMA) {
        case 3: s->configuration = v; break;
        case 2:
            switch (v & 7) {
            case ACT_START:
                s->control = (s->control | RUNNING) & ~STOPPED;
                s->read_count = 0;
                break;
            case ACT_STOP:
                s->control = (s->control | STOPPED) & ~RUNNING;
                break;
            case ACT_FREEZE: s->control |= FROZEN; break;
            case ACT_UNFREEZE: s->control &= ~FROZEN; break;
            default: break;
            }
            dma_run(c);
            break;
        case 1: s->start_block = v; break;
        default: s->next_block = v; break;
        }
    } else if (off == REG_INT) {
        c->interrupts &= ~v;
        if (v & INT_DMA_EOL)
            c->dma.eol = false;
    } else if (off == REG_HALT && (v & 1)) {
        c->halt = true;
    }
}
