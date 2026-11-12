/*
 * The APU's GP in C: DirectSound's GP kernel and the stock XDK effects that
 * a title's effects image chains, run on the same 24-bit X memory the
 * DSP56300 program keeps, so the output matches the program's word for word.
 *
 * What it replaces is the GP program on the dsp56300 JIT (apu_dsp_run.c),
 * not the voice processor: the VP's mixbins come in, the GP's bins 0-5 go
 * to the EP's scratch rings, as the program's DMA puts them.
 *
 * Two ways to run it. A program the game translated with tools/dsp2c
 * (apu_gpx.h) runs when its kernel's hash matches the boot ROM's copy; its
 * effects run only once the downloaded code's hash matches too, and until
 * then (or for other code) the kernel runs with the effects skipped. With
 * no translation, the hand-written XTalk and mixers here run for the one
 * image in their table, and bins pass through for any other.
 */
#ifndef APU_GP_NATIVE_H
#define APU_GP_NATIVE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define APU_GPN_BINS    32
#define APU_GPN_SAMPLES 32

/* The GP's scratch space by byte address; dir true writes scratch. */
typedef void (*ApuGpnScratchRw)(void *opaque, uint8_t *ptr, uint32_t addr,
                                size_t len, bool dir);

typedef struct ApuGpNative ApuGpNative;

ApuGpNative *apu_gpn_create(ApuGpnScratchRw rw, void *opaque);
void apu_gpn_destroy(ApuGpNative *g);

/* GPRST's bootstrap edge: the program starts over, nothing is loaded. */
void apu_gpn_reset(ApuGpNative *g);

/* One GP frame. mixbuf holds the VP's 32 mixbins as 24-bit words (bin b at
 * b*32) on entry and the GP's on return. commit false is the shadow mode:
 * the frame reads DirectSound's command but neither acknowledges it nor
 * writes scratch, so the real GP still sees and answers it. */
void apu_gpn_frame(ApuGpNative *g, uint32_t mixbuf[APU_GPN_BINS *
                                                   APU_GPN_SAMPLES],
                   bool commit);

/* An effects image has been downloaded and is one the table knows. */
bool apu_gpn_known(const ApuGpNative *g);
/* An image has been downloaded (known or not). */
bool apu_gpn_loaded(const ApuGpNative *g);

/* Run the translated core on the GP's MCPX window (X at +0, Y at +0x6000,
 * P at +0xA000), as the hardware does: the title's own reads and writes of
 * GP memory then reach it. For apu_gp=native; the shadow keeps its own
 * memory and takes the title's writes through apu_gpn_poke_x. */
void apu_gpn_set_window(ApuGpNative *g, uint8_t *window);
/* A word the title wrote to the GP's X memory (shadow mode). */
void apu_gpn_poke_x(ApuGpNative *g, uint32_t addr, uint32_t value);

/* The X memory word at a DSP address (the mixbuf at 0xC00 or 0x1400). */
uint32_t apu_gpn_x(const ApuGpNative *g, uint32_t addr);

#ifdef __cplusplus
}
#endif

#endif /* APU_GP_NATIVE_H */
