# dsp2c: MCPX GP programs to C

`dsp2c.py` translates a title's GP program (DirectSound's DSP56300 kernel and the effects image the title downloads) into one C function for `src/apu/apu_gpx.h`. With it, `RECOMP_DEBUG=apu_gp=native` runs the GP without the dsp56300 JIT. The output matches the JIT instruction for instruction.

The translation is of DirectSound's code. It belongs in the game's tree, git-ignored like the rest of the recompiled game, not in this toolkit.

## Making one

1. Dump the GP after its effects code is downloaded, from 5 s on: `RECOMP_DEBUG=apu_dsp_dump=<dir>`. This writes `gp-NNNN.win` (X at +0, Y at +0x6000, P at +0xA000) and `gp-NNNN.scr` (scratch).
2. Take P, from the window's bytes at +0xA000 up to the end of the code.
3. Run:

   ```
   python3 -I tools/dsp2c/dsp2c.py <p.u32> <kernel_words> <code_at> <code_words> <name> <out.c> --disasm <dsp56300-disasm>
   ```

   For DirectSound 5344 the kernel is P:0-0x170 and the code is downloaded to P:0x171. The code's length is the command block's word +1 (Enter the Matrix: 0x5AE).

   Enter the Matrix's `scripts/gpgen.sh` wraps steps 2 and 3.
4. Build the output into the game. A constructor registers it with `apu_gpx_register`.

At run time:
- **At GPRST's bootstrap**, `apu_gp_native.c` matches the boot ROM's copy (scratch's first 0x800 words) against the registered kernels' hashes.
- **After the download**, it checks the downloaded code's hash.
- **If either doesn't match**, the native GP falls back to the hand-written effects.

An instruction form the translator doesn't know stops it with the address and the text. Add the case rather than approximate it.

## Semantics

These follow the DSP56300 manual and xemu's interpreter, except where the JIT differs. The JIT is the reference, because its output is what the native GP has to equal. Each difference below was found with the lockstep test:

- **S bit:** set when a value moved out of an accumulator has bits 46 and 45 different. xemu doesn't model it.
- **SR's upper byte:** survives ALU instructions. xemu clears it.
- **End of a DO loop:** restores only LF. xemu also clears S, MR and EMR.
- **`macr`'s V and L:** taken from the addends' signs against the **rounded** result's sign. A negative sum that rounds to 0 counts as an overflow.
- **`bset` and `bclr` on SR:** leave C alone.
- **`mpy` and `mpyi` under SR.SM:** saturate the product too. -1 × -1 gives 0x7FFFFF:FFFFFF and sets V and L.

The translator skips computing E, U, N and Z for an instruction when a later instruction in the same basic block rewrites all four before anything reads SR. Every block boundary counts them as live. For Enter the Matrix this took the GP from 31 to 27 µs a frame. Lockstep a translation built this way with `LOCKSTEP_SR_MASK=0x3C`: SR is meant to differ there. A dead flag that is actually read would still show up as a PC or register mismatch.

`bench.c` times a translation on its own. It loads a dump as `lockstep.c` does and is built the same way, with `-O2` and without the JIT archive. Keeping the DSP registers in locals rather than in `GpxCore` was tried and made it slower (37 µs). The one large run function spills them.

## The lockstep test

`lockstep.c` runs the translation and the JIT side by side from one dump, with random input in mixbins 0-10:
- After every instruction it compares the register files.
- After every frame it compares X memory and scratch.
- It prints the first difference, with the state before and after it.

Build (macOS, against the build's JIT archive):

```
D=<build>/_deps/dsp56300/dsp56300-*/
clang -O1 -g -Wno-unused-label -I src/apu -I $D/include \
    -DGPX_PROGRAM_C='"<out.c>"' tools/dsp2c/lockstep.c src/apu/apu_gpx.c \
    $D/lib/libdsp56300_emu_ffi.a -framework CoreFoundation -framework Security \
    -o lockstep
./lockstep <gp-NNNN.win> <frames> [seed]
```

- **`LOCKSTEP_SR_MASK=0x42`** leaves SR bits out of the register comparison (here V and L), so the test can get past a flag difference while you look at it.
- **`lockstep probe <op> <a2> <a1> <a0> <x0> <x1> <y0> <y1> <sr>`** runs one instruction on the JIT and prints the accumulator and SR. Use it to pin down a flag rule.

Enter the Matrix (2026-11-12): three gameplay dumps, three seeds each, 3000 frames each (about 250 million instructions in all) ran in lockstep.
