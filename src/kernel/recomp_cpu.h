/*
 * recomp_cpu.h - a guest thread's registers in one thread-local block.
 *
 * The recompiled code keeps the guest's registers in thread-locals. On
 * Mach-O every thread-local variable is reached through its own descriptor
 * call (_tlv_get_addr), and a function reuses an address only per variable:
 * a function touching eight registers paid eight calls, about 5% of the
 * guest thread in Enter the Matrix's gameplay. In one block a function pays
 * one. g_eax and the rest are macros for its fields, so every use (&g_eax,
 * g_fp_stack[i], an assignment) reads as before.
 *
 * The SIMD registers are two small blocks of their own, beside their types
 * in xbox_memory_layout.h; g_fs_base stays a variable of its own.
 *
 * Keep RecompCpu identical to templates/runtime/recomp_types.h's copy, which
 * the generated code sees: tools/recomp/test_lifter_cpu_block.py compares
 * them. The block is defined in xbox_memory_layout.c.
 *
 * Do not declare these registers yourself (extern ... g_eax): the names are
 * macros. Include this header.
 */
#ifndef RECOMP_CPU_H
#define RECOMP_CPU_H

#include <stdint.h>

#ifndef RECOMP_TLS
#  if defined(_MSC_VER)
#    define RECOMP_TLS __declspec(thread)
#  elif defined(__GNUC__) || defined(__clang__)
#    define RECOMP_TLS __thread
#  else
#    define RECOMP_TLS _Thread_local
#  endif
#endif

/* RECOMP_CPU_BLOCK_BEGIN */
#ifndef RECOMP_CPU_DEFINED
#define RECOMP_CPU_DEFINED
typedef struct RecompCpu {
    uint32_t r_eax, r_ecx, r_edx, r_ebx, r_esi, r_edi, r_esp, r_ebp;
    uint32_t r_seh_ebp, r_eflags, r_mxcsr;
    int r_df, r_fp_top, r_fp_cmp;
    uint16_t r_fp_control_word, r_fp_cc;
    double r_fp_stack[8];
} RecompCpu;
extern RECOMP_TLS RecompCpu g_cpu;
#define g_eax (g_cpu.r_eax)
#define g_ecx (g_cpu.r_ecx)
#define g_edx (g_cpu.r_edx)
#define g_ebx (g_cpu.r_ebx)
#define g_esi (g_cpu.r_esi)
#define g_edi (g_cpu.r_edi)
#define g_esp (g_cpu.r_esp)
#define g_ebp (g_cpu.r_ebp)
#define g_seh_ebp (g_cpu.r_seh_ebp)
#define g_eflags (g_cpu.r_eflags)
#define g_mxcsr (g_cpu.r_mxcsr)
#define g_df (g_cpu.r_df)
#define g_fp_top (g_cpu.r_fp_top)
#define g_fp_cmp (g_cpu.r_fp_cmp)
#define g_fp_control_word (g_cpu.r_fp_control_word)
#define g_fp_cc (g_cpu.r_fp_cc)
#define g_fp_stack (g_cpu.r_fp_stack)
#endif
/* RECOMP_CPU_BLOCK_END */

#endif /* RECOMP_CPU_H */
