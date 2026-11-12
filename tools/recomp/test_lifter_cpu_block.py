"""The guest register block: the runtime's and the generated code's copies.

src/kernel/recomp_cpu.h (the runtime) and templates/runtime/recomp_types.h
(copied into every game's generated code) each define RecompCpu and the
g_eax ... macros, and xbox_memory_layout.h and recomp_types.h each define
the SIMD register blocks. The two sides link against one definition, so a
difference in layout is memory corruption with no error: they must be the
same text.
"""
import pathlib
import unittest

_ROOT = pathlib.Path(__file__).resolve().parents[2]


def _between(text, begin, end):
    i = text.index(begin)
    return text[i:text.index(end, i) + len(end)]


class CpuBlockTest(unittest.TestCase):
    def setUp(self):
        self.types = (_ROOT / "templates" / "runtime" /
                      "recomp_types.h").read_text(encoding="utf-8")
        self.cpu = (_ROOT / "src" / "kernel" /
                    "recomp_cpu.h").read_text(encoding="utf-8")
        self.layout = (_ROOT / "src" / "kernel" /
                       "xbox_memory_layout.h").read_text(encoding="utf-8")

    def test_general_block_identical(self):
        b, e = "/* RECOMP_CPU_BLOCK_BEGIN */", "/* RECOMP_CPU_BLOCK_END */"
        self.assertEqual(_between(self.cpu, b, e), _between(self.types, b, e))

    def test_simd_block_identical(self):
        b = "#ifndef RECOMP_SIMD_REGS_DEFINED"
        self.assertEqual(_between(self.layout, b, "#endif\n"),
                         _between(self.types, b, "#endif\n"))

    def test_no_separate_register_declarations(self):
        for name in ("g_eax", "g_esp", "g_fp_stack", "g_xmm0", "g_mm0"):
            self.assertNotIn(f"RECOMP_TLS uint32_t {name};", self.types)
            self.assertNotIn(f" {name}, ", self.types.split("#define")[0][-1:])


if __name__ == "__main__":
    unittest.main()
