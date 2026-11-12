#!/usr/bin/env python3
"""Translate an MCPX GP program (DSP56300) to C for src/apu/apu_gpx.h.

usage: dsp2c.py <p.u32> <kernel_words> <code_at> <code_words> <name> <out.c>
           [--disasm PATH]

<p.u32> is the GP's P memory as little-endian u32 words, from
RECOMP_DEBUG=apu_dsp_dump (a .win file's bytes at +0xA000), after the
title's effects code was downloaded. P:0 up to <kernel_words> is the kernel
the boot ROM loads; <code_at>, <code_words> is the downloaded effects code.
The output defines one GpxProgram, registered by a constructor, whose run
function executes the program from GpxCore.pc to the frame halt.

The instruction subset is what DirectSound's GP programs use; anything else
stops the translation with the address and text, so a new program gets new
cases here rather than a silent mistranslation. Semantics are noted in
apu_gpx.h.
"""
import re
import struct
import subprocess
import sys

DISASM = 'dsp56300-disasm'

ACCS = ('a', 'b')
DATA = ('x0', 'x1', 'y0', 'y1')


class Unsupported(Exception):
    pass


def fnv(words):
    h = 0xcbf29ce484222325
    for w in words:
        for b in range(3):
            h ^= (w >> (8 * b)) & 0xFF
            h = (h * 0x100000001b3) & 0xFFFFFFFFFFFFFFFF
    return h


def disassemble(path, words, disasm):
    out = subprocess.run([disasm, '-f', 'u32le', '-c', 'never', path],
                         check=True, capture_output=True, text=True).stdout
    insns = []
    for line in out.splitlines():
        m = re.match(r'P:\$([0-9A-F]{6})\s+((?:[0-9A-F]{6} ?)+)\s+(.*)$',
                     line)
        if not m:
            continue
        addr = int(m.group(1), 16)
        if addr >= words:
            break
        n = len(m.group(2).split())
        insns.append((addr, n, m.group(3).strip()))
    return insns


# ---- operands ----

def reg(name):
    if name in ACCS or name in DATA:
        return f'c->{name}'
    m = re.fullmatch(r'([rnm])([0-7])', name)
    if m:
        return f'c->{m.group(1)}[{m.group(2)}]'
    if name in ('sr', 'la', 'lc'):
        return f'c->{name}'
    raise Unsupported(f'register {name}')


def imm(tok):
    """'#$xx' / '#>$xxxxxx' / '#$xxxxxx': (value, short)."""
    m = re.fullmatch(r'#(>?)\$([0-9a-f]+)', tok)
    if not m:
        raise Unsupported(f'immediate {tok}')
    short = m.group(1) == '' and len(m.group(2)) <= 2
    return int(m.group(2), 16), short


class Ea:
    """An effective address: the address expression and its update."""

    def __init__(self, tok):
        self.space = tok[0]
        body = tok[2:]
        if body.startswith('>'):
            body = body[1:]
        self.post = None
        m = re.fullmatch(r'\$([0-9a-f]+)', body)
        if m:
            self.addr = f'0x{int(m.group(1), 16):X}u'
            self.const = int(m.group(1), 16)
            return
        self.const = None
        m = re.fullmatch(r'\((r[0-7])\)(\+n[0-7]|\+|-)?', body)
        if m:
            r = reg(m.group(1))
            self.addr = r
            upd = m.group(2)
            if upd == '+':
                self.post = f'{r} = ({r} + 1) & 0xFFFFFF;'
            elif upd == '-':
                self.post = f'{r} = ({r} - 1) & 0xFFFFFF;'
            elif upd:
                self.post = f'{r} = ({r} + {reg(upd[1:])}) & 0xFFFFFF;'
            return
        m = re.fullmatch(r'\((r[0-7])\+(n[0-7])\)', body)
        if m:
            self.addr = f'({reg(m.group(1))} + {reg(m.group(2))})'
            return
        m = re.fullmatch(r'\((r[0-7])([+-])(\d+)\)', body)
        if m:
            self.addr = f'({reg(m.group(1))} {m.group(2)} {m.group(3)}u)'
            return
        raise Unsupported(f'address {tok}')

    def read(self):
        if self.space == 'x':
            return f'gpx_xr(c, {self.addr})'
        if self.space == 'y':
            return f'gpx_yr(c, {self.addr})'
        if self.space == 'p':
            return f'gpx_pr(c, {self.addr})'
        raise Unsupported(self.space)

    def write(self, v):
        if self.space == 'x':
            return f'gpx_xw(c, {self.addr}, {v});'
        if self.space == 'y':
            return f'gpx_yw(c, {self.addr}, {v});'
        raise Unsupported(f'write {self.space}')


def is_mem(tok):
    return re.match(r'[xyp]:', tok) is not None


def src24(tok):
    """A move source as a 24-bit word (accumulators through the limiter)."""
    if tok.startswith('#'):
        v, short = imm(tok)
        return v, short, None
    if is_mem(tok):
        e = Ea(tok)
        return e.read(), False, e
    if tok in ACCS:
        return f'gpx_acc24(c, c->{tok})', False, None
    m = re.fullmatch(r'([ab])([012])', tok)
    if m:
        return f'gpx_a{m.group(2)}(c->{m.group(1)})', False, None
    return reg(tok), False, None


def dst_write(tok, v, short_imm):
    """Write a 24-bit word to a move destination."""
    if is_mem(tok):
        e = Ea(tok)
        return [e.write(v)], e
    if short_imm:
        if tok in ACCS or tok in DATA:
            v = f'(({v}) << 16)'
    if tok in ACCS:
        return [f'c->{tok} = gpx_ld({v});'], None
    m = re.fullmatch(r'([ab])([01])', tok)
    if m:
        return [f'c->{m.group(1)} = gpx_set_a{m.group(2)}(c->{m.group(1)},'
                f' {v});'], None
    return [f'{reg(tok)} = ({v}) & 0xFFFFFF;'], None


def moves(toks):
    """Parallel moves: (reads, writes, address updates), reads first."""
    reads, writes, posts = [], [], []
    for i, t in enumerate(toks):
        s, d = t.split(',')
        v, short, e1 = src24(s)
        tmp = f'mv{i}'
        if isinstance(v, int):
            reads.append(f'uint32_t {tmp} = 0x{v:X}u;')
        else:
            reads.append(f'uint32_t {tmp} = {v};')
        w, e2 = dst_write(d, tmp, short)
        writes += w
        for e in (e1, e2):
            if e and e.post:
                posts.append(e.post)
    return reads, writes, posts


# ---- the ALU ----

def alu_src(tok):
    """An ALU source as a 56-bit value."""
    if tok in ACCS:
        return f'c->{tok}'
    if tok.startswith('#'):
        v, _ = imm(tok)
        return f'gpx_ld(0x{v:X}u)'
    if tok in DATA:
        return f'gpx_ld(c->{tok})'
    raise Unsupported(f'alu source {tok}')


def alu(mn, ops):
    """C for one ALU instruction (no moves)."""
    o = ops.split(',') if ops else []
    if mn == 'nop':
        return []
    if mn in ('add', 'sub'):
        s, d = o
        fn = 'gpx_add' if mn == 'add' else 'gpx_sub'
        return [f'{{ uint32_t f; int64_t r = {fn}(c, c->{d}, {alu_src(s)},'
                f' &f); c->{d} = r; gpx_eunz(c, r);'
                f' c->sr = (c->sr & ~(GPX_V | GPX_C)) | f; }}']
    if mn == 'cmp':
        s, d = o
        return [f'{{ uint32_t f; int64_t r = gpx_sub(c, c->{d}, {alu_src(s)},'
                f' &f); gpx_eunz(c, r);'
                f' c->sr = (c->sr & ~(GPX_V | GPX_C)) | f; }}']
    if mn == 'cmpu':
        s, d = o
        src = (f'gpx_ld(gpx_acc24(c, c->{s}))' if s in ACCS
               else f'gpx_ld(c->{s})')
        return [f'{{ uint32_t f; int64_t r = gpx_sub(c, c->{d}, {src}, &f);'
                f' c->sr &= ~(GPX_V | GPX_C | GPX_Z | GPX_N);'
                f' c->sr |= f & GPX_C; if (r == 0) c->sr |= GPX_Z;'
                f' if (r < 0) c->sr |= GPX_N; }}']
    if mn == 'clr':
        (d,) = o
        return [f'c->{d} = 0; c->sr = (c->sr & ~(GPX_E | GPX_N | GPX_V)) |'
                f' GPX_U | GPX_Z;']
    if mn in ('and', 'or'):
        s, d = o
        if s.startswith('#'):
            v, _ = imm(s)
            src = f'0x{v:X}u'
        else:
            src = reg(s)
        op = '&' if mn == 'and' else '|'
        return [f'{{ uint32_t w = gpx_a1(c->{d}) {op} {src};'
                f' c->{d} = gpx_set_a1(c->{d}, w);'
                f' c->sr &= ~(GPX_N | GPX_Z | GPX_V);'
                f' if (w & 0x800000) c->sr |= GPX_N;'
                f' if (!w) c->sr |= GPX_Z; }}']
    if mn in ('asl', 'asr'):
        if len(o) == 1:
            n, s, d = '1', o[0], o[0]
        else:
            cnt, s, d = o
            if cnt.startswith('#'):
                n = str(imm(cnt)[0])
            else:
                n = f'(({reg(cnt)} & 0x3F) > 55 ? 55 : ({reg(cnt)} & 0x3F))'
        if mn == 'asl':
            return [f'{{ int n = {n}; int64_t v = c->{s};'
                    f' uint32_t f = 0;'
                    f' if (n) {{ if (((uint64_t)v >> (56 - n)) & 1) f |= GPX_C;'
                    f' int64_t top = v >> (55 - n);'
                    f' if (top != 0 && top != -1) f |= GPX_V | GPX_L; }}'
                    f' int64_t r = gpx_n56((int64_t)((uint64_t)v << n));'
                    f' c->{d} = r; c->sr = (c->sr & ~(GPX_C | GPX_V)) | f;'
                    f' gpx_eunz(c, r); }}']
        return [f'{{ int n = {n}; int64_t v = c->{s}; uint32_t f = 0;'
                f' if (n && ((v >> (n - 1)) & 1)) f |= GPX_C;'
                f' int64_t r = v >> n; c->{d} = r;'
                f' c->sr = (c->sr & ~(GPX_C | GPX_V)) | f; gpx_eunz(c, r); }}']
    if mn in ('mpy', 'mac', 'macr'):
        sign, s1 = o[0][0], o[0][1:]
        s2, d = o[1], o[2]
        neg = 'true' if sign == '-' else 'false'
        prod = f'gpx_mul({reg(s1)}, {reg(s2)}, {neg})'
        if mn == 'mpy':
            # under SR.SM the product saturates too (-1 * -1), V and L set
            return [f'{{ uint32_t f = 0; int64_t r = gpx_sat(c, {prod}, &f);'
                    f' c->{d} = r; gpx_eunz(c, r);'
                    f' c->sr = (c->sr & ~GPX_V) | f; }}']
        if mn == 'macr':
            # The JIT's macr: V (and L) from the addends' signs against the
            # rounded result's, so a negative sum that rounds to 0 is an
            # overflow (probed with tools/dsp2c/lockstep probe).
            return [f'{{ uint32_t f; int64_t d_ = c->{d}, p_ = {prod};'
                    f' int64_t r = gpx_add(c, d_, p_, &f); r = gpx_rnd(c, r);'
                    f' f &= ~(GPX_V | GPX_L);'
                    f' if ((d_ < 0) == (p_ < 0) && (r < 0) != (d_ < 0))'
                    f' f |= GPX_V | GPX_L;'
                    f' c->{d} = r; gpx_eunz(c, r);'
                    f' c->sr = (c->sr & ~GPX_V) | (f & ~GPX_C); }}']
        return [f'{{ uint32_t f; int64_t r = gpx_add(c, c->{d}, {prod}, &f);'
                f' c->{d} = r; gpx_eunz(c, r);'
                f' c->sr = (c->sr & ~GPX_V) | (f & ~GPX_C); }}']
    raise Unsupported(f'alu {mn} {ops}')


ALU_MN = {'add', 'sub', 'cmp', 'cmpu', 'clr', 'and', 'or', 'asl', 'asr',
          'mpy', 'mac', 'macr', 'nop'}

COND = {
    'eq': '(c->sr & GPX_Z)',
    'ne': '!(c->sr & GPX_Z)',
    'lt': '(!!(c->sr & GPX_N) != !!(c->sr & GPX_V))',
    'ge': '(!!(c->sr & GPX_N) == !!(c->sr & GPX_V))',
    'gt': '(!(c->sr & GPX_Z) && (!!(c->sr & GPX_N) == !!(c->sr & GPX_V)))',
    'le': '((c->sr & GPX_Z) || (!!(c->sr & GPX_N) != !!(c->sr & GPX_V)))',
    'cc': '!(c->sr & GPX_C)', 'cs': '(c->sr & GPX_C)',
    'pl': '!(c->sr & GPX_N)', 'mi': '(c->sr & GPX_N)',
}


def target(tok):
    m = re.fullmatch(r'>?\$([0-9a-f]+)', tok)
    if not m:
        raise Unsupported(f'target {tok}')
    return int(m.group(1), 16)


def bit_src(tok):
    if is_mem(tok):
        return Ea(tok).read()
    if tok in ACCS:
        return f'gpx_acc24(c, c->{tok})'
    return reg(tok)


# ---- one instruction ----

def translate(addr, n, text, ctx):
    """C statements for one instruction. ctx: loop ends, halt, etc."""
    nxt = addr + n
    toks = text.split()
    mn = toks[0]
    out = []

    tr = f'GPX_TRACE(c, 0x{addr:X}u);'

    def go(t):
        ctx['targets'].add(t)
        return [tr,
                f'if (--c->budget < 0) {{ c->pc = 0x{t:X}u; return -1; }}',
                f'goto L_{t:04X};']

    if mn == 'dc':
        return [f'c->pc = 0x{addr:X}u; return -2;']
    if mn == 'move':
        r, w, p = moves(toks[1:])
        return r + w + p
    if mn == 'mpyi':
        sign, rest_ = toks[1][0], toks[1][1:]
        i_, s2, d = rest_.split(',')
        v, _ = imm(i_)
        neg = 'true' if sign == '-' else 'false'
        return [f'{{ uint32_t f = 0; int64_t r = gpx_sat(c,'
                f' gpx_mul(0x{v:X}u, {reg(s2)}, {neg}), &f);'
                f' c->{d} = r; gpx_eunz(c, r); c->sr = (c->sr & ~GPX_V) | f; }}']
    if mn in ALU_MN:
        ops = toks[1] if mn != 'nop' else ''
        rest = toks[2:] if mn != 'nop' else toks[1:]
        cond = None
        if rest and rest[-1].startswith('if'):
            cond = rest[-1][2:]
            rest = rest[:-1]
        r, w, p = moves(rest)
        a = alu(mn, ops)
        if cond:
            if rest:
                raise Unsupported('IFcc with moves')
            # IFcc: the ALU op runs on the condition, the CCR unchanged
            return [f'if ({COND[cond]}) {{ uint32_t sr_ = c->sr;'] + a + \
                   ['c->sr = sr_; }']
        return r + a + w + p
    if mn == 'movec':
        s, d = toks[1].split(',')
        if s.startswith('#'):
            v, _ = imm(s)
            if d.startswith('m') and v != 0xFFFFFF:
                raise Unsupported('modulo addressing')
            return [f'{reg(d)} = 0x{v:X}u;']
        if d.startswith('m') and s != 'm0' and not s.startswith('m'):
            raise Unsupported('m from a register')
        return [f'{reg(d)} = {reg(s)};']
    if mn == 'movep':
        s, d = toks[1].split(',')
        v, short, _ = src24(s)
        v = f'0x{v:X}u' if isinstance(v, int) else v
        e = Ea(d)
        out = [e.write(v)]
        if e.const == 0xFFFFC4:
            out.append(f'if (c->halt) {{ c->halt = false; c->pc = 0x{nxt:X}u;'
                       f' {tr} return 0; }}')
        return out
    if mn == 'movem':
        s, d = toks[1].split(',')
        e = Ea(s)
        out = [f'{reg(d)} = {e.read()};']
        if e.post:
            out.append(e.post)
        return out
    if mn == 'lua':
        s, d = toks[1].split(',')
        m = re.fullmatch(r'\((r[0-7])\+(\d+)\)', s)
        if not m:
            raise Unsupported(text)
        return [f'{reg(d)} = ({reg(m.group(1))} + {m.group(2)}u) & 0xFFFFFF;']
    if mn in ('bset', 'bclr'):
        b, d = toks[1].split(',')
        bit = int(b[1:], 16) if b.startswith('#$') else int(b[1:])
        r_ = reg(d)
        op = f'v | (1u << {bit})' if mn == 'bset' else f'v & ~(1u << {bit})'
        if d == 'sr':
            # the JIT leaves C alone when SR is the target
            return [f'{{ uint32_t v = c->sr; c->sr = {op}; }}']
        return [f'{{ uint32_t v = {r_}; uint32_t cb = (v >> {bit}) & 1;'
                f' {r_} = {op}; c->sr = (c->sr & ~GPX_C) | cb; }}']
    if mn in ('brset', 'brclr', 'jset', 'jclr'):
        b, s, t = toks[1].split(',')
        bit = int(b[1:])
        t = target(t)
        test = f'(({bit_src(s)} >> {bit}) & 1)'
        if mn.endswith('clr'):
            test = f'!{test}'
        return [f'if ({test}) {{'] + go(t) + ['}']
    if mn in ('bra', 'jmp'):
        return go(target(toks[1]))
    if mn[0] in 'bj' and mn[1:] in COND:
        return [f'if ({COND[mn[1:]]}) {{'] + go(target(toks[1])) + ['}']
    if mn in ('bsr', 'jsr'):
        t = target(toks[1])
        ctx['targets'].add(t)
        ctx['returns'].add(nxt)
        return [f'gpx_push(c, 0x{nxt:X}u, c->sr);', tr, f'goto L_{t:04X};']
    if mn == 'rts':
        return ['{ uint32_t pc_, sr_; gpx_pop(c, &pc_, &sr_); c->pc = pc_;',
                tr, 'goto dispatch; }']
    if mn == 'dor':
        cnt, end = toks[1].split(',')
        end = target(end)
        if cnt.startswith('#'):
            v = f'0x{imm(cnt)[0]:X}u'
        else:
            v = f'({bit_src(cnt)} & 0xFFFF)'
        ctx['loops'].setdefault(end - 1, set()).add(nxt)
        return [f'{{ uint32_t n_ = {v}; gpx_push(c, c->la, c->lc);',
                f'c->la = 0x{end - 1:X}u; gpx_push(c, 0x{nxt:X}u, c->sr);',
                'c->sr |= GPX_LF; c->lc = n_; }']
    if mn == 'rep':
        return ['REP']                  # expanded by the caller
    raise Unsupported(text)


# E, U, N and Z: which instructions rewrite all four, and which read them.
EUNZ_KILL = {'add', 'sub', 'cmp', 'mpy', 'mac', 'macr', 'mpyi', 'asl', 'asr',
             'clr'}
CONTROL = {'bra', 'jmp', 'bsr', 'jsr', 'rts', 'brset', 'brclr', 'jset',
           'jclr', 'dor', 'rep', 'dc'}


def flag_liveness(insns, ctx, code_at):
    """Addresses whose E/U/N/Z results are never read: within a basic block
    a later instruction rewrites all four before anything reads SR. Every
    block boundary (a branch target, a return point, a loop's start or end,
    after control flow or a frame halt) counts all four live."""
    leaders = set(ctx['targets']) | set(ctx['returns']) | {code_at}
    for la, starts in ctx['loops'].items():
        leaders |= starts
        leaders.add(la + 1)
    ends = set(ctx['loops'])                # a loop's last word
    dead = set()
    live = True                             # E/U/N/Z live after this insn
    for i in range(len(insns) - 1, -1, -1):
        addr, n, text = insns[i]
        toks = text.split()
        mn = toks[0]
        nxt = addr + n
        boundary = (nxt in leaders or (addr + n - 1) in ends or
                    mn in CONTROL or mn == 'movep' or
                    (mn[0] in 'bj' and mn[1:] in COND))
        if boundary:
            live = True
        cond = toks[-1].startswith('if') if len(toks) > 1 else False
        kills = mn in EUNZ_KILL and not cond
        reads = (cond or mn == 'movec' and 'sr' in toks[1].split(',')[0] or
                 mn in ('bset', 'bclr') and toks[1].endswith(',sr') or
                 (mn[0] in 'bj' and mn[1:] in COND))
        if kills and not live:
            dead.add(addr)
        # live before this instruction
        if kills:
            live = False
        if reads:
            live = True
        if addr in leaders:
            live = True
    return dead


def generate(insns, name, kernel_words, kernel_hash, code_at, code_words,
             code_hash):
    ctx = {'targets': set(), 'returns': set(), 'loops': {}}
    bodies = {}
    for addr, n, text in insns:
        try:
            bodies[addr] = translate(addr, n, text, ctx)
        except Unsupported as e:
            raise SystemExit(f'P:{addr:04X} {text}: unsupported ({e})')
    by_addr = {a: (n, t) for a, n, t in insns}
    eunz_dead = flag_liveness(insns, ctx, code_at)
    lines = []
    w = lines.append
    w('/* Generated by xboxrecomp tools/dsp2c/dsp2c.py: do not edit. */')
    w('#include "apu_gpx.h"')
    w('')
    w('/* GPX_TRACE(c, addr): after each instruction (a lockstep test). */')
    w('#ifndef GPX_TRACE')
    w('#define GPX_TRACE(c, addr) ((void)0)')
    w('#endif')
    w('')
    w('static int run(GpxCore *c)')
    w('{')
    w('dispatch:')
    w('    switch (c->pc) {')
    for addr, n, text in insns:
        w(f'    case 0x{addr:X}u: goto L_{addr:04X};')
    w('    default: return -2;')
    w('    }')
    addrs = [a for a, _, _ in insns]
    skip_next = False
    for i, (addr, n, text) in enumerate(insns):
        w(f'L_{addr:04X}: /* {text} */')
        if addr == code_at:
            w('    if (!c->code_ok) { uint32_t pc_, sr_;'
              ' gpx_pop(c, &pc_, &sr_); c->pc = pc_; goto dispatch; }')
        body = bodies[addr]
        if body == ['REP']:
            nxt_addr, nxt_n, nxt_text = insns[i + 1]
            if nxt_addr + nxt_n - 1 in ctx['loops']:
                raise SystemExit(f'P:{nxt_addr:04X}: a repeated instruction'
                                 ' ends a loop')
            cnt = text.split()[1]
            src = bit_src(cnt)
            w(f'    {{ uint32_t k_ = {src} & 0xFFFF; if (!k_) k_ = 0x10000;')
            w(f'    GPX_TRACE(c, 0x{addr:X}u);')
            w('      for (; k_; k_--) {')
            for s_ in bodies[nxt_addr]:
                w('        ' + s_)
            w(f'        GPX_TRACE(c, 0x{nxt_addr:X}u);')
            w('      } }')
            w(f'    goto L_{nxt_addr + nxt_n:04X};')
            ctx['targets'].add(nxt_addr + nxt_n)
            continue
        w('    {')
        for s in body:
            if addr in eunz_dead:
                s = s.replace(' gpx_eunz(c, r);', '')
            w('    ' + s)
        w('    }')
        trace = f'    GPX_TRACE(c, 0x{addr:X}u);'
        last = addr + n - 1
        mn0 = text.split()[0]
        if last in ctx['loops'] and (
                mn0 in ('bsr', 'jsr', 'rts', 'bra', 'jmp', 'rep', 'brset',
                        'brclr', 'jset', 'jclr', 'movep') or
                (mn0[0] in 'bj' and mn0[1:] in COND)):
            raise SystemExit(f'P:{addr:04X} {text}: control flow ends a loop')
        if last in ctx['loops']:
            starts = ctx['loops'][last]
            w(f'    if ((c->sr & GPX_LF) && c->la == 0x{last:X}u) {{')
            w('        c->lc = (c->lc - 1) & 0xFFFF;')
            w('        if (c->lc) {')
            w('    ' + trace)
            w('            if (--c->budget < 0) { c->pc = c->ssh[c->sp];'
              ' return -1; }')
            if len(starts) == 1:
                w(f'            goto L_{next(iter(starts)):04X};')
            else:
                w('            c->pc = c->ssh[c->sp]; goto dispatch;')
            w('        }')
            w('        { uint32_t pc_, sr_; gpx_pop(c, &pc_, &sr_);')
            w('          c->sr = (c->sr & ~GPX_LF) | (sr_ & GPX_LF);')
            w('          gpx_pop(c, &c->la, &c->lc); }')
            w('    }')
        w(trace)
    end = insns[-1][0] + insns[-1][1]
    w(f'L_{end:04X}:')
    w(f'    c->pc = 0x{end:X}u; return -2;')
    w('}')
    w('')
    w('static const GpxProgram k_program = {')
    w(f'    "{name}", 0x{kernel_words:X}u, 0x{kernel_hash:016x}ull,')
    w(f'    0x{code_at:X}u, 0x{code_words:X}u, 0x{code_hash:016x}ull, run,')
    w('};')
    w('')
    w('__attribute__((constructor)) static void register_program(void)')
    w('{')
    w('    apu_gpx_register(&k_program);')
    w('}')
    # labels that are jumped to but are not instructions
    missing = [t for t in ctx['targets'] if t not in by_addr and t != end]
    if missing:
        raise SystemExit(f'targets outside the program: {missing}')
    return '\n'.join(lines) + '\n'


def main():
    args = sys.argv[1:]
    disasm = DISASM
    if '--disasm' in args:
        i = args.index('--disasm')
        disasm = args[i + 1]
        del args[i:i + 2]
    path, kw, at, cw, name, out = args
    kw, at, cw = int(kw, 0), int(at, 0), int(cw, 0)
    raw = open(path, 'rb').read()
    words = [v & 0xFFFFFF for v in
             struct.unpack(f'<{len(raw) // 4}I', raw)]
    total = at + cw
    insns = disassemble(path, total, disasm)
    code = generate(insns, name, kw, fnv(words[:kw]), at, cw,
                    fnv(words[at:at + cw]))
    open(out, 'w').write(code)
    print(f'{out}: {len(insns)} instructions, kernel {kw:#x} words,'
          f' code {at:#x}+{cw:#x}')


if __name__ == '__main__':
    main()
