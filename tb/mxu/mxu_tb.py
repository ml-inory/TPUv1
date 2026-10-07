#!/usr/bin/env python3
"""Cycle-accurate reference model for the MXU systolic array.

Reads the trace produced by tb/mxu/mxu_tb.sv and compares the observed
psum_out / psum_out_valid against a model of the intended dataflow:

  * row i : act_in[i] / act_in_valid -> delay_chain(DEPTH=i)
  * col j : psum_in[j]               -> delay_chain(DEPTH=j)
  * PE(i,j): weight-stationary MAC; act flows left->right, psum top->bottom
  * psum_out[j]    = PE(ROW-1,j).psum_out
    psum_out_valid = AND over j of PE(ROW-1,j).psum_out_valid

Usage:  mxu_tb.py <trace-file>
Exit code 0 = match, 1 = mismatch (or bad trace).
"""

import sys

MASK = 0xFFFFFFFF


def s32(x):
    """Wrap to a signed 32-bit value (two's complement)."""
    x &= MASK
    return x - 0x100000000 if (x & 0x80000000) else x


def num(tok):
    """Trace token -> int, or None for x/z."""
    try:
        return int(tok)
    except ValueError:
        return None


def read_params(path):
    with open(path) as fh:
        for line in fh:
            if line.startswith('#'):
                p = {}
                for kv in line[1:].split():
                    if '=' in kv:
                        k, v = kv.split('=', 1)
                        p[k] = int(v)
                if 'ROW' in p:
                    return p
    raise SystemExit("mxu_tb.py: no '# ... ROW=.. COL=..' header in " + path)


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else 'mxu_trace.txt'
    p = read_params(path)
    ROW, COL = p['ROW'], p['COL']

    # ---- reference model state -------------------------------------------
    adcnt = [0] * ROW; adout = [None] * ROW      # row act   delay chains (DEPTH=i)
    vdcnt = [0] * ROW; vdout = [None] * ROW      # row valid delay chains (DEPTH=i)
    pdcnt = [0] * COL; pdout = [None] * COL      # col psum  delay chains (DEPTH=j)

    w    = [[0] * COL for _ in range(ROW)]       # PE weight_reg
    psum = [[None] * COL for _ in range(ROW)]    # PE psum_out
    aout = [[None] * COL for _ in range(ROW)]    # PE act_out
    pov  = [[0] * COL for _ in range(ROW)]       # PE psum_out_valid
    aov  = [[0] * COL for _ in range(ROW)]       # PE act_out_valid

    checks = errors = compared = 0

    with open(path) as fh:
        for line in fh:
            if not line.strip() or line.startswith('#'):
                continue
            t = line.split()
            k = 0
            cyc    = int(t[k]);                              k += 1
            cmp_en = int(t[k]);                              k += 1
            rst    = int(t[k]);                              k += 1
            lw     = int(t[k]);                              k += 1
            av     = int(t[k]);                              k += 1
            pv     = int(t[k]);                              k += 1
            act_in    = [int(t[k + i]) for i in range(ROW)]; k += ROW
            psum_in   = [int(t[k + j]) for j in range(COL)]; k += COL
            weight_in = [[int(t[k + i * COL + j]) for j in range(COL)]
                         for i in range(ROW)];               k += ROW * COL
            obs       = [num(t[k + j]) for j in range(COL)]; k += COL
            obs_valid = num(t[k]);                           k += 1

            # ---------- one clock of the reference model ----------
            # weights persist unless rst/load_weight touches them
            n_w    = [row[:] for row in w]
            n_psum = [[None] * COL for _ in range(ROW)]
            n_aout = [[None] * COL for _ in range(ROW)]
            n_pov  = [[0] * COL for _ in range(ROW)]
            n_aov  = [[0] * COL for _ in range(ROW)]

            for i in range(ROW):
                for j in range(COL):
                    iact  = adout[i] if j == 0 else aout[i][j - 1]
                    iactv = vdout[i] if j == 0 else aov[i][j - 1]
                    ipsum = pdout[j] if i == 0 else psum[i - 1][j]

                    if rst:
                        n_w[i][j]    = 0
                        n_psum[i][j] = psum[i][j]
                        n_aout[i][j] = aout[i][j]
                    elif lw:
                        n_w[i][j]    = weight_in[i][j]
                        n_psum[i][j] = psum[i][j]
                        n_aout[i][j] = aout[i][j]
                    else:
                        if iactv:
                            n_psum[i][j] = (None if (iact is None or ipsum is None)
                                            else s32(ipsum + w[i][j] * iact))
                            n_aout[i][j] = iact
                        else:
                            n_psum[i][j] = psum[i][j]     # hold
                            n_aout[i][j] = aout[i][j]     # hold
                        n_pov[i][j] = iactv
                        n_aov[i][j] = iactv

            # delay chains (PE state already sampled above)
            for i in range(ROW):
                if rst:
                    adcnt[i] = 0
                    vdcnt[i] = 0
                else:
                    if adcnt[i] < i:
                        adcnt[i] += 1
                    else:
                        adout[i] = act_in[i]
                    if vdcnt[i] < i:
                        vdcnt[i] += 1
                    else:
                        vdout[i] = av
            for j in range(COL):
                if rst:
                    pdcnt[j] = 0
                else:
                    if pdcnt[j] < j:
                        pdcnt[j] += 1
                    else:
                        pdout[j] = psum_in[j]

            w, psum, aout, pov, aov = n_w, n_psum, n_aout, n_pov, n_aov

            g_out = [n_psum[ROW - 1][j] for j in range(COL)]
            g_valid = 1
            for j in range(COL):
                if g_valid == 0:
                    continue
                v = n_pov[ROW - 1][j]
                if v == 0:
                    g_valid = 0
                elif v is None:
                    g_valid = None

            if not cmp_en:
                continue

            compared += 1
            for j in range(COL):
                checks += 1
                if obs[j] != g_out[j]:
                    errors += 1
                    if errors <= 20:
                        print("  [FAIL] cyc %d psum_out[%d] got=%s exp=%s"
                              % (cyc, j, obs[j], g_out[j]))
            checks += 1
            if obs_valid != g_valid:
                errors += 1
                if errors <= 20:
                    print("  [FAIL] cyc %d psum_out_valid got=%s exp=%s"
                          % (cyc, obs_valid, g_valid))

    print("-------------------------------------------------")
    if errors == 0:
        print(" *** REFERENCE CHECK PASSED ***  (%d cycles, %d checks, 0 failures)"
              % (compared, checks))
        print("-------------------------------------------------")
        return 0
    print(" *** REFERENCE CHECK FAILED ***  (%d/%d checks failed)" % (errors, checks))
    print("-------------------------------------------------")
    return 1


if __name__ == '__main__':
    sys.exit(main())
