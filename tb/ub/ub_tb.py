#!/usr/bin/env python3
"""Cycle-accurate reference model for the UB (unified buffer).

Reads the trace produced by tb/ub/ub_tb.sv and compares the observed rd_data
against a model of the intended behavior of rtl/ub.sv:

  * one address holds a whole line of WIDTH*WORD_SIZE bits (WORD_SIZE words
    of WIDTH bits); the model treats a line as one integer
  * synchronous write : wr_en -> mem[wr_addr] <= wr_data
  * synchronous read  : rd_en -> rd_data <= mem[rd_addr] (1 clock of latency)
  * read and write in the same cycle to the same address: the read returns
    the OLD line (both are non-blocking assignments in rtl/ub.sv)
  * rd_en=0 -> rd_data holds its previous value
  * synchronous reset: clears the whole memory AND rd_data, and wins over a
    concurrent write
  * values are truncated to WIDTH*WORD_SIZE bits

Usage:  ub_tb.py <trace-file>
Exit code 0 = match, 1 = mismatch (or bad trace).
"""

import sys


def num(tok):
    """Trace token -> int, or None for x/z."""
    try:
        return int(tok, 16)
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
                if 'DEPTH' in p:
                    return p
    raise SystemExit("ub_tb.py: no '# ... WIDTH=.. WORD_SIZE=.. DEPTH=..' header in "
                     + path)


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else 'ub_trace.txt'
    p = read_params(path)
    WIDTH, WORD_SIZE, DEPTH = p['WIDTH'], p['WORD_SIZE'], p['DEPTH']
    mask = (1 << (WIDTH * WORD_SIZE)) - 1

    # ---- reference model state -------------------------------------------
    mem      = [0] * DEPTH
    rd_exp   = 0              # rd_data is cleared by the reset in cycle 1
    compared = checks = errors = 0

    with open(path) as fh:
        for line in fh:
            if not line.strip() or line.startswith('#'):
                continue
            t = line.split()
            k = 0
            cyc     = int(t[k]); k += 1
            rst     = int(t[k]); k += 1
            rd_en   = int(t[k]); k += 1
            rd_addr = int(t[k]); k += 1
            wr_en   = int(t[k]); k += 1
            wr_addr = int(t[k]); k += 1
            wr_data = num(t[k]); k += 1
            obs     = num(t[k]); k += 1

            # ---------- one clock of the reference model ----------
            if rst:
                mem = [0] * DEPTH          # reset wins over any write
                rd_exp = 0                 # rd_data is cleared too
            else:
                if rd_en:
                    rd_exp = mem[rd_addr]  # sampled before this cycle's write
                if wr_en:
                    mem[wr_addr] = wr_data & mask

            compared += 1
            checks += 1
            if obs != rd_exp:
                errors += 1
                if errors <= 20:
                    print("  [FAIL] cyc %d rd_data got=%s exp=%s"
                          " (rst=%d rd_en=%d rd_addr=%d wr_en=%d wr_addr=%d"
                          " wr_data=%s)"
                          % (cyc, '%x' % obs if obs is not None else obs,
                             '%x' % rd_exp, rst, rd_en, rd_addr,
                             wr_en, wr_addr,
                             '%x' % wr_data if wr_data is not None else wr_data))

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
