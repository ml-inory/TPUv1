#!/usr/bin/env python3
"""Cycle-accurate reference model for the WeightFIFO.

Reads the trace produced by tb/weight_fifo/weight_fifo_tb.sv and compares the
observed rd_data / full / empty against a model of what the FIFO must do:

  * one entry is a whole WIDTH*DEPTH-bit weight line, treated as one integer
  * rst            -> the queue is emptied (rd_data keeps its old value)
  * wr_en, ~full   -> wr_data is enqueued at the tail
  * rd_en, ~empty  -> the head is dequeued and shows up on rd_data during that
                      same clock; rd_data holds when no entry is dequeued
  * rd_en + wr_en in the same clock: both take effect, the length is unchanged
    and the new element goes to the tail of the (shifted) queue
  * full/empty are combinational from the number of entries

Usage:  weight_fifo_tb.py <trace-file>
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
                if 'TILE' in p:
                    return p
    raise SystemExit("weight_fifo_tb.py: no '# ... TILE=.. WIDTH=.. DEPTH=..' header in "
                     + path)


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else 'weight_fifo_trace.txt'
    p = read_params(path)
    TILE, WIDTH, DEPTH = p['TILE'], p['WIDTH'], p['DEPTH']
    mask = (1 << (WIDTH * DEPTH)) - 1

    # ---- reference model state -------------------------------------------
    queue  = []              # oldest first
    rd_exp = None            # rd_data is X until the first pop
    compared = checks = errors = 0
    reported = 0

    with open(path) as fh:
        for line in fh:
            if not line.strip() or line.startswith('#'):
                continue
            t = line.split()
            k = 0
            cyc     = int(t[k]); k += 1
            rst     = int(t[k]); k += 1
            wr_en   = int(t[k]); k += 1
            rd_en   = int(t[k]); k += 1
            obs_f   = num(t[k]); k += 1
            obs_e   = num(t[k]); k += 1
            wr_data = num(t[k]); k += 1
            obs_d   = num(t[k]); k += 1

            # ---------- one clock of the reference model ----------
            if rst:
                queue = []
            else:
                rd_fire = rd_en and len(queue) > 0
                wr_fire = wr_en and len(queue) < TILE
                if rd_fire:
                    rd_exp = queue.pop(0)
                if wr_fire:
                    queue.append(wr_data & mask)

            g_full  = 1 if len(queue) == TILE else 0
            g_empty = 1 if len(queue) == 0 else 0

            compared += 1
            checks += 1
            if obs_d != rd_exp:
                errors += 1
                if reported < 20:
                    reported += 1
                    print("  [FAIL] cyc %d rd_data got=%s exp=%s"
                          " (rst=%d rd_en=%d wr_en=%d wr_data=%s)"
                          % (cyc, '%x' % obs_d if obs_d is not None else obs_d,
                             '%x' % rd_exp if rd_exp is not None else rd_exp,
                             rst, rd_en, wr_en,
                             '%x' % wr_data if wr_data is not None else wr_data))
            checks += 1
            if obs_f != g_full or obs_e != g_empty:
                errors += 1
                if reported < 20:
                    reported += 1
                    print("  [FAIL] cyc %d flags got full=%s empty=%s exp full=%d empty=%d (queued=%d)"
                          % (cyc, obs_f, obs_e, g_full, g_empty, len(queue)))

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
