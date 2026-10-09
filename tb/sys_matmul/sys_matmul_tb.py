#!/usr/bin/env python3
"""System level matmul reference model.

Reads the trace produced by tb/sys_matmul/sys_matmul_tb.sv (random matrices
generated in the tb, results observed on the real RTL) and checks:

  * every pass result equals the partial sum of that pass, i.e.
        R[b][p][j] == sum_{k in pass} sum_i A[b][k][i] * W[b][i][j]
  * the sum of a batch's pass results equals the full K_TOTAL matmul result
        out[b][j] == sum_{k < K_TOTAL} sum_i A[b][k][i] * W[b][i][j]
    which is the multi pass accumulation across the accumulator's double
    buffered groups.

Trace records:
  # SYSMAT ROW=<n> COL=<n> PASS_K=<n> BLOCK_NUM=<n>
  B <batch> <K_TOTAL>
  W <batch> <i> <j> <value>
  A <batch> <k> <i> <value>
  R <batch> <pass> <k0> <n> <col0> ... <colCOL-1>

Usage:  sys_matmul_tb.py <trace-file>
Exit code 0 = match, 1 = mismatch (or bad trace).
"""

import sys


def read_trace(path):
    shape = {}
    batches = {}     # b -> {"K": int, "W": {(i, j): v}, "A": {(k, i): v}}
    results = []     # (b, pass_idx, k0, n, [values])
    with open(path) as fh:
        for line in fh:
            if not line.strip():
                continue
            tok = line.split()
            if line.startswith('#'):
                for kv in line[1:].split():
                    if '=' in kv:
                        k, v = kv.split('=', 1)
                        shape[k] = int(v)
                continue
            tag = tok[0]
            if tag == 'B':
                b, ktot = int(tok[1]), int(tok[2])
                batches[b] = {"K": ktot, "W": {}, "A": {}}
            elif tag == 'W':
                b, i, j, v = (int(tok[1]), int(tok[2]), int(tok[3]), int(tok[4]))
                batches[b]["W"][(i, j)] = v
            elif tag == 'A':
                b, k, i, v = (int(tok[1]), int(tok[2]), int(tok[3]), int(tok[4]))
                batches[b]["A"][(k, i)] = v
            elif tag == 'R':
                b, p, k0, n = (int(tok[1]), int(tok[2]), int(tok[3]), int(tok[4]))
                results.append((b, p, k0, n, [int(x) for x in tok[5:]]))
    return shape, batches, results


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else 'sys_matmul_trace.txt'
    shape, batches, results = read_trace(path)
    if 'ROW' not in shape:
        raise SystemExit("sys_matmul_tb.py: no '# SYSMAT ROW=.. COL=..' header in " + path)
    ROW, COL, PASS_K = shape['ROW'], shape['COL'], shape['PASS_K']

    checks = errors = 0

    # per pass check
    pass_golden = {}
    for (b, p, k0, n, obs) in results:
        want = []
        for j in range(COL):
            s = 0
            for k in range(k0, k0 + n):
                for i in range(ROW):
                    s += batches[b]["A"][(k, i)] * batches[b]["W"][(i, j)]
            want.append(s)
        pass_golden[(b, p)] = want
        checks += 1
        ok = (len(obs) == COL) and all(o == w for o, w in zip(obs, want))
        if not ok:
            errors += 1
            print("  [FAIL] batch %d pass %d (k=%d..%d): got %s exp %s"
                  % (b, p, k0, k0 + n - 1, obs, want))
        else:
            print("  [PASS] batch %d pass %d (k=%d..%d) = %s"
                  % (b, p, k0, k0 + n - 1, obs))

    # per batch check: the sum of the passes is the full K_TOTAL result
    print("---------------------------------------------------------")
    for b in sorted(batches):
        ktot = batches[b]["K"]
        want = []
        for j in range(COL):
            s = 0
            for k in range(ktot):
                for i in range(ROW):
                    s += batches[b]["A"][(k, i)] * batches[b]["W"][(i, j)]
            want.append(s)
        got = [0] * COL
        covered = 0
        for (bb, p, k0, n, obs) in results:
            if bb == b:
                covered += n
                for j in range(COL):
                    got[j] += obs[j]
        checks += 1
        if covered != ktot or got != want:
            errors += 1
            print("  [FAIL] batch %d K_TOTAL=%d: sum of passes %s exp %s (covered %d acts)"
                  % (b, ktot, got, want, covered))
        else:
            print("  [PASS] batch %d K_TOTAL=%d multi pass sum = %s" % (b, ktot, got))

    print("---------------------------------------------------------")
    if errors == 0:
        print(" *** SYSTEM MATMUL REFERENCE CHECK PASSED ***  "
              "(%d batches, %d passes, %d checks, 0 failures)"
              % (len(batches), len(results), checks))
        print("---------------------------------------------------------")
        return 0
    print(" *** SYSTEM MATMUL REFERENCE CHECK FAILED ***  (%d/%d checks failed)"
          % (errors, checks))
    print("---------------------------------------------------------")
    return 1


if __name__ == '__main__':
    sys.exit(main())
