#!/usr/bin/env python3
# Faithful python port of codec2 src/generate_codebook.c (LGPL 2.1).
# Emits identical C to the original host tool so codebooks match the library ABI.
import sys, math

def parse_floats(path):
    vals = []
    with open(path, 'r') as f:
        for line in f:
            # comments start with '#'
            h = line.find('#')
            if h >= 0:
                line = line[:h]
            for tok in line.replace(',', ' ').split():
                try:
                    vals.append(float(tok))
                except ValueError:
                    pass
    return vals

def fmt_g(x):
    # match C printf %g
    return '%g' % x

def main():
    array_name = sys.argv[1]
    files = sys.argv[2:]
    cbs = []
    for p in files:
        v = parse_floats(p)
        k = int(v[0]); m = int(v[1])
        cb = v[2:2 + k * m]
        assert len(cb) == k * m, f"{p}: expected {k*m} got {len(cb)}"
        cbs.append((k, m, cb, p))
    out = []
    out.append('/* THIS IS A GENERATED FILE. Edit generate_codebook.c and its input */\n')
    out.append('/*\n * This intermediary file and the files that used to create it are under \n'
               ' * The LGPL. See the file COPYING.\n */\n')
    out.append('#include "defines.h"\n')
    for i, (k, m, cb, p) in enumerate(cbs):
        out.append('  /* %s */' % p)
        out.append('#ifdef __EMBEDDED__')
        out.append('static const float codes%d[] = {' % i)
        out.append('#else')
        out.append('static float codes%d[] = {' % i)
        out.append('#endif')
        limit = k * m
        row = ''
        for j in range(limit):
            row += '  ' + fmt_g(cb[j])
            if j < limit - 1:
                row += ','
            if ((j + 1) % k) == 0:
                out.append(row); row = ''
        if row:
            out.append(row)
        out.append('};')
    out.append('')
    out.append('const struct lsp_codebook %s[] = {' % array_name)
    for i, (k, m, cb, p) in enumerate(cbs):
        out.append('  /* %s */' % p)
        log2m = int(round(math.log(m) / math.log(2)))
        out.append('  {')
        out.append('    %d,' % k)
        out.append('    %d,' % log2m)
        out.append('    %d,' % m)
        out.append('    codes%d' % i)
        out.append('  },')
    out.append('  { 0, 0, 0, 0 }')
    out.append('};')
    sys.stdout.write('\n'.join(out) + '\n')

if __name__ == '__main__':
    main()
