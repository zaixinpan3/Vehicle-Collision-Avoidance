"""Controller time per frame of campaign arms (study script).
Usage: python3 -I timing.py <label>=<out-dir> [<label>=<out-dir> ...]
For each arm, exact (seed 0) and noisy runs separately: quantiles of the
controller time per frame, the frames over 50 and 100 ms, and the median
time by plan length (holds). Prints the table."""
import glob, json, os, statistics as st, sys


def quantile(a, p):
    a = sorted(a)
    if not a: return float('nan')
    x = p * (len(a) - 1); i = int(x); f = x - i
    return a[i] if i + 1 >= len(a) else a[i] + (a[i + 1] - a[i]) * f


def frames(out, exact):
    rows = []
    for f in glob.glob(os.path.join(out, '*-summary.json')):
        if os.path.basename(f).startswith('seed0-') != exact: continue
        fr = json.load(open(f)).get('frames') or []
        fr = [fr] if isinstance(fr, dict) else fr
        rows += [(x['controllerSeconds'], x['horizonSteps']) for x in fr
                 if x.get('controllerSeconds') is not None and x.get('horizonSteps') is not None]
    return rows


def main():
    bands = [(0, 20), (21, 40), (41, 80), (81, 10 ** 6)]
    for kind, exact in (('exact', True), ('noisy', False)):
        print(f'===== {kind} =====')
        for spec in sys.argv[1:]:
            label, out = spec.split('=', 1)
            rows = frames(out, exact)
            t = [1000 * s for s, _ in rows]
            if not t: continue
            print(f'[{label}] frames {len(t)}: ms median {st.median(t):.1f} p95 {quantile(t, .95):.1f} '
                  f'p99 {quantile(t, .99):.1f} max {max(t):.0f}; over 50 ms {sum(x > 50 for x in t)}, over 100 ms {sum(x > 100 for x in t)}')
            parts = []
            for lo, hi in bands:
                sel = [1000 * s for s, h in rows if lo <= h <= hi]
                if sel:
                    top = f'{hi}' if hi < 10 ** 6 else '+'
                    parts.append(f'{lo}-{top} holds: {len(sel)} frames, median {st.median(sel):.1f} p95 {quantile(sel, .95):.1f} ms')
            print('   ' + '; '.join(parts))


if __name__ == '__main__':
    main()
