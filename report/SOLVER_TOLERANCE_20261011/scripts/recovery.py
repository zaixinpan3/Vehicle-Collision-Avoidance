"""Paired recovery-confirmation times of campaign arms (study script).
Usage: python3 -I recovery.py <label>=<out-dir> <label>=<out-dir> [...]
For every pair (first arm against each later one, then consecutive later
arms), exact (seed 0) and noisy runs separately, over the encounters both
arms recover: the median and mean difference of the confirmation time
(later minus earlier label) and how many are later, earlier or the same.
Prints the table."""
import glob, json, os, statistics as st, sys


def load(out):
    runs = {}
    for f in glob.glob(os.path.join(out, '*-summary.json')):
        r = json.load(open(f))
        rec = r.get('recovery') or {}
        if rec.get('recovered'):
            runs[os.path.basename(f)[:-13]] = rec['confirmationTimeSeconds']
    return runs


def main():
    arms = [spec.split('=', 1) for spec in sys.argv[1:]]
    data = [(label, load(out)) for label, out in arms]
    pairs = [(data[0], d) for d in data[1:]] + [(data[i], data[i + 1]) for i in range(1, len(data) - 1)]
    for (a, ra), (b, rb) in pairs:
        for kind, exact in (('exact', True), ('noisy', False)):
            keys = [k for k in ra if k in rb and k.startswith('seed0-') == exact]
            if not keys: continue
            d = [rb[k] - ra[k] for k in keys]
            print(f'{b} - {a} {kind}: {len(keys)} recovered by both, median {st.median(d):+.2f} s, mean {st.mean(d):+.2f} s; '
                  f'later {sum(x > 1e-9 for x in d)}, earlier {sum(x < -1e-9 for x in d)}, same {sum(abs(x) <= 1e-9 for x in d)}')


if __name__ == '__main__':
    main()
