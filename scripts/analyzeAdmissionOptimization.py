#!/usr/bin/env python3
"""Compare alternating admission timings and retain all closed-loop outcomes."""
import argparse
from collections import defaultdict
import hashlib
import json
from pathlib import Path
import statistics
import subprocess

from analyzeMatlabControllerProfile import stats


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('baseline_campaign', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    raw = args.directory.resolve()
    subprocess.run(['python', str(root / 'scripts/analyzeFixedDirectionValidation.py'),
                    str(raw), str(args.output), '--tests', 'full-tests.json'], check=True)
    result = json.loads(args.output.read_text())
    paired = json.loads((raw / 'paired-admission.json').read_text())
    groups = defaultdict(lambda: defaultdict(list))
    for block in paired['blocks']:
        assert block['completeDecisionEqual'] and block['finalSafetyProgramEqual'] and block['certified']
        assert block['nativeSolves'] == 1
        assert len(block['samples']) == paired['repetitions']
        groups[block['fixture']][block['variant']].append(block)
    comparisons = []
    for fixture, variants in groups.items():
        entry = {'fixture': fixture, 'timing': {}, 'phaseMediansMs': {}, 'rounds': []}
        for variant in ('baseline', 'optimized'):
            assert len(variants[variant]) == paired['rounds']
            samples = [sample for block in variants[variant] for sample in block['samples']]
            entry['timing'][variant] = stats([sample['seconds'] for sample in samples])
            entry['phaseMediansMs'][variant] = {
                key: 1000 * statistics.median(sample['phase'][key] for sample in samples)
                for key in samples[0]['phase']}
        for number in range(1, paired['rounds'] + 1):
            medians = {}
            for variant in ('baseline', 'optimized'):
                block = next(block for block in variants[variant] if block['round'] == number)
                medians[variant] = 1000 * statistics.median(sample['seconds'] for sample in block['samples'])
            entry['rounds'].append({'round': number, 'mediansMs': medians,
                                    'decreaseMs': medians['baseline'] - medians['optimized']})
        before, after = (entry['timing'][key]['medianMs'] for key in ('baseline', 'optimized'))
        entry['medianDecreaseMs'] = before - after
        entry['medianDecreasePercent'] = 100 * (before - after) / before
        comparisons.append(entry)
    result['pairedAdmission'] = {'comparisons': comparisons,
        'protocol': {key: value for key, value in paired.items() if key != 'blocks'},
        'measuredCalls': sum(len(block['samples']) for block in paired['blocks']),
        'excludedWarmupCalls': len(paired['blocks']) * paired['warmupsPerBlock'],
        'completeDecisionsAndFinalSafetyProgramsEqual': True}
    baseline = json.loads((raw / 'baseline/manifest.json').read_text())
    for name, expected in baseline['files'].items():
        assert hashlib.sha256((raw / 'baseline' / name).read_bytes()).hexdigest() == expected
        committed = subprocess.check_output(['git', 'show', baseline['commit'] + ':controller/' + name], cwd=root)
        assert hashlib.sha256(committed).hexdigest() == expected
    result['baselineRevision'] = baseline['commit']
    result['closedLoopEquivalence'] = []
    baseline_files = []
    for file in sorted(raw.glob('diagnostic/*/*-exact-state.json')):
        previous = args.baseline_campaign.resolve() / file.relative_to(raw)
        actual, expected = (json.loads(path.read_text()) for path in (file, previous))
        assert actual['completed'] and expected['completed']
        differences = {}
        for field in ('state', 'input'):
            a, b = actual[field], expected[field]
            assert len(a) == len(b) and all(len(x) == len(y) for x, y in zip(a, b))
            differences[field] = max(abs(x - y) for row_a, row_b in zip(a, b) for x, y in zip(row_a, row_b))
        result['closedLoopEquivalence'].append({'source': str(file), 'baseline': str(previous),
                                                'maximumAbsoluteDifferences': differences})
        baseline_files.append(previous)
    assert len(result['closedLoopEquivalence']) == 12
    artifacts = [raw / 'paired-admission.json', raw / 'baseline/manifest.json',
        *(raw / 'baseline' / name for name in baseline['files']), *baseline_files,
        root / 'scripts/benchmarkAdmissionOptimization.m', Path(__file__).resolve(),
        root / 'scripts/buildAdmissionProfilePrototypes.py',
        root / 'tests/admissionAssemblyOptimizationTest.m', root / 'controller/JOINT_SUPPORT_CERTIFICATES.md']
    for path in artifacts:
        result['artifactsSha256'][str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')


if __name__ == '__main__':
    main()
