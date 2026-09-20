#!/usr/bin/env python3
"""Summarize same-input native replays; never label these full-frame timings."""
import argparse
import hashlib
import json
from pathlib import Path
import statistics


def stats(values):
    values = sorted(values)
    return {'count': len(values), 'medianMs': statistics.median(values) * 1000,
            'maximumMs': max(values) * 1000,
            'over50Ms': sum(x > .05 for x in values)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('directory', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    root = args.directory
    baseline = json.loads((root/'capture/matlab-replay.json').read_text())
    frames = {x['file']: x for x in baseline['frames']}
    actual = [json.loads(line) for line in (root/'native-replay.jsonl').read_text().splitlines()]
    validation = json.loads((root/'capture/native-validation.json').read_text())
    assert not validation['hardCertificateFailures'] and not validation['acceptanceDifferences']
    assert len(actual) == len(frames) * baseline['repetitions']
    groups = [('all', list(frames))]
    groups += [(f'{scenario}-{curvature:g}', [key for key, value in frames.items()
                if value['scenario'] == scenario and value['curvature'] == curvature])
               for curvature in [0, .01] for scenario in ['stationary', 'oncoming', 'crossing']]
    groups += [('inherited', [key for key, value in frames.items() if value['inherited']]),
               ('fresh', [key for key, value in frames.items() if not value['inherited']])]
    report = {'scope': baseline['scope'], 'frameCount': len(frames), 'groups': {},
              'validation': {k:v for k,v in validation.items() if k != 'frames'}, 'artifacts': {}}
    for name, keys in groups:
        selected = [row for row in actual if row['file'] in keys]
        report['groups'][name] = {
            'matlab': stats([time for key in keys for time in frames[key]['seconds']]),
            'native': stats([row['seconds'] for row in selected]),
            'nativePhaseMediansMs': [statistics.median(row['metrics'][i] for row in selected)*1000 for i in range(3)],
            'statuses': {str(status):sum(row['status'] == status for row in selected) for status in [0,1,2,3]}}
    worst = max(actual,key=lambda x:x['seconds'])
    report['slowestNativeFrame'] = {**{k:v for k,v in worst.items() if k not in ('decision','angles')},
        'scenario': frames[worst['file']]['scenario'], 'curvature': frames[worst['file']]['curvature'],
        'frame': frames[worst['file']]['frame'], 'horizonSteps': frames[worst['file']]['horizonSteps']}
    hybrid = json.loads((root/'hybrid-metrics.json').read_text())
    report['hybrid'] = {'trials': hybrid['trials'], 'groups': {}}
    for name in ('allSeconds','activeSeconds','inheritedSeconds','freshSeconds'):
        report['hybrid']['groups'][name] = stats(hybrid[name])
    report['checks'] = {
        'unitTests': json.loads((root/'full-tests.json').read_text()),
        'checkedMex': json.loads((root/'mex-validation.json').read_text())}
    for path in ['native/controller-replay', 'native/standaloneControllerFrame.a', 'native-replay.jsonl',
                 'capture/matlab-replay.json','capture/native-validation.json','capture/capture.json',
                 'hybrid-metrics.json','full-tests.json','mex-validation.json']:
        report['artifacts'][path] = {'sha256':hashlib.sha256((root/path).read_bytes()).hexdigest(),
                                    'bytes':(root/path).stat().st_size}
    args.output.write_text(json.dumps(report,indent=2)+'\n')


if __name__ == '__main__':
    main()
