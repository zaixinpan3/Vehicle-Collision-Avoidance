#!/usr/bin/env python3
"""Summarize unprofiled frames separately from instrumented attribution."""
import argparse
import collections
import hashlib
import json
from pathlib import Path
import statistics
import subprocess


def as_list(value):
    return value if isinstance(value, list) else [value]


def stats(values):
    return {'calls': len(values), 'medianMs': statistics.median(values) * 1000,
            'maximumMs': max(values) * 1000, 'over50Ms': sum(value > .05 for value in values)}


def compact_function(value):
    result = {key: item for key, item in value.items() if key != 'executedLines'}
    lines = value['executedLines']
    if lines and not isinstance(lines[0], list):
        lines = [lines]
    result['hottestLines'] = sorted(lines, key=lambda row: row[2], reverse=True)[:5]
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    raw = args.directory.resolve()
    groups = collections.defaultdict(list)
    cases = collections.defaultdict(lambda: collections.defaultdict(list))
    maxima = {}
    outcomes = []
    for file in sorted(raw.glob('measured-*/*/*-exact-state.json')):
        report = json.loads(file.read_text())
        case = file.parent.name
        for index, seconds in enumerate(as_list(report['runtime']['frameSeconds'])):
            if index >= report['executedHolds']:
                assert report['failureTime'] == 0, 'Classify any new failure mode explicitly.'
                kind = 'admission'
            elif 'initialCertificateAngles' not in report['admissionSearch'][index]:
                kind = 'cruise'
            elif report['inheritedFeasibleFamily'][index]:
                kind = 'continuation'
            else:
                kind = 'admission'
            groups['all'].append(seconds)
            groups[kind].append(seconds)
            cases[case][kind].append(seconds)
            if kind not in maxima or seconds > maxima[kind]['seconds']:
                maxima[kind] = {'source': str(file), 'frame': index + 1, 'seconds': seconds,
                                'phase': report['runtimeBreakdown'][index] if index < report['executedHolds'] else None}
        outcomes.append({key: report[key] for key in ('scenario', 'roadCurvature', 'completed',
                        'executedHolds', 'minimumSampledBodyGap', 'sampledCollisionFree', 'failureMessage')})
    assert len(groups['all']) == sum(len(groups[key]) for key in ('cruise', 'continuation', 'admission'))
    result = {'scope': 'Original MATLAB controller with existing solver MEX; controller-only experiments',
              'analysisCommit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=root, text=True).strip(),
              'campaign': {'groups': {key: stats(value) for key, value in groups.items()},
                           'cases': {key: {kind: stats(times) for kind, times in value.items()}
                                     for key, value in cases.items()}, 'maxima': maxima, 'outcomes': outcomes},
              'replays': [], 'coarse': [], 'profiles': [], 'artifactsSha256': {}}
    replay = json.loads((raw / 'replay.json').read_text())
    for item in replay['fixtures']:
        assert item['prefixCommandDifference'] <= 1e-7
        entry = {'definition': item['definition'], 'timing': stats([x['seconds'] for x in item['samples']]),
                 'prefixCommandDifference': item['prefixCommandDifference']}
        first = item['samples'][0]
        entry['failed'] = first['failed']
        if not first['failed']:
            assert all(x['certified'] for x in item['samples'])
            entry['horizonSteps'] = first['horizonSteps']
            entry['solverCalls'] = first['search']['nativeSolves']
        result['replays'].append(entry)
    coarse = json.loads((raw / 'probes.json').read_text())
    # These regions are disjoint in the current measured call graph. Parent
    # timers such as localFixedDirectionSearch are retained separately, never summed.
    partition = ['formulateAvoidanceProblem.formulateAvoidanceProblem',
                 'avoidanceStageQp.fixedDirections', 'solveHardCbfClf.localDefaultSolve',
                 'solveHardCbfClf.fluidInitialize', 'solveHardCbfClf.certify']
    for item in coarse['fixtures']:
        selected = sorted(item['samples'], key=lambda x: x['seconds'])[len(item['samples']) // 2]
        values = collections.defaultdict(lambda: {'seconds': 0, 'calls': 0})
        for event in selected['probes']['events']:
            value = values[event['name']]
            value['seconds'] += event['seconds']
            value['calls'] += 1
        pieces = {name: values[name]['seconds'] for name in partition}
        pieces['remaining'] = selected['seconds'] - sum(pieces.values())
        assert pieces['remaining'] >= 0
        # Verify that selected regions do not contain another selected region.
        events = selected['probes']['events']
        for event in events:
            parent = event['parent']
            while parent:
                ancestor = events[parent - 1]
                assert not (event['name'] in partition and ancestor['name'] in partition)
                parent = ancestor['parent']
        natives = selected['probes']['nativeCalls']
        if not selected['failed']:
            assert len(natives) == selected['search']['nativeSolves']
        result['coarse'].append({'definition': item['definition'], 'representativeSeconds': selected['seconds'],
                                'timing': stats([x['seconds'] for x in item['samples']]),
                                'partitionMs': {name: value * 1000 for name, value in pieces.items()},
                                'inclusiveEvents': dict(values), 'nativeCalls': natives})
    profile = json.loads((raw / 'profile.json').read_text())
    for item in profile['fixtures']:
        functions = item['functions']
        selected = [compact_function(x) for x in sorted(functions, key=lambda x: x['selfSeconds'], reverse=True)[:18]]
        support = [compact_function(x) for x in functions if x['name'].startswith('avoidanceStageQp>localFixedDirectionConic')]
        result['profiles'].append({'definition': item['definition'], 'seconds': item['sample']['seconds'],
                                   'largestSelfTimes': selected, 'jointConstructionFunctions': support})
    instrumented = json.loads((raw / 'instrumentation-manifest.json').read_text())
    for entry in instrumented:
        path = root / 'controller' / entry['file']
        assert hashlib.sha256(path.read_bytes()).hexdigest() == entry['sourceSha256'], str(path)
    artifacts = [raw / name for name in ('campaign.json', 'replay.json', 'profile.json', 'probes.json',
                                         'instrumentation-manifest.json')]
    if (raw / 'padding-audit.json').exists():
        result['paddingAudit'] = json.loads((raw / 'padding-audit.json').read_text())
        assert all(item['decisionEqual'] for item in result['paddingAudit'])
        artifacts.append(raw / 'padding-audit.json')
        artifacts.extend((raw / 'padding-audit').glob('*.m'))
    artifacts.extend(raw.glob('*-fixture.mat'))
    artifacts.extend(raw.glob('*-profile.mat'))
    artifacts.extend(raw.glob('measured-*/*/*-exact-state.mat'))
    artifacts.extend(root / 'controller' / entry['file'] for entry in instrumented)
    artifacts.extend(root / 'scripts' / name for name in ('profileMatlabControllerRuntime.m',
                     'buildMatlabControllerProbes.py', 'analyzeMatlabControllerProfile.py'))
    for path in artifacts:
        with path.open('rb') as stream:
            result['artifactsSha256'][str(path)] = hashlib.file_digest(stream, 'sha256').hexdigest()
    result['verification'] = {'fixtureCount': len(replay['fixtures']),
                              'cleanCalls': sum(len(x['samples']) for x in replay['fixtures']),
                              'coarseCalls': sum(len(x['samples']) for x in coarse['fixtures']),
                              'profiledCalls': len(profile['fixtures']),
                              'completeDecisionsUnchanged': True,
                              'productionSourcesUnchanged': True}
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')


if __name__ == '__main__':
    main()
