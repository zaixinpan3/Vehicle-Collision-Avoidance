#!/usr/bin/env python3
"""Audit saved frame maxima and partition exact-input diagnostic replays.

Original timing buckets cannot be retrospectively split. Nested probe times
describe one representative replay, and never replace campaign measurements.
"""
import argparse
import collections
import hashlib
import json
from pathlib import Path
import subprocess

from analyzeMatlabControllerProfile import as_list, compact_function, stats


PARTITION = [
    'collisionAvoidanceController.localControllerConfiguration',
    'readPlanningInputs.readPlanningInputs',
    'collisionAvoidanceController.localFiniteModel',
    'hardEncounterBarrier.prepare',
    'formulateAvoidanceProblem.formulateAvoidanceProblem',
    'solveHardCbfClf.fluidInitialize',
    'avoidanceStageQp.fixedDirections',
    'solveHardCbfClf.localDefaultSolve',
    'solveHardCbfClf.certify',
    'collisionAvoidanceController.outputAssembly',
]


def original_frame(file, report, index):
    seconds = as_list(report['runtime']['frameSeconds'])[index]
    phase = report['runtimeBreakdown'][index]
    search = report['admissionSearch'][index]
    return {'source': str(file), 'frame': index + 1,
            'timeSeconds': index * report['configuration']['controller']['sampleTime'],
            'seconds': seconds, 'phase': phase,
            'otherSeconds': seconds - sum(phase.values()),
            'scenario': report['scenario'], 'curvature': report['roadCurvature'],
            'horizonSteps': report['horizonSteps'][index],
            'nativeSolves': search['nativeSolves'], 'policy': search['policy']}


def summarize_probes(item):
    samples = item['samples']
    representative = sorted(samples, key=lambda sample: sample['seconds'])[len(samples) // 2]
    events = representative['probes']['events']
    inclusive = collections.defaultdict(lambda: {'seconds': 0, 'calls': 0})
    tree = []
    for index, event in enumerate(events, 1):
        parent = event['parent']
        depth = 0
        while parent:
            ancestor = events[parent - 1]
            assert not (event['name'] in PARTITION and ancestor['name'] in PARTITION)
            parent = ancestor['parent']
            depth += 1
        children = sum(child['seconds'] for child in events if child['parent'] == index)
        exclusive = event['seconds'] - children
        assert exclusive >= -1e-12, event
        tree.append({'name': event['name'], 'depth': depth, 'parent': event['parent'],
                     'inclusiveMs': event['seconds'] * 1000, 'exclusiveMs': exclusive * 1000})
        inclusive[event['name']]['seconds'] += event['seconds']
        inclusive[event['name']]['calls'] += 1
    pieces = {name: inclusive[name]['seconds'] * 1000 for name in PARTITION}
    pieces['otherAndTimerOverhead'] = representative['seconds'] * 1000 - sum(pieces.values())
    assert pieces['otherAndTimerOverhead'] >= 0
    assert abs(sum(pieces.values()) - representative['seconds'] * 1000) < 1e-9
    native = representative['probes']['nativeCalls']
    assert len(native) == representative['search']['nativeSolves']
    assert all(sample['certified'] and not sample['failed'] for sample in samples)
    return {'name': item['definition']['name'], 'sampleIndex': samples.index(representative) + 1,
            'representativeMs': representative['seconds'] * 1000,
            'timing': stats([sample['seconds'] for sample in samples]),
            'partitionMs': pieces, 'callTree': tree, 'inclusiveEvents': dict(inclusive),
            'nativeCalls': native}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('campaign', type=Path)
    parser.add_argument('directory', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    raw = args.directory.resolve()
    campaign = args.campaign.resolve()
    groups = collections.defaultdict(list)
    maxima = {}
    sources = []
    for file in sorted(campaign.glob('diagnostic/*/*-exact-state.json')):
        report = json.loads(file.read_text())
        assert report['completed'] and report['executedHolds'] == report['sampleCount']
        sources.append(file)
        for index, seconds in enumerate(as_list(report['runtime']['frameSeconds'])):
            search = report['admissionSearch'][index]
            if 'initialCertificateAngles' not in search:
                kind = 'cruise'
            elif report['inheritedFeasibleFamily'][index]:
                kind = 'continuation'
            else:
                kind = 'admission'
            groups['all'].append(seconds)
            groups[kind].append(seconds)
            populations = ['measured']
            if kind == 'continuation':
                populations.append('continuation')
            if report['roadCurvature'] != 0 and report['scenario'] == 'crossing':
                populations.append('circularCrossing')
            for population in populations:
                if population not in maxima or seconds > maxima[population]['seconds']:
                    maxima[population] = original_frame(file, report, index)
    for file in sorted(campaign.glob('warmup-*/*-exact-state.json')):
        report = json.loads(file.read_text())
        sources.append(file)
        times = as_list(report['runtime']['frameSeconds'])
        index = max(range(len(times)), key=times.__getitem__)
        if 'warmup' not in maxima or times[index] > maxima['warmup']['seconds']:
            maxima['warmup'] = original_frame(file, report, index)
    assert groups['all'] and len(groups['all']) == sum(len(groups[key]) for key in ('cruise', 'continuation', 'admission'))
    replay = json.loads((raw / 'replay.json').read_text())
    probes = json.loads((raw / 'probes.json').read_text())
    profile = json.loads((raw / 'profile.json').read_text())
    assert [item['definition']['name'] for item in replay['fixtures']] == [
        item['definition']['name'] for item in probes['fixtures']] == [
        item['definition']['name'] for item in profile['fixtures']]
    result = {'controllerCommit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=root, text=True).strip(),
              'scope': 'Original campaign buckets, clean controller-only replay, and separate instrumented attribution',
              'rawDirectory': str(raw), 'campaignDirectory': str(campaign),
              'campaign': {'maxima': maxima, 'groups': {key: stats(value) for key, value in groups.items()}},
              'replays': [], 'probes': [], 'profiles': [], 'artifactsSha256': {}}
    for item in replay['fixtures']:
        assert item['prefixCommandDifference'] <= 1e-7
        samples = item['samples']
        assert all(sample['certified'] and not sample['failed'] for sample in samples)
        definition = item['definition']
        population = 'measured' if definition['name'] == 'measured-longest' else 'circularCrossing'
        assert definition['campaignSeconds'] == maxima[population]['seconds']
        assert definition['frame'] == maxima[population]['frame']
        result['replays'].append({'name': definition['name'], 'fixture': item['file'],
                                  'source': definition['source'], 'frame': definition['frame'],
                                  'timing': stats([sample['seconds'] for sample in samples]),
                                  'horizonSteps': samples[0]['horizonSteps'],
                                  'nativeSolves': samples[0]['search']['nativeSolves']})
    result['probes'] = [summarize_probes(item) for item in probes['fixtures']]
    for item in profile['fixtures']:
        functions = [value for value in item['functions'] if '/controller/' in value['file']
                     or value['name'] == 'solveAvoidanceSocpMex']
        result['profiles'].append({'name': item['definition']['name'],
                                   'instrumentedMs': item['sample']['seconds'] * 1000,
                                   'hotFunctions': [compact_function(value) for value in sorted(
                                       functions, key=lambda value: value['selfSeconds'], reverse=True)[:18]]})
    manifest = json.loads((raw / 'instrumentation-manifest.json').read_text())
    for item in manifest:
        for path, expected in [(root / 'controller' / item['file'], item['sourceSha256']),
                               (raw / 'instrumented' / item['file'], item['instrumentedSha256'])]:
            assert hashlib.sha256(path.read_bytes()).hexdigest() == expected, str(path)
    artifacts = sources + [raw / name for name in ('replay.json', 'probes.json', 'profile.json', 'instrumentation-manifest.json')]
    artifacts += list(raw.glob('*-fixture.mat')) + list(raw.glob('*-profile.mat'))
    artifacts += [root / 'controller' / item['file'] for item in manifest]
    artifacts += list((root / 'solver' / 'bicycle').glob('*.mexa64'))
    artifacts += [root / 'solver' / 'clarabel' / 'matlab' / 'solveAvoidanceSocpMex.mexa64',
                  root / 'config' / 'collisionAvoidanceControllerConfig.m', raw / 'code-analysis.json']
    artifacts += [root / 'scripts' / name for name in ('profileMatlabControllerRuntime.m',
                  'buildMatlabControllerProbes.py', 'analyzeLongestControllerFrames.py', 'analyzeMatlabControllerProfile.py')]
    for path in artifacts:
        result['artifactsSha256'][str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
    result['verification'] = {'fixtureCount': len(replay['fixtures']),
                              'cleanMeasuredCalls': sum(len(item['samples']) for item in replay['fixtures']),
                              'probeMeasuredCalls': sum(len(item['samples']) for item in probes['fixtures']),
                              'lineProfiledCalls': len(profile['fixtures']),
                              'completeReplayDecisionsUnchanged': True,
                              'productionSourcesMatchManifest': True,
                              'probePartitionsAreDisjointAndSumToRepresentativeTotal': True}
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')


if __name__ == '__main__':
    main()
