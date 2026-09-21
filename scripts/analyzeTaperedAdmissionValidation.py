#!/usr/bin/env python3
"""Summarize warmed admission ablations and independently audited campaigns."""
import argparse
import collections
import hashlib
import json
from pathlib import Path
import statistics
import subprocess

from analyzeMatlabControllerProfile import as_list, stats


def read(path):
    return json.loads(path.read_text())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    raw = args.directory.resolve()
    artifacts = []

    def load(name):
        path = raw / name
        artifacts.append(path)
        return read(path)

    replay = load('replay.json')
    result = {
        'scope': 'Warmed full MATLAB frames; sensitivity and footprint audits are untimed capability checks',
        'baselineCommit': 'c622028432b8989494dea0b1e010002a269ace2b',
        'analysisParentCommit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=root, text=True).strip(),
        'matlabVersion': replay['matlabVersion'],
        'replay': [], 'campaign': {}, 'sensitivity': {}, 'shapeSweep': [],
    }
    groups = collections.defaultdict(list)
    for entry in replay['entries']:
        groups[(entry['fixture'], entry['variant'])].extend(entry['samples'])
    for (fixture, variant), samples in groups.items():
        assert len(samples) == 22 and all(x['certified'] for x in samples)
        assert all(x['maximumJointResidual'] <= 0 and x['maximumPhysicalResidual'] <= 0 for x in samples)
        worst = max(samples, key=lambda x: x['seconds'])
        partition = dict(worst['phase'])
        partition['remainingSeconds'] = worst['seconds'] - sum(partition.values())
        assert partition['remainingSeconds'] >= 0
        result['replay'].append({
            'fixture': fixture, 'variant': variant, 'timing': stats([x['seconds'] for x in samples]),
            'phaseMediansMs': {k: statistics.median(x['phase'][k] for x in samples) * 1000 for k in worst['phase']},
            'maximumFramePartitionMs': {k: v * 1000 for k, v in partition.items()},
            'nativeSolveCounts': sorted(set(x['solverCalls'] for x in samples)),
            'horizonSteps': sorted(set(x['horizonSteps'] for x in samples)),
        })

    campaign = load('campaign.json')
    assert len(campaign['entries']) == 24
    for mode in ('diagnostic', 'strict'):
        times = collections.defaultdict(list)
        cases = []
        worst = None
        for entry in campaign['entries']:
            if entry['mode'] != mode:
                continue
            path = Path(entry['artifact']).with_suffix('.json')
            report = read(path)
            artifacts.extend([path, path.with_suffix('.mat')])
            assert report['completed'] and report['executedHolds'] == report['sampleCount'] == 600
            assert all(report['hardCertificateVerified'])
            frames = as_list(report['runtime']['frameSeconds'])
            assert len(frames) == 600 and sum(report['restorationSolverCallCount']) == 0
            for index, seconds in enumerate(frames):
                search = report['admissionSearch'][index]
                kind = 'cruise' if 'initialCertificateAngles' not in search else (
                    'continuation' if report['inheritedFeasibleFamily'][index] else 'admission')
                times['all'].append(seconds)
                times[kind].append(seconds)
                if worst is None or seconds > worst['seconds']:
                    worst = {'source': str(path), 'frame': index + 1, 'seconds': seconds,
                             'kind': kind, 'phase': report['runtimeBreakdown'][index]}
            cases.append({**entry, 'minimumNodeBodyGap': report['minimumNodeBodyGap'],
                          'sampledCollisionFree': report['sampledCollisionFree'],
                          'minimumSampledModelDomainMargin': report['minimumSampledModelDomainMargin'],
                          'stateAndSlipBoundsEnforced': report['stateAndSlipBoundsEnforced'],
                          'seed': report['seed'], 'frameTiming': stats(frames)})
        assert len(cases) == 12 and len(times['all']) == 7200
        assert sum(len(times[k]) for k in ('cruise', 'continuation', 'admission')) == len(times['all'])
        result['campaign'][mode] = {'trials': cases, 'timing': {k: stats(v) for k, v in times.items()},
                                    'maximumFrame': worst}
    warmups = list(raw.glob('warmup/*/*-exact-state.json'))
    assert len(warmups) == 12
    result['excludedWarmupHolds'] = sum(read(path)['executedHolds'] for path in warmups)
    artifacts.extend(warmups)

    sensitivity = load('sensitivity.json')['entries']
    assert len(sensitivity) == 60
    for variant in ('plateauFine', 'taperCoarse'):
        samples = [entry['sample'] for entry in sensitivity if entry['variant'] == variant]
        assert len(samples) == 30 and all(x['certified'] for x in samples)
        result['sensitivity'][variant] = {
            'fixtures': len(samples), 'certified': sum(x['certified'] for x in samples),
            'activeTargets': sum(x['hasTarget'] for x in samples),
            'fullPlanAdmissions': sum(x['fullPlanAdmission'] for x in samples),
        }
    sweep = load('shape-sweep.json')
    assert len(sweep) == 144
    for shoulder in (1, .9, .8, .7):
        selected = [x for x in sweep if x['shoulder'] == shoulder and x['normals'] == 16 and x['cells'] == 8]
        result['shapeSweep'].append({'shoulderFraction': shoulder, 'fixtures': len(selected),
                                     'scalarCertificates': sum(x['certified'] for x in selected)})
    audits = load('independent-audit.json')
    assert len(audits) == 6 and all(x['endpointMaximumError'] < 1e-9 for x in audits)
    omitted = {'times', 'bodyGaps', 'satGaps', 'nominalTimes', 'nominalSatGaps', 'refinedHolds'}
    result['independentAudit'] = [{k: v for k, v in x.items() if k not in omitted} for x in audits]
    result['generatedMexVerification'] = load('generated/verification.json')
    assert result['generatedMexVerification']['originalVerifierPassed']
    result['tests'] = load('test-results.json')
    assert result['tests']['failed'] == 0 and result['tests']['incomplete'] == 0
    result['codeAnalysis'] = load('code-analysis.json')
    # These are technical artifacts, never weekly or monthly archive documents.
    artifacts.extend(raw / name for name in ('probeAdmissionShapes.m', 'shapedAdmissionProbe.m',
                     'verifyGeneratedTaper.m', 'auditTaperedCampaign.m', 'all-tests-final.mat',
                     'finalizeTaperTests.m', 'full-suite-before-test-update.mat',
                     'generated-audit.log', 'all-tests-final.log', 'generated/standaloneControllerFrameMex.mexa64'))
    artifacts.extend(root.glob('controller/*.m'))
    artifacts.extend(root / name for name in ('config/collisionAvoidanceControllerConfig.m',
                     'scripts/runTaperedAdmissionValidation.m', 'scripts/analyzeTaperedAdmissionValidation.py',
                     'scripts/runExactStateRecursiveFeasibilityScenario.m', 'scripts/standaloneControllerBenchmark.m',
                     'tests/taperedAdmissionTest.m', 'tests/fullPlanAdmissionTest.m', 'tests/standaloneControllerFrameTest.m',
                     'tests/nodeCertificateTest.m'))
    result['artifactsSha256'] = {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in sorted(set(artifacts))}
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')


if __name__ == '__main__':
    main()
