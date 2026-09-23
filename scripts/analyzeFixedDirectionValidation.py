#!/usr/bin/env python3
"""Summarize fixed-direction solves without discarding warmed timing outliers."""
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
    parser.add_argument('--tests', default='full-regression.json', help='Saved test summary relative to directory')
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
        'scope': 'Fixed directions followed by complete hard-constrained trajectory optimization; warmup excluded',
        'analysisParentCommit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=root, text=True).strip(),
        'matlabVersion': replay['matlabVersion'], 'replay': [], 'campaign': {},
        'excludedReplayWarmupCalls': 20,
    }
    groups = collections.defaultdict(list)
    for entry in replay['entries']:
        groups[entry['fixture']].append(entry)
    for fixture, entries in groups.items():
        samples = [sample for entry in entries for sample in entry['samples']]
        assert len(samples) == 22
        assert all(x['certified'] and x['directionsUnchanged'] and not x.get('initializerIssued', x.get('scalarWitnessIssued', True)) for x in samples)
        assert all(x['solverCalls'] == 1 and x['maximumJointResidual'] <= 0
                   and x['maximumPhysicalResidual'] <= 0 for x in samples)
        assert all(x['fixedAnglesMaximumError'] == 0 and x['controlLineDepartureNorm'] > 1e-4 for x in entries)
        worst = max(samples, key=lambda x: x['seconds'])
        partition = dict(worst['phase'])
        partition['remainingSeconds'] = worst['seconds'] - sum(partition.values())
        assert partition['remainingSeconds'] >= 0
        dimensions = {k: entries[0][k] for k in ('variables', 'constraints', 'controlLineDepartureNorm',
                                               'fixedAnglesMaximumError', 'seedObjective', 'optimizedObjective')}
        result['replay'].append({
            'fixture': fixture, 'timing': stats([x['seconds'] for x in samples]), **dimensions,
            'phaseMediansMs': {k: statistics.median(x['phase'][k] for x in samples) * 1000 for k in worst['phase']},
            'maximumFramePartitionMs': {k: v * 1000 for k, v in partition.items()},
            'nativeSolveCounts': [1], 'horizonSteps': sorted(set(x['horizonSteps'] for x in samples)),
        })

    campaign = load('campaign.json')
    assert len(campaign['entries']) == 24
    for mode in ('diagnostic', 'strict'):
        times = collections.defaultdict(list)
        cases, worst, retained = [], None, 0
        for entry in campaign['entries']:
            if entry['mode'] != mode:
                continue
            path = Path(entry['artifact']).with_suffix('.json')
            report = read(path)
            artifacts.extend([path, path.with_suffix('.mat')])
            executed = report['executedHolds']
            assert report['sampleCount'] == 600
            assert report['completed'] == (executed == report['sampleCount'])
            frames = as_list(report['runtime']['frameSeconds'])
            assert len(frames) == executed + int(not report['completed'])
            assert sum(as_list(report['restorationSolverCallCount'])) == 0
            for index, seconds in enumerate(frames):
                kind = 'rejected'
                phase = None
                if index < executed:
                    search = report['admissionSearch'][index]
                    kind = 'cruise' if 'initialCertificateAngles' not in search else (
                        'continuation' if as_list(report['inheritedFeasibleFamily'])[index] else 'admission')
                    phase = report['runtimeBreakdown'][index]
                if kind in ('admission', 'continuation'):
                    assert search['policy'] == 'fixedDirectionTrajectoryOptimization'
                    assert search['nativeSolves'] == 1 and not search['issuedAdmissionWitness']
                    if kind == 'admission':
                        assert search['usedFullPlanAdmission'] and search['fullPlanStatus'] == 'certified'
                    else:
                        assert search['initialCertificateAngles'] == search['fixedCertificateAngles']
                    retained += search['usedCertifiedIncumbent']
                times['all'].append(seconds)
                times[kind].append(seconds)
                if worst is None or seconds > worst['seconds']:
                    worst = {'source': str(path), 'frame': index + 1, 'seconds': seconds,
                             'kind': kind, 'phase': phase}
            cases.append({**entry, 'minimumNodeBodyGap': report['minimumNodeBodyGap'],
                          'sampledCollisionFree': report['sampledCollisionFree'],
                          'passed': report['passed'], 'runtimeQualified': report['runtimeQualified'],
                          'minimumSampledModelDomainMargin': report['minimumSampledModelDomainMargin'],
                          'maximumNodeHeadingErrorRadians': max(abs(x) for x in as_list(report['state'][2])),
                          'headingDiagnosticLimitRadians': report['configuration']['model']['headingDomainRadius'],
                          'stateAndSlipBoundsEnforced': report['stateAndSlipBoundsEnforced'],
                          'seed': report['seed'], 'frameTiming': stats(frames)})
        assert len(cases) == 12
        assert sum(len(times[k]) for k in ('cruise', 'continuation', 'admission', 'rejected')) == len(times['all'])
        result['campaign'][mode] = {
            'trials': cases, 'timing': {k: stats(v) for k, v in times.items() if v},
            'maximumFrame': worst, 'retainedOptimizedIncumbents': retained,
            'rejectedAttempts': len(times['rejected']),
            'completedTrials': sum(x['completed'] for x in cases),
            'passedTrials': sum(x['passed'] for x in cases),
            'runtimeQualifiedTrials': sum(x['runtimeQualified'] for x in cases),
            'executedHolds': sum(x['executedHolds'] for x in cases),
        }
    warmups = sorted(raw.glob('warmup/*/*-exact-state.json'))
    assert len(warmups) == 12
    result['excludedCampaignWarmupHolds'] = sum(read(path)['executedHolds'] for path in warmups)
    artifacts.extend(warmups)

    sensitivity = load('sensitivity.json')['entries']
    assert len(sensitivity) == 30
    samples = [entry['sample'] for entry in sensitivity]
    accepted = [x for x in samples if x['certified']]
    assert all(x['directionsUnchanged'] and not x.get('initializerIssued', x.get('scalarWitnessIssued', True)) for x in accepted)
    assert all(x['solverCalls'] == 1 for x in accepted if x['hasTarget'])
    result['sensitivity'] = {'fixtures': 30, 'certified': sum(x['certified'] for x in samples),
                             'acceptedActiveTargets': sum(x['hasTarget'] for x in accepted),
                             'allOutcomes': sensitivity,
                             'scope': 'Existing heuristic-development fixtures, not held-out validation.'}
    audits = load('independent-audit.json')
    assert len(audits) == 6 and all(x['endpointMaximumError'] < 1e-9 for x in audits)
    omitted = {'times', 'bodyGaps', 'satGaps', 'nominalTimes', 'nominalSatGaps', 'refinedHolds'}
    result['independentAudit'] = [{k: v for k, v in x.items() if k not in omitted} for x in audits]
    if (raw / 'generated/verification.json').is_file():
        result['generatedMexVerification'] = load('generated/verification.json')
        assert result['generatedMexVerification']['originalVerifierPassed']
        assert result['generatedMexVerification']['admissionStatus'] == 4
    else:
        result['generatedMexVerification'] = None
    result['tests'] = load(args.tests)
    assert result['tests']['failed'] == 0 and result['tests']['incomplete'] == 0
    result['codeAnalysis'] = load('code-analysis.json')
    boundary_probe = raw / 'road-boundary-probe/stationary-exact-state.json'
    if boundary_probe.is_file():
        probe = load('road-boundary-probe/stationary-exact-state.json')
        artifacts.append(boundary_probe.with_suffix('.mat'))
        result['roadBoundaryProbe'] = {key: probe[key] for key in (
            'roadBoundariesEnabled', 'completed', 'executedHolds', 'failureIdentifier', 'failureMessage')}
    # Hash only independent technical artifacts, never weekly archive reports.
    artifacts.extend(path for extension in ('*.m', '*.mat', '*.log') for path in raw.glob(extension)
                     if not path.name.startswith('weekly-'))
    if (raw / 'generated/standaloneControllerFrameMex.mexa64').is_file():
        artifacts.append(raw / 'generated/standaloneControllerFrameMex.mexa64')
    artifacts.extend(root.glob('controller/*.m'))
    artifacts.extend(root / name for name in ('config/collisionAvoidanceControllerConfig.m',
                     'scripts/runFixedDirectionValidation.m', 'scripts/analyzeFixedDirectionValidation.py',
                     'scripts/auditFixedDirectionCampaign.m',
                     'scripts/runExactStateRecursiveFeasibilityScenario.m', 'scripts/standaloneControllerFrame.m',
                     'scripts/standaloneControllerBenchmark.m', 'tests/fluidInitializationTest.m',
                     'tests/fullPlanAdmissionTest.m', 'tests/standaloneControllerFrameTest.m',
                     'tests/jointSupportCertificateTest.m', 'tests/admissionSafetyTest.m',
                     'tests/certificateContinuationTest.m', 'tests/collisionAvoidanceControllerTest.m'))
    artifacts.extend(root / name for name in ('tests/nrmmVffmReferenceTest.m',
                     'controller/NRMM_VFFM_INITIALIZATION.md'))
    if args.output.with_suffix('.md').is_file():
        artifacts.append(args.output.with_suffix('.md'))
    result['artifactsSha256'] = {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                               for path in sorted(set(artifacts))}
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')


if __name__ == '__main__':
    main()
