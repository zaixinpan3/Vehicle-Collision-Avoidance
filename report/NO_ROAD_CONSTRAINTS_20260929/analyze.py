from pathlib import Path
import csv, hashlib, json, subprocess

root = Path('/home/zai/Downloads/ResearchProjects/collisionAvoidance')
raw = Path(__file__).parent
current = json.loads((raw / 'independent-audit.json').read_text())
previous = json.loads((root / 'report/COLLISION_THREAT_VALIDATION_20260929/independent-audit.json').read_text())
previous_cases = {(x['referenceSpeedMetersPerSecond'], x['scenario']): x for x in previous['results']}
rows = []
for entry in current['results']:
    path = Path(entry['source'])
    result = json.loads(path.read_text())['results']
    key = (entry['referenceSpeedMetersPerSecond'], entry['scenario'])
    before = previous_cases[key]
    running = result['trace'][1:]
    assert result['roadConstraintsEnforced'] is False
    assert entry['independentBaselineCollision']
    if entry['completed']:
        assert len(result['trace']) == 160
        assert entry['geometryAudit']['everyHoldFeasible']
        assert entry['geometryAudit']['allPredictionsZeroSlack']
        assert entry['geometryAudit']['strictlyCollisionFree']
    else:
        assert entry['executedFrames'] == 0
    row = dict(speedMetersPerSecond=key[0], scenario=key[1], completed=entry['completed'],
               collisionAvoided=entry['collisionAvoided'], executedFrames=entry['executedFrames'],
               minimumClearanceMeters=entry['minimumReplayClearanceMeters'],
               minimumRoadMarginMeters=result['minimumReplayRoadMarginMeters'],
               initializationSeconds=entry['initializationSeconds'],
               failedInitializationSeconds=entry['failedFrameSeconds'],
               maximumRunningSeconds=entry['maximumRunningSeconds'],
               runningDeadlineMisses=sum(t['controllerSeconds'] > .05 for t in running),
               runningConicCalls=sum(t['solverCalls'] for t in running),
               failure=entry['failure'],
               previousCompleted=before['completed'],
               previousMinimumClearanceMeters=before['minimumReplayClearanceMeters'],
               previousMaximumRunningSeconds=before['maximumRunningSeconds'])
    rows.append(row)
with (raw / 'comparison.csv').open('w', newline='') as handle:
    writer = csv.DictWriter(handle, fieldnames=list(rows[0]), lineterminator='\n')
    writer.writeheader(); writer.writerows(rows)
summary = dict(scope='Road boundaries removed; unchanged fourteen cruise-collision fixtures and controller settings',
               sourceBaseCommit='b87600c2786a3b2cde4d365c7e31cd6c3383b264',
               scenarios=len(rows), completed=sum(r['completed'] for r in rows),
               collisionAvoided=sum(r['collisionAvoided'] for r in rows),
               initializationFailures=sum(r['executedFrames'] == 0 for r in rows),
               maximumRunningSeconds=current['maximumCompletedRunningSeconds'],
               minimumCompletedClearanceMeters=current['minimumCompletedClearanceMeters'],
               maximumSuccessfulInitializationSeconds=max(r['initializationSeconds'] for r in rows if r['completed']),
               totalRunningDeadlineMisses=sum(r['runningDeadlineMisses'] for r in rows),
               totalRunningConicCalls=sum(r['runningConicCalls'] for r in rows),
               completedRoadDepartures=sum(r['minimumRoadMarginMeters'] < 0 for r in rows if r['completed']),
               rows=rows)
(raw / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
manifest = json.loads((raw / 'source-manifest.json').read_text())
for path, digest in manifest['files'].items():
    assert hashlib.sha256((root / path).read_bytes()).hexdigest() == digest, path
artifacts = [*raw.glob('*.json'), *raw.glob('speed*/*.json'), raw / 'runChecks.m', raw / 'analyze.py',
             raw / 'run.log', raw / 'audit.log', raw / 'comparison.csv']
(raw / 'raw-artifact-hashes.json').write_text(json.dumps({str(p): hashlib.sha256(p.read_bytes()).hexdigest()
    for p in sorted(artifacts) if p.name != 'raw-artifact-hashes.json'}, indent=2) + '\n')
print(json.dumps({k: v for k, v in summary.items() if k != 'rows'}, indent=2))
