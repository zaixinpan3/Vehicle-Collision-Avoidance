#!/usr/bin/env python3
"""Write report/RTI_FAILURE_ROOT_CAUSES_20261001/manifest.json (source, bundle and raw-artifact hashes)."""
import hashlib, json, os, subprocess, glob
repo = '/home/zai/Downloads/ResearchProjects/collisionAvoidance'
W = '/home/zai/.cache/collisionAvoidance/rti-failure-analysis-20261001'
B = repo + '/report/RTI_FAILURE_ROOT_CAUSES_20261001'
commit = '5f7497718d4774a2515330650ee8815ccfa67b20'
def sha(path):
    return hashlib.sha256(open(path, 'rb').read()).hexdigest()
sources = []
for rel in ['controller/collisionAvoidanceController.m', 'controller/solvePredictiveControl.m', 'controller/terminalContinuation.m',
            'controller/nonlinearBicycleModel.m', 'controller/predictiveSafetyGeometry.m', 'controller/modifiedFialaTire.m',
            'controller/readControllerInputs.m', 'controller/laneGeometry.m', 'config/collisionAvoidanceControllerConfig.m',
            'scripts/runNonlinearPredictiveSafetyValidation.m', 'scripts/collisionThreatScenario.m', 'scripts/givenPathCollisionBaseline.m']:
    committed = hashlib.sha256(subprocess.check_output(['git', 'show', f'{commit}:{rel}'], cwd=repo)).hexdigest()
    snapshot = sha(f'{W}/source/{rel}')
    sources.append({'path': rel, 'sha256AtCommit': committed, 'snapshotIdentical': committed == snapshot})
natives = [{'path': os.path.relpath(p, W + '/source'), 'sha256': sha(p)} for p in sorted(glob.glob(W + '/source/solver/controller/*.mexa64'))]
bundle = [{'path': os.path.relpath(p, repo), 'sha256': sha(p)} for p in sorted(glob.glob(B + '/**/*', recursive=True))
          if os.path.isfile(p) and not p.endswith('manifest.json')]
raw = []
for pattern in ['campaign/speed*/*.json', 'campaign/speed*/*.mat', 'replay/*.mat', 'ablation/*.mat', 'cf/*.mat', 'cf/*/solvePredictiveControl.m',
                'methods/rtiInternals.m', 'source.sha256', 'working-tree.diff', 'campaign-*.log', 'bundle/*.csv']:
    for p in sorted(glob.glob(f'{W}/{pattern}')):
        raw.append({'path': p, 'sha256': sha(p), 'bytes': os.path.getsize(p)})
manifest = {'kind': 'Source, bundle and raw-artifact hashes; no weekly or monthly report hashes',
            'controllerCommit': commit, 'sources': sources, 'nativeKernels': natives, 'bundle': bundle, 'rawArtifacts': raw}
assert all(s['snapshotIdentical'] for s in sources)
json.dump(manifest, open(B + '/manifest.json', 'w'), indent=2)
open(B + '/manifest.json', 'a').write('\n')
print(len(sources), 'sources identical to commit;', len(bundle), 'bundle files;', len(raw), 'raw artifacts')
