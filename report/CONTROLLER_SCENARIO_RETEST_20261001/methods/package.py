"""Package a completed retest, preserving raw originals in the experiment cache."""
from pathlib import Path
import json,hashlib,shutil,sys
raw=Path(sys.argv[1]);root=Path(sys.argv[2]);bundle=root/'report/CONTROLLER_SCENARIO_RETEST_20261001'
bundle.mkdir(exist_ok=True);(bundle/'methods').mkdir(exist_ok=True)
s=json.loads((raw/'summary.json').read_text());assert s['caseCount']==14
verify=json.loads((raw/'replay-verification.json').read_text());assert len(verify)==14
log=(raw/'campaign.log').read_text();assert 'CAMPAIGN-COMPLETE' in log
for name in ['summary.json','scenario-results.csv','frame-results.csv','configurations.json',
             'source-manifest.json','environment.json','replay-verification.json','warmup.json','process-status.json']:
    shutil.copy2(raw/name,bundle/name)
for name in ['campaign.m','analyze.py','verifyReplay.py','buildReport.py','package.py']:
    shutil.copy2(raw/name,bundle/'methods'/name)
# Keep the complete textual campaign log as an export, avoiding ignored *.log.
(bundle/'campaign-output.txt').write_text('\n'.join(line.rstrip() for line in log.splitlines())+'\n')
(bundle/'REPRODUCTION.txt').write_text('''Fresh 14-scenario retest, October 1, 2026

Tested commit: 9de49299a7c4f7d6b05e6a0dea48453fb7877a5a
No controller, configuration, scenario, or unit-test source was changed.
Raw experiment directory:
  /home/zai/.cache/collisionAvoidance/recovery-clf-validation-20261001
The source/ directory contains frozen copies of the files in source-manifest.json.
The three native binaries were existing local dependencies, not newly built or
added to version control. Their exact hashes are recorded.

The exact executed methods are exported under methods/. To repeat elsewhere,
create a fresh raw directory and source/ tree from the tested commit, supply the
recorded native dependencies, and adjust source/output at the top of campaign.m.
MATLAB R2026a with Optimization Toolbox is required.

Executed from the repository root (RAW means the raw directory above):
  matlab -singleCompThread -batch "run('RAW/campaign.m')" > RAW/campaign.log 2>&1
  python3 RAW/analyze.py RAW
  python3 RAW/verifyReplay.py RAW
  python3 RAW/buildReport.py RAW REPOSITORY_ROOT
  python3 RAW/package.py RAW REPOSITORY_ROOT
The shell tool returned exit status 143 after CAMPAIGN-COMPLETE and all 14
JSON/MAT pairs and warmup.json were saved. The cause was not established;
process-status.json preserves the observation. Artifact completeness and results
were independently checked; no zero exit status is claimed.

The campaign contains two preparation calls per speed, then seven sequential
scenarios at 8 and 15 m/s. It saves one original JSON and MAT per case. Each
simulation runs at most 1600 50-ms holds, stopping after confirmed five-second
nominal recovery. There is no stochastic input or random seed. The current
5-cm optimization-node buffer, no-road constraint configuration, and exact
observations are retained. Configuration records are in configurations.json.

The machine had another project computation running. Full controller-call
wall times include all model preparation and solves but exclude offline plant
integration, geometry and serialization. Timing is not injected as actuation
delay. First scenario frames are included; explicit preparation calls are
separate in warmup.json. Percentiles use linear interpolation at (n-1)*p.

Independent Python rectangle distances recompute every recorded ODE sample;
they must match MATLAB minima within 1e-8 m. Recovery dwell is independently
recomputed from all recorded transverse errors. These checks are offline only.
The legacy auditor's solver-admission function is not used because it does not
represent the current third-stage/no-admission contract. No prior unit-test
results are presented as freshly executed in this task.

The controller currently selects between laneQuadratic and recoveryCostToGo
based on the encounter range. The user reiterated during this retest that only
one CLF is intended throughout. This discrepancy is explicitly recorded in the
report; the current results are not a validation of a unified-CLF algorithm.

Data exports are derived copies; original traces remain in the raw directory.
campaign-output.txt trims trailing whitespace from the original campaign.log,
whose original bytes are separately hashed in the manifest.
The manifest hashes original traces and the exports. Raw large traces, native
binaries and generated PDFs are deliberately excluded from the project commit.
''')
def entry(p):return dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest(),bytes=p.stat().st_size)
source=json.loads((raw/'source-manifest.json').read_text());verified=[]
for rel,digest in source['files'].items():
    frozen=raw/'source'/rel
    assert hashlib.sha256(frozen.read_bytes()).hexdigest()==digest,rel
    current=hashlib.sha256((root/rel).read_bytes()).hexdigest()
    verified.append(dict(path=rel,sha256=digest,identicalToWorkingTree=current==digest))
assert all(r['identicalToWorkingTree'] for r in verified),'Source changed since experiment started.'
files=[p for p in bundle.rglob('*') if p.is_file() and p.name!='manifest.json']
files.append(root/'report/CONTROLLER_SCENARIO_RETEST_20261001.tex')
manifest=dict(kind='Fresh retest source and artifact hashes; no weekly or monthly report hashes',
    testedCommit=source['testedCommit'],rawDirectory=str(raw),source=verified,originalLog=entry(raw/'campaign.log'),
    exports=[dict(entry(p),path=str(p.relative_to(root))) for p in sorted(files)],
    originals=[entry(p) for p in sorted((raw/'campaign').glob('speed*/*')) if p.suffix in ('.json','.mat')])
assert len(manifest['originals'])==28
(bundle/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
print('Packaged',len(files),'exports and hashed 28 original traces.')
