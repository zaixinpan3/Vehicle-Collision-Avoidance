import json,glob,os,math,csv,sys,subprocess
B=os.path.expanduser('~/.cache/collisionAvoidance/terminal-20261006')
OUT='/home/zai/Downloads/ResearchProjects/collisionAvoidance/report/TERMINAL_SAFE_SET_RECURSIVE_FEASIBILITY_20261006'
os.makedirs(OUT,exist_ok=True)
fields=['campaign','case','seed','holds','outcome','collision','failure','minimumReplayClearanceMeters','shiftedFrames','anchorCertified','candidateFeasible','candidateCollisionReasons','potentialFieldRestarts','alphaFull','alphaZero','appendedTerminalSteps','maximumHorizonSteps','p95ControllerSeconds','maximumControllerSeconds']
rows=[]
for camp in ['exact','rk4','noisy']:
    for p in sorted(glob.glob(f'{B}/{camp}/*.json')):
        r=json.load(open(p))['results']; r=r[0] if isinstance(r,list) else r
        t=r['trace']; name=os.path.basename(p)[:-5]
        seed=''
        if '-seed' in name: name,seed=name.split('-seed')
        sh=[e for e in t if e['initialization']=='shiftedInputRollout']
        sec=sorted(e['controllerSeconds'] for e in t)
        rows.append(dict(campaign=camp,case=name,seed=seed,holds=len(t),outcome=r.get('outcome'),collision=int(bool(r.get('collisionDetected'))),
            failure=(r.get('failure') or '').split(':')[-1].strip()[:120],
            minimumReplayClearanceMeters=r.get('minimumReplayClearanceMeters'),shiftedFrames=len(sh),
            anchorCertified=sum(1 for e in sh if e.get('anchorCertified')),candidateFeasible=sum(1 for e in sh if e.get('candidateFeasible')),
            candidateCollisionReasons=sum(1 for e in sh if not e.get('candidateFeasible') and e.get('candidateReason')=='collision'),
            potentialFieldRestarts=sum(1 for e in t if e.get('potentialFieldRestarted')),
            alphaFull=sum(1 for e in t if e.get('acceptedStep')==1),alphaZero=sum(1 for e in t if e.get('acceptedStep')==0),
            appendedTerminalSteps=sum(e.get('appendedTerminalSteps') or 0 for e in t),maximumHorizonSteps=max([e['horizonSteps'] for e in t] or [0]),
            p95ControllerSeconds=sec[int(.95*(len(sec)-1))] if sec else '',maximumControllerSeconds=sec[-1] if sec else ''))
with open(f'{OUT}/runs.csv','w',newline='') as f:
    w=csv.DictWriter(f,fieldnames=fields); w.writeheader(); w.writerows(rows)
print('runs.csv',len(rows))
