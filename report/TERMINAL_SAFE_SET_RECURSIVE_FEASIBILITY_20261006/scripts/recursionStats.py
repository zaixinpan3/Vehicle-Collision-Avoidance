import json,sys,glob,os,math,statistics as st
rows=[]
for path in sorted(glob.glob(os.path.join(sys.argv[1],'*.json'))):
    try: r=json.load(open(path))['results']
    except Exception as e: print('skip',path,e); continue
    if isinstance(r,list): r=r[0]
    t=r['trace']; n=len(t)
    name=os.path.basename(path)[:-5]
    def g(e,k,d=float('nan')):
        v=e.get(k); return d if v is None else v
    shifted=[e for e in t if e['initialization']=='shiftedInputRollout']
    cert=sum(1 for e in shifted if e.get('anchorCertified'))
    cand=sum(1 for e in shifted if e.get('candidateFeasible'))
    steps=[g(e,'acceptedStep') for e in t]
    zero=sum(1 for s in steps if s==0); full=sum(1 for s in steps if s==1)
    restarts=sum(1 for e in t if e.get('potentialFieldRestarted'))
    appended=sum(g(e,'appendedTerminalSteps',0) for e in t)
    H=[e['horizonSteps'] for e in t]
    sec=[e['controllerSeconds'] for e in t]
    sec.sort(); p95=sec[int(.95*(len(sec)-1))] if sec else float('nan')
    rows.append((name,n,r.get('outcome'),r.get('failure') or '',r.get('minimumReplayClearanceMeters'),len(shifted),cert,cand,full,zero,restarts,appended,max(H) if H else 0,p95,max(sec) if sec else 0))
print('%-34s %5s %-16s %7s %6s %6s %6s %5s %5s %5s %6s %5s %6s %6s %s'%('case','holds','outcome','gap','shift','cert','cand','a=1','a=0','rest','append','Hmax','p95','max','failure'))
for r in rows:
    print('%-34s %5d %-16s %7.3f %6d %6d %6d %5d %5d %5d %6d %5d %6.3f %6.3f %s'%(r[0],r[1],r[2],r[4] if r[4] is not None else float('nan'),r[5],r[6],r[7],r[8],r[9],r[10],r[11],r[12],r[13],r[14],r[3][:80]))
