import json,sys,glob,os,math
h=0.05
def predict(q,t):
    # predictiveSafetyGeometry.predictTarget, position only
    arc=q[3]*t+.5*q[4]*t*t; k=math.sin(q[5])/q[6]; a=k*arc/2
    sc=1.0 if a==0 else math.sin(a)/a
    ang=q[2]+q[5]+a
    return (q[0]+arc*sc*math.cos(ang), q[1]+arc*sc*math.sin(ang))
def pct(v,p):
    if not v: return float('nan')
    v=sorted(v); return v[min(len(v)-1,int(p*(len(v)-1)+.5))]
D=sys.argv[1]
agg={}
allInnov=[];allForecast=[];allForecastTime=[];reasons={};exitAhead=[]
runs=[]
for path in sorted(glob.glob(os.path.join(D,'*.json'))):
    try: r=json.load(open(path))['results']
    except Exception as e: continue
    if isinstance(r,list): r=r[0]
    t=r['trace']; name=os.path.basename(path)[:-5]
    shifted=[e for e in t if e['initialization']=='shiftedInputRollout']
    cert=sum(1 for e in shifted if e.get('anchorCertified')); cand=sum(1 for e in shifted if e.get('candidateFeasible'))
    for e in shifted:
        if not e.get('candidateFeasible'):
            k=e.get('candidateReason') or '?'; reasons[k]=reasons.get(k,0)+1
    restarts=sum(1 for e in t if e.get('potentialFieldRestarted'))
    zero=sum(1 for e in t if e.get('acceptedStep')==0)
    innov=[e['trust'].get('observationInnovation') for e in t if isinstance(e.get('trust'),dict)]
    innov=[v for v in innov if isinstance(v,(int,float)) and not math.isnan(v)]
    allInnov+=innov
    fc=[]
    for a,b in zip(t[:-1],t[1:]):
        qa=a.get('estimatedTarget'); qb=b.get('estimatedTarget')
        if not qa or not qb or not isinstance(qa,list) or not isinstance(qb,list): continue
        if len(qa)!=11 or len(qb)!=11: continue
        E=a['endpointIndex']; ia=a['absoluteSampleIndex']; ib=b['absoluteSampleIndex']
        ta=(E-ia)*h; tb=(E-ib)*h
        if tb<0: continue
        pa=predict(qa,ta); pb=predict(qb,tb)
        d=math.hypot(pa[0]-pb[0],pa[1]-pb[1]); fc.append(d); allForecast.append(d); allForecastTime.append(ta)
    for e in t:
        x=e.get('terminalExitSeconds')
        if isinstance(x,(int,float)) and not math.isnan(x): exitAhead.append(e['horizonSteps']*h+x)
    runs.append((name,len(t),r.get('outcome'),r.get('collisionDetected'),r.get('minimumReplayClearanceMeters'),len(shifted),cert,cand,restarts,zero,pct(innov,.5),pct(innov,.95),max(innov) if innov else float('nan'),pct(fc,.5),pct(fc,.95),max(fc) if fc else float('nan'),(r.get('failure') or '')[:90]))
print('%-40s %5s %-16s %4s %6s %5s %5s %5s %4s %4s %7s %7s %7s %7s %7s %7s %s'%('run','holds','outcome','col','gap','shift','cert','cand','rst','a0','in50','in95','inMax','fc50','fc95','fcMax','failure'))
for x in runs:
    print('%-40s %5d %-16s %4s %6.3f %5d %5d %5d %4d %4d %7.4f %7.4f %7.4f %7.3f %7.3f %7.3f %s'%(x[0],x[1],x[2],int(bool(x[3])),x[4] if x[4] is not None else float('nan'),x[5],x[6],x[7],x[8],x[9],x[10],x[11],x[12],x[13],x[14],x[15],x[16]))
n=len(runs)
print('\nRUNS',n,'recovered',sum(1 for x in runs if x[2]=='recovered'),'collisions',sum(1 for x in runs if x[3]),'outcomes',{o:sum(1 for x in runs if x[2]==o) for o in set(x[2] for x in runs)})
S=sum(x[5] for x in runs); C=sum(x[6] for x in runs); F=sum(x[7] for x in runs)
print('shifted frames',S,'certified',C,'candidate feasible',F,'restarts',sum(x[8] for x in runs),'alpha0',sum(x[9] for x in runs))
print('candidate failure reasons',dict(sorted(reasons.items(),key=lambda kv:-kv[1])))
print('observation innovation [m, pose metric] median %.4f p95 %.4f max %.4f (n=%d)'%(pct(allInnov,.5),pct(allInnov,.95),max(allInnov) if allInnov else float('nan'),len(allInnov)))
print('forecast change at endpoint [m] median %.3f p95 %.3f max %.3f (n=%d)'%(pct(allForecast,.5),pct(allForecast,.95),max(allForecast) if allForecast else float('nan'),len(allForecast)))
bins=[(0,1),(1,2),(2,4),(4,8),(8,30)]
for lo,hi in bins:
    v=[d for d,tt in zip(allForecast,allForecastTime) if lo<=tt<hi]
    print('  endpoint %4.1f-%4.1f s ahead: n=%5d median %.3f p95 %.3f max %.3f'%(lo,hi,len(v),pct(v,.5),pct(v,.95),max(v) if v else float('nan')))
print('time from now to target exit [s] median %.2f p95 %.2f max %.2f'%(pct(exitAhead,.5),pct(exitAhead,.95),max(exitAhead) if exitAhead else float('nan')))

# Ego estimation error in CLF units: eta = ||F J (xhat - x)||, F = chol(P)
import numpy as np
clf=json.load(open('/home/zai/Downloads/ResearchProjects/collisionAvoidance/config/clfMatrices.json'))
def factor(speed,curv):
    for e in clf:
        if e['referenceSpeed']==speed and abs(e['curvature']-curv)<1e-9 and '"dragCoefficient":0.3' in e['key']:
            return np.linalg.cholesky(np.array(e['matrix'])).T
etaBy={}
for path in sorted(glob.glob(os.path.join(D,'*.json'))):
    try: r=json.load(open(path))['results']
    except Exception: continue
    if isinstance(r,list): r=r[0]
    name=os.path.basename(path)
    speed=8 if name.startswith('speed8') else 15
    curved='curved' in name
    F=factor(speed,.005 if curved else 0)
    vals=[]
    for e in r['trace']:
        xh=np.array(e['estimatedState'],float).ravel(); x=np.array(e['state'],float).ravel()
        if xh.size!=6 or x.size!=6: continue
        d=xh-x
        if curved:
            c=np.array([0.0,200.0]); rel=x[:2]-c; n=-rel/np.linalg.norm(rel)
        else:
            n=np.array([0.0,1.0])
        dpsi=math.atan2(math.sin(d[2]),math.cos(d[2]))
        de=np.array([n@d[:2],dpsi,d[3],d[4],d[5]])
        vals.append(float(np.linalg.norm(F@de)))
    key=('%d'%speed)+('-curve' if curved else '-straight')
    etaBy.setdefault(key,[]).extend(vals)
for k,v in sorted(etaBy.items()):
    print('eta %-14s n=%6d median %.4f p95 %.4f p99 %.4f max %.4f'%(k,len(v),pct(v,.5),pct(v,.95),pct(v,.99),max(v) if v else float('nan')))
