"""Classify actual nominal-value increases at modeled zero CLF slack."""
import json,collections,sys
from pathlib import Path
raw=Path(sys.argv[1]) if len(sys.argv)>1 else Path(__file__).parent
counts=collections.Counter();worst=[];cases=[]
for path in (raw/'campaign').glob('speed*/*.json'):
    r=json.loads(path.read_text())['results'];trace=r['trace'];local=collections.Counter()
    for i,f in enumerate(trace[:-1]):
        tol=r['configuration']['solver']['feasibilityTolerance']*max(f['clfInitialValue'],1)
        delta=trace[i+1]['clfInitialValue']-f['clfInitialValue']
        if f['clfSlack']>tol or delta<=tol:continue
        assert i>0,'Classifying the first state requires its path error explicitly.'
        error=trace[i-1]['transverseError']
        outside=any(abs(x)>v for x,v in zip(error,r['recovery']['tolerances']))
        tag=('outside' if outside else 'inside')+('_converged' if f['optimizationConverged'] else '_nonconverged')
        counts[tag]+=1;local[tag]+=1
        worst.append(dict(speed=r['configuration']['referenceSpeed'],scenario=r['scenario'],time=f['time'],
            value=f['clfInitialValue'],delta=delta,required=f['clfRequiredDecrease'],outside=outside,converged=f['optimizationConverged']))
    cases.append(dict(speed=r['configuration']['referenceSpeed'],scenario=r['scenario'],counts=dict(local)))
worst.sort(key=lambda r:r['delta'],reverse=True)
summary=dict(counts=dict(counts),cases=cases,largestIncreases=worst[:10])
(raw/'descent-diagnostic.json').write_text(json.dumps(summary,indent=2)+'\n')
print(counts)
