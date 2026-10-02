"""Offline collision diagnosis; no controller or acceptance-rule changes."""
from pathlib import Path
import json, sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
root=Path('/home/zai/Downloads/ResearchProjects/collisionAvoidance')
sys.path.insert(0,str(root/'scripts'))
from auditJointPredictiveSafety import target_state, rectangle, distance
out=Path(__file__).parent
source=Path('/home/zai/.cache/collisionAvoidance/unified-clf-implementation-20261001/assessment/campaign/speed15/turningCrossing.json')
r=json.loads(source.read_text())['results'];q=r['targetInitialState'];v=r['configuration']['vehicle'];shape=[v['length']/2,v['width']/2,*v['rectangleOffset']]
def sat(x,t):
    target=target_state(q,t);a=np.array(rectangle(*x[:3],shape));b=np.array(rectangle(*target[:3],q[7:11]))
    return max(max((b@n).min()-(a@n).max(),(a@n).min()-(b@n).max()) for h in [x[2],x[2]+np.pi/2,target[2],target[2]+np.pi/2] for n in [np.array([np.cos(h),np.sin(h)])])
actual=[]
for f in r['trace'][:35]:
    for dt,x in zip(f['auditTimes'],f['auditStates']):
        t=f['time']+dt;actual.append([t,sat(x,t),*x])
actual=np.array(actual);bad=actual[actual[:,1]<0];worst=actual[np.argmin(actual[:,1])]
n=np.loadtxt(out/'nodes.csv',delimiter=',');summary=[]
for t in [.65,.7,.75,.8,.85,.9,1.0,1.3,1.55,1.6,1.65]:
    rows=n[np.isclose(n[:,0],t)];a=rows[np.isclose(rows[:,2],1.65)][0]
    summary.append(dict(frameTime=t,affineClearanceAt165=a[4],nonlinearClearanceAt165=a[5],positionErrorAt165=a[6],maximumAffineForceRatio=float(rows[:,7].max()),affineSignedGapAt165=sat(a[9:15],1.65),nonlinearSignedGapAt165=sat(a[15:21],1.65)))
frames=json.loads((out/'frames.json').read_text());assert max(f['inputDifference'] for f in frames)==0
assert abs(worst[1]+.2237679944797044)<1e-10
selected=[r['trace'][i] for i in [16,17,18,32,33]]
(out/'selected-original-frames.json').write_text(json.dumps(selected,indent=2)+'\n')
np.savetxt(out/'collision-replay.csv',actual,delimiter=',',header='time,signedSatGap,x,y,yaw,vx,vy,yawRate',comments='')
(out/'summary.json').write_text(json.dumps(dict(source=str(source),replayedFrames=len(frames),maximumCommandDifference=max(f['inputDifference'] for f in frames),firstObservedCollision=bad[0,0],lastObservedCollision=bad[-1,0],worstTime=worst[0],worstSignedSatGap=worst[1],predictionComparisons=summary),indent=2)+'\n')
plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False})
fig,axes=plt.subplots(1,3,figsize=(15,4.6),layout='constrained')
ax=axes[0];t=worst[0];x=worst[2:];qt=target_state(q,t)
for pose,sh,col,label in [(x,shape,'#2878b5','Ego'),(qt,q[7:11],'#df6547','Target')]:
    ax.add_patch(Polygon(rectangle(*pose[:3],sh),facecolor=col,edgecolor=col,alpha=.55,label=label));ax.arrow(pose[0],pose[1],np.cos(pose[2]),np.sin(pose[2]),color=col,width=.035,length_includes_head=True)
ax.plot(actual[:,2],actual[:,3],color='#2878b5',lw=1,alpha=.4)
ax.set(xlim=(17,26),ylim=(-4,5),aspect='equal',xlabel='x [m]',ylabel='y [m]',title=f'Deepest overlap: t = {t:.4f} s');ax.legend()
ax=axes[1];ax.plot(actual[:,0],actual[:,1],color='#2878b5');ax.axhline(0,color='black',lw=.8);ax.axhline(.1,color='gray',ls='--',label='Optimization buffer');ax.axvspan(bad[0,0],bad[-1,0],color='#df6547',alpha=.18);ax.set(xlim=(1.5,1.75),ylim=(-.3,1.4),xlabel='Time [s]',ylabel='Signed rectangle SAT gap [m]',title='Collision spans a sample boundary');ax.legend()
ax=axes[2];data=n[np.isclose(n[:,0],.85)];times=data[:,2]
for start,label,col in [(9,'Affine prediction','#2878b5'),(15,'Same inputs, nonlinear rollout','#df6547')]:
    gaps=[sat(row[start:start+6],row[2]) for row in data];ax.plot(times,gaps,marker='.',label=label,color=col)
ax.axhline(0,color='black',lw=.8);ax.axhline(.1,color='gray',ls='--');ax.set(xlim=(1.5,1.8),ylim=(-.8,1.6),xlabel='Predicted absolute time [s]',ylabel='Signed rectangle SAT gap [m]',title='Plan made at t = 0.85 s');ax.legend(loc='upper left',fontsize=8)
fig.savefig(out/'collision-analysis.png',dpi=180);fig.savefig(out/'collision-analysis.pdf')
print(json.dumps(summary,indent=2))
