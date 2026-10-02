"""Render the recorded flow-guided seed, not an invented geometric streamline."""
from pathlib import Path
import json,sys,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
root=Path('/home/zai/Downloads/ResearchProjects/collisionAvoidance')
sys.path.insert(0,str(root/'scripts'))
from auditJointPredictiveSafety import rectangle,distance
dataDir=Path(sys.argv[1]) if len(sys.argv)>1 else Path(__file__).parent
p=Path(sys.argv[2]) if len(sys.argv)>2 else Path('/home/zai/.cache/collisionAvoidance/flow-visualization-20261002')
p.mkdir(parents=True,exist_ok=True)
d=json.loads((dataDir/'flow-frame.json').read_text());x=np.array(d['states']).T;q=np.array(d['targetStates']).T;t=np.array(d['times']);sh=d['egoShape'];g=[]
for a,b in zip(x,q):
    e=np.array(rectangle(*a[:3],sh));o=np.array(rectangle(*b[:3],b[7:11]))
    g.append(max(max((o@n).min()-(e@n).max(),(e@n).min()-(o@n).max()) for h in [a[2],a[2]+np.pi/2,b[2],b[2]+np.pi/2] for n in [np.array([np.cos(h),np.sin(h)])]))
g=np.array(g);i=int(np.argmin(g));d['signedGaps']=g.tolist();d['worstIndex']=i
assert abs(g[i]+.2901614120216358)<1e-9 and t[i]==1.65
assert np.allclose([distance(rectangle(*a[:3],sh),rectangle(*b[:3],b[7:11])) for a,b in zip(x,q)],d['nodeClearance'],atol=1e-10)
summary=dict(frameTime=d['frameTime'],minimumSampledSignedGap=float(g[i]),minimumTime=float(t[i]),minimumIndex=i,initialEgoState=x[0].tolist(),initialTargetState=q[0].tolist(),collisionEgoState=x[i].tolist(),collisionTargetState=q[i].tolist(),sampledCollisionTimes=t[g<0].tolist(),horizonEnd=float(t[-1]),rows=len(t))
(p/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
plt.rcParams.update({'font.size':11,'axes.spines.top':False,'axes.spines.right':False})
fig=plt.figure(figsize=(13,9),layout='constrained');gs=fig.add_gridspec(2,2,height_ratios=[1.25,1]);ax=fig.add_subplot(gs[0,:]);blue='#2577B5';red='#D75944';teal='#159B8C'
def car(ax,pose,shape,col,label=None,alpha=.45):
    ax.add_patch(Polygon(rectangle(*pose[:3],shape),facecolor=col,edgecolor=col,alpha=alpha,label=label));ax.arrow(pose[0],pose[1],1.3*np.cos(pose[2]),1.3*np.sin(pose[2]),width=.035,color=col,length_includes_head=True)
ax.axhline(0,color='gray',ls=':',label='Given path');ax.plot(x[:17,0],x[:17,1],color=teal,lw=2.8,label='Flow-guided ego seed');ax.plot(x[16:,0],x[16:,1],color=teal,lw=2.8,ls='--',label='Completion part of seed');ax.plot(q[:,0],q[:,1],color=red,lw=2,label='Known target trajectory')
car(ax,x[0],sh,blue,'Current ego: t=0.85 s',.6);car(ax,q[0],q[0,7:11],red,'Current target: t=0.85 s',.6)
for j in [9,16]:
    car(ax,x[j],sh,blue,alpha=.18);car(ax,q[j],q[j,7:11],red,alpha=.18)
    ax.text(x[j,0]-1,x[j,1]+1.6,f'{t[j]:.2f} s',color=blue);ax.text(q[j,0]+1.8,q[j,1]-.6,f'{t[j]:.2f} s',color=red)
ax.scatter(x[i,0],x[i,1],s=85,marker='x',color='black',zorder=6);ax.annotate('Overlap in the seed at 1.65 s',xy=x[i,:2],xytext=(28,4),arrowprops={'arrowstyle':'->','color':'black'},weight='bold')
ax.set(xlim=(8,40),ylim=(-6,11),aspect='equal',xlabel='x [m]',ylabel='y [m]',title='Actual flow-guided initialization generated at t = 0.85 s');ax.legend(loc='upper left',fontsize=9,ncol=2)
ax=fig.add_subplot(gs[1,0]);car(ax,x[i],sh,blue,'Ego seed at 1.65 s',.6);car(ax,q[i],q[i,7:11],red,'Target at 1.65 s',.6);ax.set(xlim=(17.5,26),ylim=(-3.6,4.2),aspect='equal',xlabel='x [m]',ylabel='y [m]',title=f'Same-time close-up: signed gap {g[i]:.3f} m');ax.legend(loc='upper left',fontsize=9)
ax=fig.add_subplot(gs[1,1]);ax.plot(t,g,color=teal,lw=2);ax.axhline(0,color='black',lw=1,label='Touching');ax.axhline(.1,color='gray',ls=':',label='0.10 m optimization buffer');ax.axvline(1.65,color='#8c65b5',ls='--',label='Hard tail begins');ax.scatter([t[i]],[g[i]],color=red,s=50,zorder=5);ax.fill_between(t,g,0,where=g<0,color=red,alpha=.3);ax.set(xlim=(1.35,1.95),ylim=(-.5,3),xlabel='Absolute predicted time [s]',ylabel='Signed rectangle SAT gap [m]',title='The seed itself passes through the conflict');ax.legend(loc='upper left',fontsize=9)
fig.savefig(p/'flow-seed.png',dpi=170);fig.savefig(p/'flow-seed.pdf')
html='''<!doctype html><html lang="en"><meta charset="utf-8"><title>Flow seed · collision diagnosis</title><style>
*{box-sizing:border-box}body{margin:0;background:#f0f3f7;color:#203047;font:15px system-ui}main{max-width:1150px;margin:24px auto;padding:0 20px}h1{font-size:26px;margin:0 0 8px}.sub{color:#5b697b;margin-bottom:18px}.panel{background:white;border-radius:16px;padding:20px;box-shadow:0 4px 24px #20304709}canvas{width:100%;height:510px}button{border:0;border-radius:8px;background:#203047;color:white;padding:10px 18px;cursor:pointer}input{flex:1}.controls{display:flex;gap:14px;align-items:center;margin-top:14px}.metrics{display:flex;gap:25px;margin-top:18px;font-size:17px}.bad{color:#c74736;font-weight:700}.legend{display:flex;gap:20px;font-size:13px;margin:12px 0}small{color:#697386;display:block;margin-top:14px;line-height:1.5}</style><main>
<h1>Flow-guided initialization at t = 0.85 s</h1><div class="sub">Recorded vehicle rollout and known moving target. Scrub time to compare both vehicles at the same instant.</div><div class="panel"><div class="legend"><span style="color:#2577b5">■ Ego on the seed</span><span style="color:#d75944">■ Moving target</span><span style="color:#159b8c">━ Flow-guided seed / dashed completion</span><span>··· Given path</span></div><canvas id="map"></canvas><div class="controls"><button id="play">Play</button><input id="time" type="range" min="0" max="76" value="0"><button id="hit">Show overlap</button><button id="view">Full horizon</button></div><div class="metrics"><span id="when"></span><span id="gap"></span><span id="part"></span></div><small>Outline vehicles mark the current state at 0.85 s. Filled vehicles follow the stored initialization, not the later optimized or executed trajectory. Each step is 0.05 s. No road boundary is imposed. A negative SAT gap means body overlap; a positive SAT gap is not generally Euclidean distance.</small></div></main><script>
const d=__DATA__, X=d.states[0].map((_,i)=>d.states.map(r=>r[i])),Q=d.targetStates[0].map((_,i)=>d.targetStates.map(r=>r[i])),c=document.getElementById('map'),ctx=c.getContext('2d'),slider=document.getElementById('time');slider.max=X.length-1;let full=false,timer=null;
function draw(){const W=c.clientWidth,H=c.clientHeight;c.width=W*2;c.height=H*2;ctx.scale(2,2);let lo=full?[7,-35]:[8,-6],hi=full?[68,15]:[40,11];const s=Math.min((W-70)/(hi[0]-lo[0]),(H-55)/(hi[1]-lo[1])),ox=45+(W-70-s*(hi[0]-lo[0]))/2,oy=25+(H-55-s*(hi[1]-lo[1]))/2;function pt(x,y){return[ox+(x-lo[0])*s,oy+(hi[1]-y)*s]}function line(a,b,color,width=1,dash=[]){ctx.strokeStyle=color;ctx.lineWidth=width;ctx.setLineDash(dash);ctx.beginPath();ctx.moveTo(...pt(...a));ctx.lineTo(...pt(...b));ctx.stroke();ctx.setLineDash([])}
ctx.font='11px system-ui';ctx.fillStyle='#758295';for(let x=Math.ceil(lo[0]/5)*5;x<=hi[0];x+=5){line([x,lo[1]],[x,hi[1]],'#eef1f5');ctx.fillText(x,...pt(x,lo[1]-.7))}for(let y=Math.ceil(lo[1]/5)*5;y<=hi[1];y+=5){line([lo[0],y],[hi[0],y],'#eef1f5');const a=pt(lo[0],y);ctx.fillText(y,a[0]-24,a[1]+4)}line([lo[0],0],[hi[0],0],'#a6afb9',1,[3,5]);
ctx.save();ctx.beginPath();ctx.rect(ox,oy,(hi[0]-lo[0])*s,(hi[1]-lo[1])*s);ctx.clip();for(let j=1;j<X.length;j++){line(X[j-1].slice(0,2),X[j].slice(0,2),'#159b8c',2.5,j>d.prefixSteps?[6,4]:[]);line(Q[j-1].slice(0,2),Q[j].slice(0,2),'#d75944',2)}
function car(a,sh,col,outline=false){const co=Math.cos(a[2]),si=Math.sin(a[2]);let vertices=[[-1,-1],[1,-1],[1,1],[-1,1]].map(([u,v])=>{let x=sh[2]+u*sh[0],y=sh[3]+v*sh[1];return pt(a[0]+co*x-si*y,a[1]+si*x+co*y)});ctx.beginPath();vertices.forEach((p,i)=>i?ctx.lineTo(...p):ctx.moveTo(...p));ctx.closePath();ctx.strokeStyle=col;ctx.lineWidth=outline?1.3:2;ctx.setLineDash(outline?[4,3]:[]);if(!outline){ctx.globalAlpha=.35;ctx.fillStyle=col;ctx.fill();ctx.globalAlpha=1}ctx.stroke();ctx.setLineDash([]);if(!outline)line(a.slice(0,2),[a[0]+1.5*co,a[1]+1.5*si],col,3)}
car(X[0],d.egoShape,'#2577b5',true);car(Q[0],Q[0].slice(7,11),'#d75944',true);const k=+slider.value;car(X[k],d.egoShape,'#2577b5');car(Q[k],Q[k].slice(7,11),'#d75944');ctx.restore();document.getElementById('when').textContent='Time '+d.times[k].toFixed(2)+' s';const g=document.getElementById('gap');g.textContent='Signed gap '+d.signedGaps[k].toFixed(3)+' m';g.className=d.signedGaps[k]<0?'bad':'';document.getElementById('part').textContent=k<d.prefixSteps?'Primary horizon':'Hard completion tail';}
function stop(){clearInterval(timer);timer=null;document.getElementById('play').textContent='Play'}slider.oninput=()=>{stop();draw()};document.getElementById('play').onclick=()=>{if(timer){stop();return}if(+slider.value>=X.length-1)slider.value=0;document.getElementById('play').textContent='Pause';timer=setInterval(()=>{slider.value=+slider.value+1;draw();if(+slider.value>=X.length-1)stop()},150)};document.getElementById('hit').onclick=()=>{stop();slider.value=d.worstIndex;draw()};document.getElementById('view').onclick=()=>{full=!full;document.getElementById('view').textContent=full?'Conflict view':'Full horizon';draw()};window.onresize=draw;draw();</script></html>'''
(p/'flow-seed.html').write_text(html.replace('__DATA__',json.dumps(d)))
print(json.dumps(summary,indent=2))
