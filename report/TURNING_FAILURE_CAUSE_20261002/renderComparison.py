"""Visualize fixed-normal conservatism using identical actual vehicle poses."""
from pathlib import Path
import json,sys,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
sys.path.insert(0,'/home/zai/Downloads/ResearchProjects/collisionAvoidance/scripts')
from auditJointPredictiveSafety import rectangle,distance
src=Path(sys.argv[1]) if len(sys.argv)>1 else Path(__file__).parent
out=Path(sys.argv[2]) if len(sys.argv)>2 else Path('/home/zai/.cache/collisionAvoidance/turning-root-cause-20261002');out.mkdir(exist_ok=True)
d=json.loads((src/'comparison.json').read_text());r=d['comparisons'][0]
a=np.array(rectangle(*r['actualPoseAt165'],d['shape']));b=np.array(rectangle(*d['targetPose'],d['targetShape']))
assert abs(distance(a,b)-r['actualGapAt165'])<1e-10
plt.rcParams.update({'font.size':11,'axes.spines.top':False,'axes.spines.right':False})
fig,axes=plt.subplots(1,3,figsize=(16,5.7),layout='constrained')
for ax,n,title,col in zip(axes[:2],[d['oldNormal'],r['actualBestNormalAt165']],['Frozen normal from the seed','Normal consistent with the turned vehicle'],['#d65b43','#159b8c']):
    n=np.array(n);support=(b@n).max();gap=(a@n).min()-support
    ax.add_patch(Polygon(a,facecolor='#2577b5',edgecolor='#2577b5',alpha=.4,label='Same ego pose'))
    ax.add_patch(Polygon(b,facecolor='#d75944',edgecolor='#d75944',alpha=.4,label='Same target pose'))
    xx=np.array([17,27]);yy=(support-n[0]*xx)/n[1];ax.plot(xx,yy,color=col,lw=2,label='Target supporting line')
    ax.plot(xx,(support+.1-n[0]*xx)/n[1],color=col,ls=':',lw=1,label='10 cm buffer line')
    k=np.argmin(a@n);v=a[k];foot=v-gap*n;ax.plot([v[0],foot[0]],[v[1],foot[1]],color=col,lw=3)
    ax.scatter(*v,color=col,s=55,zorder=5);ax.annotate(f'Projected gap\n{gap*100:+.1f} cm',xy=(v+foot)/2,xytext=(18.1,-2.5),arrowprops={'arrowstyle':'->','color':col},color=col,weight='bold')
    origin=np.array([24.4,3.2]);ax.arrow(*origin,*(1.4*n),width=.025,color=col,head_width=.16,length_includes_head=True);ax.text(22.4,4.3,f'Normal: {np.degrees(np.arctan2(n[1],n[0])):.2f} deg',color=col)
    ax.set(xlim=(17.5,26.6),ylim=(-3.5,5.2),aspect='equal',xlabel='x [m]',ylabel='y [m]',title=title);ax.legend(loc='lower right',fontsize=8)
ax=axes[2];ax.set_title('Best possible repair of the original row')
ax.axvspan(-.32,.1,color='#f8e8e4');ax.axvspan(.1,.16,color='#e4f4ef');ax.axvline(0,color='#777',ls=':');ax.axvline(.1,color='#159b8c',lw=2)
for y,val,label,col in [(2,-.290161412,'Seed corner','#d65b43'),(1,.051924475,'Best allowed correction','#2577b5')]:
    ax.plot([-.32,val],[y,y],color=col,lw=7,solid_capstyle='round');ax.scatter(val,y,s=80,color=col,zorder=5);ax.text(-.315,y+.2,f'{label}: {val*100:+.1f} cm',color=col,weight='bold')
ax.annotate('',xy=(.1,.6),xytext=(.051924475,.6),arrowprops={'arrowstyle':'<->','color':'#d65b43'});ax.text(-.02,.28,'Still 4.8 cm short',color='#d65b43',weight='bold');ax.text(.106,2.45,'Required\n+10 cm',color='#159b8c');ax.set(xlim=(-.33,.16),ylim=(0,2.9),yticks=[],xlabel='Projected corner separation [m]')
fig.suptitle('A fixed separating direction can reject a collision-free turn',fontsize=17)
fig.savefig(out/'normal-diagnosis.png',dpi=165);fig.savefig(out/'normal-diagnosis.pdf')
print('Independent same-pose rectangle comparison verified:',distance(a,b))
