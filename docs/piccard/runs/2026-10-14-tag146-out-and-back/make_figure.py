import json,math,sys
import numpy as np, matplotlib
matplotlib.use('Agg'); import matplotlib.pyplot as plt
OUT=sys.argv[1]
d=json.load(open('docking.json')); tr=json.load(open('trial.json'))
poses=d['poses']; cf=d['controller_frame_in_ground_truth']; yw=cf['yaw_rad']; ox,oy=cf['origin_m'][0],cf['origin_m'][1]
TAG=np.array([4.02,0.325,3.505])
def c2w(p):
    x,y=p[0],p[1]; return np.array([ox+math.cos(yw)*x-math.sin(yw)*y, oy+math.sin(yw)*x+math.cos(yw)*y])
al=[];det=[]
with open('telemetry.jsonl') as f:
    for line in f:
        if '"kind":"alignment"' in line:
            r=json.loads(line); c=r['controller_cg']; g=r['ground_truth_cg']
            al.append((r['t'],*c2w(c['position_m']),c['rpy_rad'][2]+yw,g['position_m'][0],g['position_m'][1],g['rpy_rad'][2]))
        elif '"camera":"cam_front"' in line and '"kind":"detections"' in line:
            r=json.loads(line)
            for i,p in zip(r['ids'],r['positions_m']):
                if i=='tag36h11:146': det.append((r['t'],*p))
A=np.array(al); D=np.array(det); t,ex,ey,eyaw,gx,gy,gyaw=A.T
view=[p for p in poses if p['label']=='view'][0]; a,b=view['start_t']+20,view['end_t']
m=(t>=a)&(t<=b); dm=(D[:,0]>=a)&(D[:,0]<=b)
cam_off=np.linalg.norm(TAG[:2]-np.array([gx[m].mean(),gy[m].mean()]))-D[dm,3].mean()
def reproj(px,py,yaw,dx,dz):
    fwd=cam_off+dz; right=dx
    return np.array([px+fwd*math.cos(yaw)-right*math.sin(yaw), py+fwd*math.sin(yaw)+right*math.cos(yaw)])
def nearest(ts):
    i=np.searchsorted(t,ts); i=np.clip(i,1,len(t)-1); return np.where(abs(t[i-1]-ts)<abs(t[i]-ts),i-1,i)
j=nearest(D[:,0]); ok=abs(t[j]-D[:,0])<0.6
RE=np.array([reproj(ex[k],ey[k],eyaw[k],D[n,1],D[n,3]) for n,k in zip(np.where(ok)[0],j[ok])])
RG=np.array([reproj(gx[k],gy[k],gyaw[k],D[n,1],D[n,3]) for n,k in zip(np.where(ok)[0],j[ok])])
def spread(R): med=np.median(R,axis=0); dd=np.linalg.norm(R-med,axis=1); return med,dd.std(),dd.max(),np.linalg.norm(med-TAG[:2])
mE,sE,xE,offE=spread(RE); mG,sG,xG,offG=spread(RG); err=np.hypot(ex-gx,ey-gy)
gap=(D[D[:,0]<300][-1,0], D[D[:,0]>300][0,0])
S='#fcfcfb'; INK='#0b0b0b'; INK2='#52514e'; MUTED='#898781'; GRID='#e1e0d9'; AXIS='#c3c2b7'; C1='#2a78d6'; C2='#eb6834'; C3='#1baf7a'
plt.rcParams.update({'font.family':'sans-serif','font.size':10,'axes.edgecolor':AXIS,'axes.labelcolor':INK2,'xtick.color':MUTED,'ytick.color':MUTED,'text.color':INK})
fig=plt.figure(figsize=(12.5,10),facecolor=S); gs=fig.add_gridspec(2,2,height_ratios=[3.2,1.1],hspace=0.3,wspace=0.05,left=0.07,right=0.99,top=0.88,bottom=0.08)
axL=fig.add_subplot(gs[0,0]); axR=fig.add_subplot(gs[0,1],sharex=axL,sharey=axL); axT=fig.add_subplot(gs[1,:])
for ax,title,R,med,sg,mx,off,col,X,Y in ((axL,'EKF odometry',RE,mE,sE,xE,offE,C1,ex,ey),(axR,'Ground truth',RG,mG,sG,xG,offG,C3,gx,gy)):
    ax.set_facecolor(S); ax.set_aspect('equal'); ax.grid(True,color=GRID,lw=0.8); ax.set_axisbelow(True)
    ax.add_patch(plt.Rectangle((-3.05,-4.57),6.1,9.14,fill=False,ec=AXIS,lw=1,ls=':'))
    ax.text(-2.95,4.3,'tank',fontsize=8,color=MUTED)
    if ax is axL: ax.plot(gy,gx,color=MUTED,lw=1.6,ls="--",label="ground truth",zorder=2)
    ax.plot(Y,X,color=col,lw=2,label=title,zorder=3)
    k=nearest(D[:,0])[::6]; ax.scatter(Y[k],X[k],s=9,color=col,edgecolor=S,lw=0.4,zorder=4,label='pose at a tag-146 sighting')
    ax.scatter(R[::4,1],R[::4,0],s=10,color=C2,alpha=0.7,edgecolor='none',zorder=5,label='tag 146 reprojected from that pose')
    ax.scatter([TAG[1]],[TAG[0]],marker='*',s=180,color=INK,zorder=7,label='true tag 146')
    ax.scatter([med[1]],[med[0]],marker='x',s=70,color=C2,lw=2,zorder=7,label='median reprojection')
    lo=np.min(R,axis=0); hi=np.max(R,axis=0); ax.add_patch(plt.Rectangle((lo[1],lo[0]),hi[1]-lo[1],hi[0]-lo[0],fill=False,ec=C2,ls='--',lw=1,zorder=6))
    ax.set_title(f'{title}\nreprojection spread σ {sg*100:.1f} cm, max {mx*100:.1f} cm\nmedian {off*100:.1f} cm from the true tag',fontsize=10,color=INK,loc='left',pad=8)
    ax.set_xlabel('East (m)')
    ax.annotate('dive',(gy[0],gx[0]),textcoords='offset points',xytext=(8,-4),fontsize=9,color=INK2)
    ax.annotate('stand-off',(gy[m].mean(),gx[m].mean()),textcoords='offset points',xytext=(10,-3),fontsize=9,color=INK2)
    ax.annotate('turn, out, turn back',(gy[(t>330)&(t<340)].mean(),gx[(t>330)&(t<340)].mean()),textcoords='offset points',xytext=(10,0),fontsize=9,color=INK2)
axL.set_ylabel('North (m)'); axL.set_xlim(-3.4,3.4); axL.set_ylim(-5,5.1)
axL.legend(loc='upper left',bbox_to_anchor=(0.0,0.86),fontsize=8.5,frameon=False); axR.legend(loc='upper left',bbox_to_anchor=(0.0,0.86),fontsize=8.5,frameon=False); axR.tick_params(labelleft=False)
axT.set_facecolor(S); axT.grid(True,color=GRID,lw=0.8); axT.set_axisbelow(True)
axT.axvspan(gap[0],gap[1],color=GRID,alpha=0.8,lw=0,label='tag 146 not detected')
axT.plot(t,err*100,color=C1,lw=2,label='EKF horizontal error vs ground truth')
axT.vlines(D[::3,0],-1.2,1.2,color=C2,lw=0.6,alpha=0.6,label='tag-146 sighting (front camera)')
top=max(err*100)*1.45
for n,p in enumerate(poses):
    axT.axvline(p['start_t'],color=AXIS,lw=0.8); axT.text(p['start_t']+2,top*0.93-(n%2)*top*0.08,p['label'],fontsize=8,color=INK2,va='top')
axT.set_ylim(-1.5,top); axT.set_xlim(t[0],t[-1]); axT.set_xlabel('collector time (s)'); axT.set_ylabel('cm'); axT.legend(loc='upper right',fontsize=8.5,frameon=False,ncol=3,bbox_to_anchor=(1.0,1.18))
fig.suptitle(f'RACE tag-146 out-and-back, Stonefish, seed {tr["simulator_seed"]["value"]}: bird\'s-eye tracks\nand tag 146 reprojected at every sighting (no graph solution yet)',fontsize=12,color=INK,x=0.07,ha='left',y=0.985)
fig.text(0.07,0.01,f'Synthetic simulation scored by ground truth. {len(D)} front-camera sightings of tag 146 at 7 Hz; tag not detected {gap[0]:.0f}–{gap[1]:.0f} s ({gap[1]-gap[0]:.0f} s). \nCamera offset ahead of cg fitted at the stand-off: {cam_off:.2f} m. Job 4ef8719e, 2026-10-14.',fontsize=8,color=INK2)
fig.savefig(f'{OUT}/race-tag146-out-and-back-bev.png',dpi=160,facecolor=S,bbox_inches="tight",pad_inches=0.25); print('saved')
