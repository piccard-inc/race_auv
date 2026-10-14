import json,math,sys
import numpy as np, matplotlib
matplotlib.use('Agg'); import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
OUT=sys.argv[1]
d=json.load(open('docking.json')); tr=json.load(open('trial.json')); req=json.load(open('request/mission.json'))
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
err=np.hypot(ex-gx,ey-gy); rmse=float(np.sqrt(np.mean(err**2))); emax=float(err.max())
rE=np.linalg.norm(RE-TAG[:2],axis=1); rG=np.linalg.norm(RG-TAG[:2],axis=1); rmseE=float(np.sqrt(np.mean(rE**2))); rmseG=float(np.sqrt(np.mean(rG**2)))
gap=(D[D[:,0]<300][-1,0], D[D[:,0]>300][0,0])
S='#fcfcfb'; INK='#0b0b0b'; INK2='#52514e'; MUTED='#898781'; GRID='#e1e0d9'; AXIS='#c3c2b7'; C2='#eb6834'; C3='#1baf7a'
# ordinal blue ramp for the eight sections in time order (palette steps 250..700)
RAMP=["#9ec5f4","#6da7ec","#3987e5","#256abf","#184f95","#0d366b","#6250d6","#e34948"]
plt.rcParams.update({'font.family':'sans-serif','font.size':10,'axes.edgecolor':AXIS,'axes.labelcolor':INK2,'xtick.color':MUTED,'ytick.color':MUTED,'text.color':INK})
fig=plt.figure(figsize=(11.5,11.5),facecolor=S); gs=fig.add_gridspec(2,1,height_ratios=[3.5,1.1],hspace=0.22,left=0.08,right=0.98,top=0.9,bottom=0.07)
ax=fig.add_subplot(gs[0]); axT=fig.add_subplot(gs[1])
ax.set_facecolor(S); ax.set_aspect('equal'); ax.grid(True,color=GRID,lw=0.8); ax.set_axisbelow(True)
ax.add_patch(plt.Rectangle((-3.05,-4.57),6.1,9.14,fill=False,ec=AXIS,lw=1,ls=':')); ax.text(-2.95,4.3,'tank wall',fontsize=8,color=MUTED)
ax.plot(gy,gx,color=MUTED,lw=2.2,ls='--',zorder=2)
handles=[Line2D([],[],color=MUTED,lw=2.2,ls='--',label='ground truth (Stonefish)')]
for n,p in enumerate(poses):
    s=(t>=p['start_t'])&(t<=p['end_t']+0.6)
    ax.plot(ey[s],ex[s],color=RAMP[n],lw=2.4,solid_capstyle='round',zorder=3)
    mid=np.where(s)[0][len(np.where(s)[0])//2]; ax.annotate(str(n+1),(ey[mid],ex[mid]),textcoords='offset points',xytext=(-14 if n in (4,5) else 6,0),fontsize=8,color=RAMP[n],fontweight='bold',zorder=8)
    handles.append(Line2D([],[],color=RAMP[n],lw=2.4,label=f"{n+1}. {p['label']} ({p['start_t']:.0f}–{p['end_t']:.0f} s)"))
k=nearest(D[:,0])
ax.scatter(ey[k],ex[k],s=14,marker='|',color=C2,lw=1,zorder=4)
handles.append(Line2D([],[],marker='|',color=C2,lw=0,markersize=7,label="EKF position at each tag-146 sighting"))
ax.scatter(RE[::3,1],RE[::3,0],s=9,color=C2,alpha=0.6,edgecolor='none',zorder=5)
handles.append(Line2D([],[],marker='o',color=C2,lw=0,markersize=4,label=f"tag 146 from the EKF pose, each sighting (RMSE {rmseE*100:.1f} cm)"))
ax.scatter(RG[::3,1],RG[::3,0],s=9,color=C3,alpha=0.6,edgecolor='none',zorder=5)
handles.append(Line2D([],[],marker='o',color=C3,lw=0,markersize=4,label=f"tag 146 from the true pose, each sighting (RMSE {rmseG*100:.1f} cm)"))
ax.scatter([TAG[1]],[TAG[0]],marker='*',s=220,color=INK,zorder=7); handles.append(Line2D([],[],marker='*',color=INK,lw=0,markersize=12,label='true tag 146'))
wp={}
for p in req['poses']: wp.setdefault((p['x_m'],p['y_m']),[]).append(p['label'])
for (x,y),labs in wp.items():
    w=c2w((x,y)); ax.scatter([w[1]],[w[0]],marker='s',s=70,facecolor=S,edgecolor=INK,lw=1.4,zorder=6)
    ax.annotate('waypoint: '+', '.join(labs),(w[1],w[0]),textcoords='offset points',xytext=(12,-4),fontsize=9,color=INK2)
handles.append(Line2D([],[],marker='s',color=INK,markerfacecolor=S,lw=0,markersize=8,label="mission waypoint"))
# zoom inset at the tag
axi=ax.inset_axes([0.63,0.71,0.35,0.27]); axi.set_facecolor(S); axi.grid(True,color=GRID,lw=0.6); axi.set_axisbelow(True)
axi.scatter(RE[:,1],RE[:,0],s=6,color=C2,alpha=0.5,edgecolor='none'); axi.scatter(RG[:,1],RG[:,0],s=6,color=C3,alpha=0.5,edgecolor='none')
axi.scatter([TAG[1]],[TAG[0]],marker='*',s=160,color=INK,zorder=7); axi.set_aspect('equal'); axi.set_xlim(TAG[1]-0.35,TAG[1]+0.35); axi.set_ylim(TAG[0]-0.3,TAG[0]+0.3)
axi.tick_params(labelsize=7,colors=MUTED); axi.set_title('around the tag, 0.7 m × 0.6 m',fontsize=8,color=INK2)
ax.set_xlim(-3.4,3.4); ax.set_ylim(-5,5.1); ax.set_xlabel('East (m)'); ax.set_ylabel('North (m)')
ax.set_title(f'EKF odometry vs ground truth: horizontal ATE RMSE {rmse*100:.1f} cm, max {emax*100:.1f} cm (aligned once, at the first sample). EKF only; no factor-graph solution.',fontsize=10,color=INK,loc='left',pad=8)
ax.legend(handles=handles,loc="center left",fontsize=8,frameon=False,bbox_to_anchor=(0.0,0.42),title="EKF track by section",title_fontsize=8.5,alignment="left")
axT.set_facecolor(S); axT.grid(True,color=GRID,lw=0.8); axT.set_axisbelow(True)
axT.axvspan(gap[0],gap[1],color=GRID,alpha=0.8,lw=0,label='tag 146 not detected')
for n,p in enumerate(poses):
    s=(t>=p['start_t'])&(t<=p['end_t']+0.6); axT.plot(t[s],err[s]*100,color=RAMP[n],lw=2)
axT.plot([],[],color=RAMP[4],lw=2,label='EKF horizontal error vs ground truth (coloured by section)')
axT.vlines(D[::3,0],-1.2,1.2,color=C2,lw=0.6,alpha=0.6,label='tag-146 sighting (front camera)')
top=max(err*100)*1.45
for n,p in enumerate(poses):
    axT.axvline(p['start_t'],color=AXIS,lw=0.8); axT.text(p['start_t']+2,top*0.93-(n%2)*top*0.08,f"{n+1}. {p['label']}",fontsize=8,color=INK2,va='top')
axT.set_ylim(-1.5,top); axT.set_xlim(t[0],t[-1]); axT.set_xlabel('collector time (s)'); axT.set_ylabel('cm'); axT.legend(loc='upper right',fontsize=8.5,frameon=False,ncol=3,bbox_to_anchor=(1.0,1.18))
fig.suptitle(f'RACE tag-146 out-and-back, Stonefish, seed {tr["simulator_seed"]["value"]}, job 4ef8719e (2026-10-14)',fontsize=12,color=INK,x=0.08,ha='left',y=0.965)
fig.text(0.08,0.012,f'Frame: Stonefish world, x north, y east, z down (NED). The mission is a list of waypoints with hold times in the controller\'s world_ned frame (request.json); the vehicle moves between them on its position controller.\n{len(D)} front-camera sightings of tag 146 at 7 Hz; not detected {gap[0]:.0f}–{gap[1]:.0f} s. Each sighting is the detector\'s XYZ of the tag in the camera frame (no tag orientation), projected into the world through the EKF pose (orange) or the true pose (green). Camera 0.68 m ahead of cg.',fontsize=8,color=INK2)
fig.savefig(f'{OUT}/race-tag146-out-and-back-bev.png',dpi=160,facecolor=S,bbox_inches='tight',pad_inches=0.25); print('saved')
