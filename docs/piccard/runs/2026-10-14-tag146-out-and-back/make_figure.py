import json,math,sys
import numpy as np, matplotlib
matplotlib.use('Agg'); import matplotlib.pyplot as plt
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
rE=np.linalg.norm(RE-TAG[:2],axis=1); rG=np.linalg.norm(RG-TAG[:2],axis=1)
rmseE=float(np.sqrt(np.mean(rE**2))); rmseG=float(np.sqrt(np.mean(rG**2)))
gap=(D[D[:,0]<300][-1,0], D[D[:,0]>300][0,0])
# waypoints (controller frame -> world)
wp={}
for p in req['poses']: wp.setdefault((p['x_m'],p['y_m']),[]).append(p['label'])
nums={'ate_horizontal_rmse_m':rmse,'ate_horizontal_max_m':emax,'ate_horizontal_at_end_m':float(err[-1]),
      'tag146_reprojected_from_ekf_pose_rmse_from_true_tag_m':rmseE,'tag146_reprojected_from_true_pose_rmse_from_true_tag_m':rmseG,
      'sightings_146_front':int(len(D)),'tag_out_interval_s':[float(gap[0]),float(gap[1])],'camera_offset_fit_m':float(cam_off),
      'alignment':'EKF (controller world_ned frame) mapped to the Stonefish world by the transform recorded from the first cg_link sample (docking.json controller_frame_in_ground_truth); no least-squares alignment',
      'frame':'Stonefish world: x north, y east, z down (NED); figure axes East (y) and North (x)','seed':tr['simulator_seed']['value'],'mission_s':tr['mission_seconds']}
json.dump(nums,open(f'{OUT}/figure-numbers.json','w'),indent=1); print(json.dumps(nums,indent=1))
S='#fcfcfb'; INK='#0b0b0b'; INK2='#52514e'; MUTED='#898781'; GRID='#e1e0d9'; AXIS='#c3c2b7'; C1='#2a78d6'; C2='#eb6834'
plt.rcParams.update({'font.family':'sans-serif','font.size':10,'axes.edgecolor':AXIS,'axes.labelcolor':INK2,'xtick.color':MUTED,'ytick.color':MUTED,'text.color':INK})
fig=plt.figure(figsize=(11,11),facecolor=S); gs=fig.add_gridspec(2,1,height_ratios=[3.4,1.1],hspace=0.25,left=0.08,right=0.98,top=0.9,bottom=0.07)
ax=fig.add_subplot(gs[0]); axT=fig.add_subplot(gs[1])
ax.set_facecolor(S); ax.set_aspect('equal'); ax.grid(True,color=GRID,lw=0.8); ax.set_axisbelow(True)
ax.add_patch(plt.Rectangle((-3.05,-4.57),6.1,9.14,fill=False,ec=AXIS,lw=1,ls=':')); ax.text(-2.95,4.3,'tank wall',fontsize=8,color=MUTED)
ax.plot(gy,gx,color=MUTED,lw=2.2,ls='--',label='ground truth (Stonefish)',zorder=2)
ax.plot(ey,ex,color=C1,lw=2,label='EKF odometry (/race_auv/odometry/filtered)',zorder=3)
ax.scatter(RE[::3,1],RE[::3,0],s=9,color=C2,alpha=0.6,edgecolor='none',zorder=5,label=f'tag 146 as seen from the EKF pose, every sighting (RMSE {rmseE*100:.1f} cm from the true tag)')
ax.scatter([TAG[1]],[TAG[0]],marker='*',s=220,color=INK,zorder=7,label='true tag 146')
for (x,y),labs in wp.items():
    w=c2w((x,y)); ax.scatter([w[1]],[w[0]],marker='s',s=70,facecolor=S,edgecolor=INK,lw=1.4,zorder=6)
    ax.annotate('waypoint: '+', '.join(labs),(w[1],w[0]),textcoords='offset points',xytext=(12,-4),fontsize=9,color=INK2)
ax.annotate('',xy=(ey[(t>330)&(t<332)].mean()+0.05,ex[(t>330)&(t<332)].mean()),xytext=(1.6,-1.2),arrowprops=dict(arrowstyle='->',color=INK2,lw=1))
ax.text(1.65,-1.15,'EKF track on the way out and back\n(blue); truth is the grey dashed line',fontsize=9,color=INK2,va='center')
ax.set_xlim(-3.4,3.4); ax.set_ylim(-5,5.1); ax.set_xlabel('East (m)'); ax.set_ylabel('North (m)')
ax.set_title(f'EKF odometry vs ground truth: horizontal ATE RMSE {rmse*100:.1f} cm, max {emax*100:.1f} cm (aligned once, at the first sample)\nEKF only; no factor-graph solution in this figure',fontsize=10.5,color=INK,loc='left',pad=8)
ax.legend(loc='upper left',bbox_to_anchor=(0.0,0.9),fontsize=8.5,frameon=False)
axT.set_facecolor(S); axT.grid(True,color=GRID,lw=0.8); axT.set_axisbelow(True)
axT.axvspan(gap[0],gap[1],color=GRID,alpha=0.8,lw=0,label='tag 146 not detected')
axT.plot(t,err*100,color=C1,lw=2,label='EKF horizontal error vs ground truth')
axT.vlines(D[::3,0],-1.2,1.2,color=C2,lw=0.6,alpha=0.6,label='tag-146 sighting (front camera)')
top=max(err*100)*1.45
for n,p in enumerate(poses):
    axT.axvline(p['start_t'],color=AXIS,lw=0.8); axT.text(p['start_t']+2,top*0.93-(n%2)*top*0.08,p['label'],fontsize=8,color=INK2,va='top')
axT.set_ylim(-1.5,top); axT.set_xlim(t[0],t[-1]); axT.set_xlabel('collector time (s)'); axT.set_ylabel('cm'); axT.legend(loc='upper right',fontsize=8.5,frameon=False,ncol=3,bbox_to_anchor=(1.0,1.18))
fig.suptitle(f'RACE tag-146 out-and-back, Stonefish, seed {tr["simulator_seed"]["value"]}, job 4ef8719e (2026-10-14)',fontsize=12,color=INK,x=0.08,ha='left',y=0.965)
fig.text(0.08,0.012,f'Frame: Stonefish world, x north, y east, z down (NED). The mission is a list of waypoints with hold times in the controller\'s world_ned frame (request.json); the vehicle moves between them on its position controller.\n{len(D)} front-camera sightings of tag 146 at 7 Hz; not detected {gap[0]:.0f}–{gap[1]:.0f} s. Tag position uses the detector\'s XYZ only, no tag orientation. Reprojecting from the true pose instead gives RMSE {rmseG*100:.1f} cm (camera chain check). Camera offset ahead of cg: {cam_off:.2f} m.',fontsize=8,color=INK2)
fig.savefig(f'{OUT}/race-tag146-out-and-back-bev.png',dpi=160,facecolor=S,bbox_inches='tight',pad_inches=0.25); print('saved')
