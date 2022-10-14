import matplotlib.pyplot as plt
import matplotlib as mpl
mpl.rcParams.update(mpl.rcParamsDefault)
import numpy as np
from matplotlib import cm
import pandas as pd
from stl import mesh
from mpl_toolkits import mplot3d
import os
import json

def m4(m):
    m = np.concatenate((m, np.ones((np.shape(m)[0],1))),axis=1).transpose()
    return m

def u2r(df):
    dff=df.copy()
    dff['X'] = df['X']*(-1)
    dff['Y'] = df['Z']*(-1)
    dff['Z'] = df['Y']*(+1)
    return dff

def plotPosDist(ax,pos,err):
    for i in range(np.shape(pos)[0]-1):
        ax.plot3D(pos.iloc[i:(i+2),0],pos.iloc[i:(i+2),1],pos.iloc[i:(i+2),2],
                  c=cm.RdYlGn(err[i]/np.max(err)),
                  linewidth=2
            )
        
def plotPosForce(ax,pos,force):
    for i in range(np.shape(pos)[0]-1):
        ax.plot3D(pos.iloc[i:(i+2),0],pos.iloc[i:(i+2),1],pos.iloc[i:(i+2),2],
                  c=cm.RdYlGn_r(err[i]/np.max(force)),
                  linewidth=2
            )
    
def plotObstacles(ax, path,c):
    for o in os.listdir(path):
        if o.endswith("_vertices.csv"):
            obv = u2r(pd.read_csv(os.path.join(path,o))).to_numpy()
            obt = pd.read_csv(os.path.join(path,o.split("_")[0]+"_triangulation.csv")).to_numpy()
            m = mesh.Mesh(np.zeros(obt.shape[0], dtype=mesh.Mesh.dtype))
            for i, f in enumerate(obt):
                for j in range(3):
                    try:
                        m.vectors[i][j] = obv[f[j],:]   
                    except: pass
            ax.add_collection3d(mplot3d.art3d.Poly3DCollection(m.vectors,alpha=0.05, ec=c,fc=c,linewidth=0.1))
   
    
def clean_axes(ax):
    ax.xaxis.pane.set_facecolor('w')
    ax.yaxis.pane.set_facecolor('w')
    ax.zaxis.pane.set_facecolor('w')
    ax.get_xaxis().set_ticklabels([])
    ax.get_yaxis().set_ticklabels([])
    ax.get_zaxis().set_ticklabels([])
    
def on_move(event):
    if event.inaxes == axerr:
        if axerr.button_pressed in axerr._rotate_btn:
            axforce.view_init(elev=axerr.elev, azim=axerr.azim)
        elif axerr.button_pressed in axerr._zoom_btn:
            axforce.set_xlim3d(axerr.get_xlim3d())
            axforce.set_ylim3d(axerr.get_ylim3d())
            axforce.set_zlim3d(axerr.get_zlim3d())
    elif event.inaxes == axforce:
        if axforce.button_pressed in axforce._rotate_btn:
            axerr.view_init(elev=axforce.elev, azim=axforce.azim)
        elif axforce.button_pressed in axforce._zoom_btn:
            axerr.set_xlim3d(axforce.get_xlim3d())
            axerr.set_ylim3d(axforce.get_ylim3d())
            axerr.set_zlim3d(axforce.get_zlim3d())
    else:
        return
    fig.canvas.draw_idle()
    
def centerandequal(ax,pos):
    xlim = [np.amin(pos.to_numpy()[:,0]), np.amax(pos.to_numpy()[:,0])]; xrange = xlim[1] - xlim[0]
    ylim = [np.amin(pos.to_numpy()[:,1]), np.amax(pos.to_numpy()[:,1])]; yrange = ylim[1] - ylim[0]
    zlim = [np.amin(pos.to_numpy()[:,2]), np.amax(pos.to_numpy()[:,2])]; zrange = zlim[1] - zlim[0]
    maxrange = max(xrange, yrange, zrange)
    BORDERS = 1.5
    ax.set_xlim3d([(xlim[0]+xlim[1])/2-maxrange/BORDERS,(xlim[0]+xlim[1])/2+maxrange/BORDERS])
    ax.set_ylim3d([(ylim[0]+ylim[1])/2-maxrange/BORDERS,(ylim[0]+ylim[1])/2+maxrange/BORDERS])
    ax.set_zlim3d([(zlim[0]+zlim[1])/2-maxrange/BORDERS,(zlim[0]+zlim[1])/2+maxrange/BORDERS])
    
def rms(data):
    return np.sqrt(np.mean(data**2))

wd = os.path.dirname(os.path.realpath(__file__))
wd+="\\"+os.path.basename(wd)

traj = u2r(pd.read_csv(wd+'_traj0.csv'))

task = pd.read_csv(wd+'_VFs.csv')
pos = u2r(task[['PositionX','PositionY','PositionZ']].rename(
    columns={'PositionX':'X','PositionY':'Y','PositionZ':'Z'}
))
force = u2r(task[['TrajectoryOrientationGuidanceVFRL_forceX0','TrajectoryOrientationGuidanceVFRL_forceY0','TrajectoryOrientationGuidanceVFRL_forceZ0']].rename(
    columns={'TrajectoryOrientationGuidanceVFRL_forceX0':'X','TrajectoryOrientationGuidanceVFRL_forceY0':'Y','TrajectoryOrientationGuidanceVFRL_forceZ0':'Z'}
))
torque = u2r(task[['TrajectoryOrientationGuidanceVFRL_torqueX0','TrajectoryOrientationGuidanceVFRL_torqueY0','TrajectoryOrientationGuidanceVFRL_torqueZ0']].rename(
    columns={'TrajectoryOrientationGuidanceVFRL_torqueX0':'X','TrajectoryOrientationGuidanceVFRL_torqueY0':'Y','TrajectoryOrientationGuidanceVFRL_torqueZ0':'Z'}
))

err = task['TrajectoryOrientationGuidanceVFRL_dist0'].to_numpy()
angle = task['TrajectoryOrientationGuidanceVFRL_angle0'].to_numpy()
time = task['Time'].to_numpy()

eval = dict()
eval["subject"] = wd.split("\\")[-4][-1]
eval["task"] = wd.split("\\")[-3]
eval["repetition"] = wd.split("\\")[-2].split("_")[-1]
eval["assisted"] = np.count_nonzero(task["Assisted"]) > 0.8*len(task)
eval["assistance"] = np.mean(task["Assistance"])
eval["time"] = time[-1]-time[0]
eval["clutch_time"] = np.count_nonzero(task["Clutch"])/len(task)
eval["avg_dist"] = rms(err)
eval["avg_angle"] = rms(angle)
eval["avg_force"] = rms(np.linalg.norm(force.to_numpy(),axis=1))
eval["avg_torque"] = rms(np.linalg.norm(torque.to_numpy(),axis=1))
eval_json = json.dumps(eval, indent=4)
with open(wd+'_eval.json', 'w') as f:
    f.write(eval_json)
    
traj = u2r(pd.read_csv(wd+'_traj0.csv'))

fig = plt.figure(figsize=plt.figaspect(0.5))

axerr = fig.add_subplot(1,2,1,projection='3d'); clean_axes(axerr)
plotPosDist(axerr,pos,err)
plotObstacles(axerr, "C:\\Users\\alber\\Desktop\\Active_Constraints\\Assessment\\PlotSTLs\\"+wd.split("\\")[-3]+"stl","#42b9f5")   
centerandequal(axerr,pos)
axerr.plot(traj['X'].to_numpy(),traj['Y'].to_numpy(),traj['Z'].to_numpy(),color='#00aaff',linewidth=2)
plt.title("D = "+str(eval['avg_dist']))

axforce = fig.add_subplot(1,2,2,projection='3d'); clean_axes(axforce) 
plotPosForce(axforce,pos,force['X']**2 + force['Y']**2 + force['Z']**2)
plotObstacles(axforce, "C:\\Users\\alber\\Desktop\\Active_Constraints\\Assessment\\PlotSTLs\\"+wd.split("\\")[-3]+"stl","#42b9f5")   
centerandequal(axerr,pos)
STRIDE = 5
axforce.quiver(pos['X'].to_numpy()[::STRIDE], pos['Y'].to_numpy()[::STRIDE], pos['Z'].to_numpy()[::STRIDE],  
        force['X'].to_numpy()[::STRIDE], force['Y'].to_numpy()[::STRIDE], force['Z'].to_numpy()[::STRIDE],  
        color= "#0000ff",length=0.005,linewidth=0.5)
axforce.plot(traj['X'].to_numpy(),traj['Y'].to_numpy(),traj['Z'].to_numpy(),color='#00aaff',linewidth=2)
plt.title("F = "+str(eval['avg_force']))

c1 = fig.canvas.mpl_connect('motion_notify_event', on_move)

plt.show()
