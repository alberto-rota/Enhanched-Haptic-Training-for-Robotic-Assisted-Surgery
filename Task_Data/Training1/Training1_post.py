import matplotlib.pyplot as plt
import matplotlib as mpl
mpl.rcParams.update(mpl.rcParamsDefault)
import numpy as np
from matplotlib.colors import LinearSegmentedColormap
import pandas as pd
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
    gyr = mpl.colors.LinearSegmentedColormap.from_list("", ["green","yellow","red"])
    for i in range(np.shape(pos)[0]-1):
        ax.plot3D(pos.iloc[i:(i+2),0],pos.iloc[i:(i+2),1],pos.iloc[i:(i+2),2],
                #   c=cm.RdYlGn_r(err[i]/np.max(err)),
                  c=gyr(err[i]/np.max(err)),
                #   c=cmapp[err[i]/np.max(err)],
                  linewidth=2
            )
        
def plotForceMag(ax,force):
    fmag2 = force['X']**2 + force['Y']**2 + force['Z']**2
    for i in range(np.shape(force)[0]-1,30):
        ax.quiver(
            pos.iloc[i:(i+2),0],pos.iloc[i:(i+2),1],pos.iloc[i:(i+2),2],
            force.iloc[i:(i+2),0],force.iloc[i:(i+2),1],force.iloc[i:(i+2),2],
                #   color=cm.rainbow(fmag2[i]/np.max(fmag2)), length=0.0025, linewidth=0.5
                color= "#ff0000",length=0.0001,linewidth=0.1
            )
    
    # axforce.quiver(pos['X'].to_numpy()[::5], pos['Y'].to_numpy()[::5], pos['Z'].to_numpy()[::5],  
    #       force['X'].to_numpy()[::5], force['Y'].to_numpy()[::5], force['Z'].to_numpy()[::5],  
    #       color= "#0000ff",length=0.005,linewidth=0.5)
    
    
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
        fig.canvas.draw_idle()
    
wd = os.path.dirname(os.path.realpath(__file__))
wd+="\\"+os.path.basename(wd)

traj = u2r(pd.read_csv(wd+'_traj0.csv'))
task = pd.read_csv(wd+'_VFs.csv')
pos = u2r(task[['PositionX','PositionY','PositionZ']].rename(
    columns={'PositionX':'X','PositionY':'Y','PositionZ':'Z'}
))
force = u2r(task[['TrajectoryGuidanceVF_X0','TrajectoryGuidanceVF_Y0','TrajectoryGuidanceVF_Z0']].rename(
    columns={'TrajectoryGuidanceVF_X0':'X','TrajectoryGuidanceVF_Y0':'Y','TrajectoryGuidanceVF_Z0':'Z'}
))
err = task['TrajectoryGuidanceVF_dist0'].to_numpy()
time = task['Time'].to_numpy()

eval = dict()
eval["time"] = time[-1]-time[0]
eval["avg_dist"] = np.mean(err)
eval["avg_force"] = np.mean(np.linalg.norm(force.to_numpy(),axis=1))
eval_json = json.dumps(eval, indent=4)
with open(wd+'_eval.json', 'w') as f:
    f.write(eval_json)

fig = plt.figure(figsize=plt.figaspect(0.5))

# cmapp = mpl.colors.ListedColormap.from_list(["green","yellow","red"])

axerr = fig.add_subplot(1,2,1,projection='3d'); clean_axes(axerr)
axerr.plot3D(traj['X'],traj['Y'],traj['Z'], color= "#0088ff",linewidth=2)
plotPosDist(axerr,pos,err)
plt.title("Distance from Reference Trajectory: D = "+str(eval['avg_dist']))

axforce = fig.add_subplot(1,2,2,projection='3d'); clean_axes(axforce) 
axforce.plot3D(traj['X'],traj['Y'],traj['Z'], color= "#0088ff")
plotPosDist(axforce,pos,force['X']**2 + force['Y']**2 + force['Z']**2)
plt.title("Haptic Feedback Force: F = "+str(eval['avg_force']))

c1 = fig.canvas.mpl_connect('motion_notify_event', on_move)

plt.show()
