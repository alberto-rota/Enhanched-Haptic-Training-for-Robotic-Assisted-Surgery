import numpy as np
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

def rms(data):
    return np.sqrt(np.mean(data**2))
    
wd = os.path.join(os.path.dirname(os.path.realpath(__file__)))
wd = os.path.join(wd, os.path.basename(wd))

# ob0 = u2r(pd.read_csv(wd+'_obst0.csv'))
task = pd.read_csv(wd+'_VFs.csv')
pos = u2r(task[['PositionX','PositionY','PositionZ']].rename(
    columns={'PositionX':'X','PositionY':'Y','PositionZ':'Z'}
))
force = u2r(task[['SurfaceOrientationGuidanceVF_forceX0','SurfaceOrientationGuidanceVF_forceY0','SurfaceOrientationGuidanceVF_forceZ0']].rename(
    columns={'SurfaceOrientationGuidanceVF_forceX0':'X','SurfaceOrientationGuidanceVF_forceY0':'Y','SurfaceOrientationGuidanceVF_forceZ0':'Z'}
))
torque = u2r(task[['SurfaceOrientationGuidanceVF_torqueX0','SurfaceOrientationGuidanceVF_torqueY0','SurfaceOrientationGuidanceVF_torqueZ0']].rename(
    columns={'SurfaceOrientationGuidanceVF_torqueX0':'X','SurfaceOrientationGuidanceVF_torqueY0':'Y','SurfaceOrientationGuidanceVF_torqueZ0':'Z'}
))

err = task['SurfaceOrientationGuidanceVF_dist0'].to_numpy()
angle = task['SurfaceOrientationGuidanceVF_angle0'].to_numpy()
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

print(eval)
print("> JSON written to dir: "+wd+'_eval.json\n')