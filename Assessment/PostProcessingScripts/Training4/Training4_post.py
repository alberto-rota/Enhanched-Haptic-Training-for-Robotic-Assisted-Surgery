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
eval["subject"] = wd.split("\\")[-4][-1]
eval["task"] = wd.split("\\")[-3]
eval["repetition"] = wd.split("\\")[-2][-1]
eval["time"] = time[-1]-time[0]
eval["avg_dist"] = np.mean(err)
eval["avg_force"] = np.mean(np.linalg.norm(force.to_numpy(),axis=1))
eval_json = json.dumps(eval, indent=4)
with open(wd+'_eval.json', 'w') as f:
    f.write(eval_json)
    
print(eval)
print("> JSON written to dir"+wd+'_eval.json\n')