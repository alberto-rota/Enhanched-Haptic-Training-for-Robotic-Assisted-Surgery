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

wd = os.path.join(os.path.dirname(os.path.realpath(__file__)))
wd = os.path.join(wd, os.path.basename(wd))

# ob0 = u2r(pd.read_csv(wd+'_obst0.csv'))
task = pd.read_csv(wd+'_VFs.csv')
pos = u2r(task[['PositionX','PositionY','PositionZ']].rename(
    columns={'PositionX':'X','PositionY':'Y','PositionZ':'Z'}
))
force = u2r(task[['ObstacleAvoidanceForceFieldVF_X0','ObstacleAvoidanceForceFieldVF_Y0','ObstacleAvoidanceForceFieldVF_Z0']].rename(
    columns={'ObstacleAvoidanceForceFieldVF_X0':'X','ObstacleAvoidanceForceFieldVF_Y0':'Y','ObstacleAvoidanceForceFieldVF_Z0':'Z'}
))

err = task['ObstacleAvoidanceForceFieldVF_dist0'].to_numpy()
time = task['Time'].to_numpy()

eval = dict()
eval["time"] = time[-1]-time[0]
eval["avg_dist"] = np.mean(err)
eval["avg_force"] = np.mean(np.linalg.norm(force.to_numpy(),axis=1))
eval_json = json.dumps(eval, indent=4)
with open(wd+'_eval.json', 'w') as f:
    f.write(eval_json)

print(eval)
print("> JSON written to dir: "+wd+'_eval.json\n')