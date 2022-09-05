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
force = np.zeros(np.shape(pos))
err = 1000*np.ones((np.shape(pos)[0],))
for o in range(0,(np.shape(task)[1]-4)//4-1):
    force = force + u2r(task[['ObstacleAvoidanceForceFieldVF_forceX'+str(o),'ObstacleAvoidanceForceFieldVF_forceY'+str(o),'ObstacleAvoidanceForceFieldVF_forceZ'+str(o)]].rename(
        columns={'ObstacleAvoidanceForceFieldVF_forceX'+str(o):'X','ObstacleAvoidanceForceFieldVF_forceY'+str(o):'Y','ObstacleAvoidanceForceFieldVF_forceZ'+str(o):'Z'}
    )).to_numpy()
    err = np.amin(np.concatenate((np.expand_dims(err,axis=1), np.expand_dims(task['ObstacleAvoidanceForceFieldVF_dist'+str(o)].to_numpy(),axis=1)), axis=1),axis=1)
    
force = pd.DataFrame(force,columns=['X','Y','Z'])
time = task['Time'].to_numpy()

eval = dict()
eval["subject"] = wd.split("\\")[-4][-1]
eval["task"] = wd.split("\\")[-3]
eval["repetition"] = wd.split("\\")[-2].split("_")[-1]
eval["assisted"] = np.count_nonzero(task["Assisted"]) > 0.8*len(task)
eval["assistance"] = np.mean(task["Assistance"])
eval["time"] = time[-1]-time[0]
eval["avg_dist"] = rms(err)
eval["avg_force"] = rms(np.linalg.norm(force.to_numpy(),axis=1))

eval_json = json.dumps(eval, indent=4)
with open(wd+'_eval.json', 'w') as f:
    f.write(eval_json)

print(eval)
print("> JSON written to dir: "+wd+'_eval.json\n')