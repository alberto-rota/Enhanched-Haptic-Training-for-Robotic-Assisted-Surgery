import matplotlib.pyplot as plt
import matplotlib as mpl
mpl.rcParams.update(mpl.rcParamsDefault)
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd
import numpy as np

dff = pd.read_csv("C:\\Users\\alber\\Desktop\\MSc_Thesis\\Active Constraints\\Assets\\Logs\\SandBox_26.csv")
dff.shape
df = dff.iloc[1::5,:]
q = np.array([np.array(df.PositionX),np.array(df.PositionY),np.array(df.PositionZ),np.array(df.TrajectoryGuidanceVF_X),np.array(df.TrajectoryGuidanceVF_Y),np.array(df.TrajectoryGuidanceVF_Z)])
# q = np.array([np.array(df.PositionX),np.array(df.PositionY),np.array(df.PositionZ),np.array(df.Trajectory),np.array(df.ObstacleAvoidanceForceFieldVF_Y),np.array(df.ObstacleAvoidanceForceFieldVF_Z)])
X,Y,Z,U,V,W = zip(q)

fig = plt.figure()
ax = fig.gca(projection='3d')
# ax.quiver(X,Y,Z,U,V,W)
ax.scatter(np.array(dff.PositionZ),np.array(dff.PositionY),np.array(dff.PositionX),color='r',s=1)
plt.tight_layout()
plt.xlabel("X")
plt.ylabel("Y")
# plt.zlabel("Z")
plt.show()