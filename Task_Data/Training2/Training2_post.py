import matplotlib.pyplot as plt
import numpy as np
from numpy import genfromtxt
from scipy.spatial.transform import Rotation as Rmat
import os
import json
from stl import mesh
from mpl_toolkits import mplot3d

def m4(m):
    m = np.concatenate((m, np.ones((np.shape(m)[0],1))),axis=1).transpose()
    return m

def u2r(mat):
    matr=mat.copy()
    matr[0,:] = mat[0,:]*(-1)
    matr[1,:] = mat[2,:]*(-1)
    matr[2,:] = mat[1,:]*(+1)
    
    return matr

def main():
    wd = os.path.dirname(os.path.realpath(__file__))
    wd+="\\"+os.path.basename(wd)
    
    sceneG2L = genfromtxt(wd+"_scenetransform.csv", delimiter=',')
    rotG2L = np.eye(4)
    rotG2L[0:3,0:3] = sceneG2L[0:3,0:3]*0.01265188
    rotG2L = rotG2L
    # print(sceneG2L)
    # print(rotG2L)
    
    trajG = genfromtxt(wd+'_coneapproach0.csv', delimiter=',')[1:,0:3]
    posG = genfromtxt(wd+'_VFs.csv', delimiter=',')[1:,1:4]
    forceG = genfromtxt(wd+'_VFs.csv', delimiter=',')[1:,4:7]
    
    trajL = u2r(np.matmul(sceneG2L, m4(trajG)))
    posL = u2r(np.matmul(sceneG2L, m4(posG)))
    forceL = u2r(np.matmul(rotG2L, m4(forceG)))
    
    # JSON DUMP
    eval = dict()
    eval["time"] = genfromtxt(wd+'_VFs.csv', delimiter=',')[-1,0]-genfromtxt(wd+'_VFs.csv', delimiter=',')[1,0]
    
    trajlerp = np.linspace(trajL[:,0],trajL[:,-1],200).transpose()
    err = np.zeros((np.shape(posL)[1],1))
    # print(np.shape(posL))
    # print(np.shape(trajlerp))
    for i, p in enumerate(posL.transpose()):
        mind=1000
        for j, t in enumerate(trajlerp.transpose()):
            # print(p, t)
            if np.linalg.norm(t-p)<mind:
                mind=np.linalg.norm(t-p)
        err[i] = mind
    eval["avg_dist"] = np.mean(err)
    
    eval["avg_force"] = np.mean(np.linalg.norm(forceL,axis=0))
    eval_json = json.dumps(eval, indent=4)
    with open(wd+'_eval.json', 'w') as f:
        f.write(eval_json)
    # PLOTTING
    ax = plt.axes(projection='3d')
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.w_xaxis.set_pane_color((1,1,1,1))
    ax.w_yaxis.set_pane_color((1,1,1,1))
    ax.w_zaxis.set_pane_color((1,1,1,1))
    
    ax.set_box_aspect(aspect = (1,1,1))
    
    
    parentf = "C:\\Users\\alber\\Desktop\\Active_Constraints\\Task_Data\\Training2\\SCENESTLS"
    for stl in os.listdir(parentf):
        print(parentf+"\\"+stl)
        if stl.split("_")[-1] == "global.csv":
            scenemeshG = genfromtxt(parentf+"\\"+stl, delimiter=',')[1:,0:3]
            scenemeshL = u2r(np.matmul(sceneG2L, m4(scenemeshG)))
            # print(scenemeshL.transpose())
        if stl.split("_")[-1] == "tri.csv":
            scenetris = genfromtxt(parentf+"\\"+stl, delimiter=',')[1:,0:3]
            # print(scenetris)
            pmesh = mesh.Mesh(np.zeros(scenetris.shape[0], dtype=mesh.Mesh.dtype))
            for i, f in enumerate(scenetris):
                for j in range(3):
                    try:
                        pmesh.vectors[i][j] = scenemeshL.transpose()[int(f[j]),0:3]
                    except: pass
            ax.add_collection3d(mplot3d.art3d.Poly3DCollection(pmesh.vectors,
                    edgecolors='k', facecolors='w', alpha=0.1, linewidths=0.1))

    plt.tight_layout()
    ax.set_xlim([-4,4])
    ax.set_ylim([-4,4])
    ax.set_zlim([-4,4])

    
    ax.plot3D(posL[0,:], posL[1,:], posL[2,:], color= "#ff0000",linewidth=1)
    ax.plot3D(trajL[0,:], trajL[1,:], trajL[2,:], color= "#00ff00",linewidth=0.8)
    ax.quiver(posL[0,:], posL[1,:], posL[2,:], forceL[0,:], forceL[1,:], forceL[2,:], color= "#0000ff",length=0.05,linewidth=0.5)
    
    plt.axis('off')
    plt.show()
    
    
if __name__ == '__main__':
    main()