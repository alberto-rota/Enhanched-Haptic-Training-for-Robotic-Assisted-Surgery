// Copyright (c) 2022 Alberto Rota
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using UnityEngine.SceneManagement;


public class LogDataNephrectomy : MonoBehaviour
{

    public List<MonoBehaviour> activeConstraints;
    public string saveTo = @"C:\Users\alber\Desktop\Active_Constraints\Assets\Logs";
    string foldername;
    string folderpath;
    string path;

    string GetUniqueName(string name, string folderPath) {
        string validatedName = name+"_0";
        int tries = 0;
        while (Directory.Exists(folderPath+"\\"+validatedName)) {
            tries++;
            validatedName = name+"_"+tries.ToString();
        }
        return validatedName;
    }

    void Start() {
        // Checks which VFs are activated and enabled
        activeConstraints = new List<MonoBehaviour>();
        foreach (MonoBehaviour s in gameObject.GetComponents<MonoBehaviour>()) {
            if (s.GetType().Name != "LogData" && s.enabled == true) {
                activeConstraints.Add(s);
            }
        }   

        // Creates the UNIQUE folder to save the logs
        foldername = GetUniqueName(SceneManager.GetActiveScene().name, saveTo);
        folderpath = saveTo+"\\"+foldername;
        System.IO.Directory.CreateDirectory(folderpath);    
        Debug.Log("Task data will be saved to: "+folderpath);
        // Creates the .m file to save the logs
        File.Copy("C:\\Users\\alber\\Desktop\\Active_Constraints\\Task_Data\\Nephrectomy\\NephrectomyPostOriginal.m",
            folderpath+"\\NephrectomyPost.m");

        // SAVES NON-CHANGING DATA (TRAJECTORIES, OBSTACLES, ...)
        if (activeConstraints.Contains(gameObject.GetComponent<TrajectoryGuidanceVF>())) {
            for (int j=0; j<gameObject.GetComponents<TrajectoryGuidanceVF>().Length; j++) {
                path = folderpath+"\\"+foldername+"_traj"+j+".csv";
                FileStream streamtraj = new FileStream(path, FileMode.Append);  
                using (StreamWriter writer = new StreamWriter(streamtraj))  
                {  
                    writer.Write("X,Y,Z,\n");
                    for (int i=0; i<gameObject.GetComponents<TrajectoryGuidanceVF>()[j].Trajectory.GetComponent<LineRenderer>().positionCount; i++) {
                        Vector3 point = gameObject.GetComponents<TrajectoryGuidanceVF>()[j].Trajectory.GetComponent<LineRenderer>().GetPosition(i);
                        writer.Write(point.x.ToString()+",");
                        writer.Write(point.y.ToString()+",");
                        writer.Write(point.z.ToString()+",\n");
                    }
                }
            }

        }
        if (activeConstraints.Contains(gameObject.GetComponent<ObstacleAvoidanceForceFieldVF>())) {
            for (int j=0; j<gameObject.GetComponents<ObstacleAvoidanceForceFieldVF>().Length; j++) {
                path = folderpath+"\\"+foldername+"_obst"+j+".csv";
                FileStream streamtraj = new FileStream(path, FileMode.Append);  
                using (StreamWriter writer = new StreamWriter(streamtraj))  
                {  
                    writer.Write("X,Y,Z,\n");
                    Mesh obst_mesh = gameObject.GetComponents<ObstacleAvoidanceForceFieldVF>()[j].obstacle.GetComponent<MeshFilter>().sharedMesh;
                    for (int i=0; i<obst_mesh.vertices.Length; i++) {
                        Vector3 point = gameObject.GetComponents<ObstacleAvoidanceForceFieldVF>()[j].obstacle.transform.TransformPoint(obst_mesh.vertices[i]);
                        writer.Write(point.x.ToString()+",");
                        writer.Write(point.y.ToString()+",");
                        writer.Write(point.z.ToString()+",\n");
                    }
                }
            }
        }
        if (activeConstraints.Contains(gameObject.GetComponent<SurfaceAvoidanceVF>())) {
            for (int j=0; j<gameObject.GetComponents<SurfaceAvoidanceVF>().Length; j++) {
                path = folderpath+"\\"+foldername+"_surfavoid"+j+".csv";
                FileStream streamtraj = new FileStream(path, FileMode.Append);  
                using (StreamWriter writer = new StreamWriter(streamtraj))  
                {  
                    writer.Write("X,Y,Z,\n");
                    Mesh obst_mesh = gameObject.GetComponents<SurfaceAvoidanceVF>()[j].surface.GetComponent<MeshFilter>().sharedMesh;
                    for (int i=0; i<obst_mesh.vertices.Length; i++) {
                        Vector3 point = gameObject.GetComponents<SurfaceAvoidanceVF>()[j].surface.transform.TransformPoint(obst_mesh.vertices[i]);
                        writer.Write(point.x.ToString()+",");
                        writer.Write(point.y.ToString()+",");
                        writer.Write(point.z.ToString()+",\n");
                    }
                }
            }
        }
        if (activeConstraints.Contains(gameObject.GetComponent<SurfaceGuidanceVF>())) {
            for (int j=0; j<gameObject.GetComponents<SurfaceGuidanceVF>().Length; j++) {
                path = folderpath+"\\"+foldername+"_surfguide"+j+".csv";
                FileStream streamtraj = new FileStream(path, FileMode.Append);  
                using (StreamWriter writer = new StreamWriter(streamtraj))  
                {  
                    writer.Write("X,Y,Z,\n");
                    Mesh obst_mesh = gameObject.GetComponents<SurfaceGuidanceVF>()[j].surface.GetComponent<MeshFilter>().sharedMesh;
                    for (int i=0; i<obst_mesh.vertices.Length; i++) {
                        Vector3 point = gameObject.GetComponents<SurfaceGuidanceVF>()[j].surface.transform.TransformPoint(obst_mesh.vertices[i]);
                        writer.Write(point.x.ToString()+",");
                        writer.Write(point.y.ToString()+",");
                        writer.Write(point.z.ToString()+",\n");
                    }
                }
            }
        }
        if (activeConstraints.Contains(gameObject.GetComponent<ConeApproachGuidanceVF>())) {
        }
        if (activeConstraints.Contains(gameObject.GetComponent<SumForces>())) {
        }

        path = folderpath+"\\"+foldername+"_VFs.csv";
        FileStream stream = new FileStream(path, FileMode.Append);  
        using (StreamWriter writer = new StreamWriter(stream))  
        {  
            // POSITION HEADER
            writer.Write("PositionX,PositionY,PositionZ,");

            // VFs HEADERS
            if (activeConstraints.Contains(gameObject.GetComponent<TrajectoryGuidanceVF>())) {
                for (int j=0; j<gameObject.GetComponents<TrajectoryGuidanceVF>().Length; j++) {
                    writer.Write("TrajectoryGuidanceVF_X"+j+",");
                    writer.Write("TrajectoryGuidanceVF_Y"+j+",");
                    writer.Write("TrajectoryGuidanceVF_Z"+j+",");
                }
            }
            if (activeConstraints.Contains(gameObject.GetComponent<ObstacleAvoidanceForceFieldVF>())) {
                for (int j=0; j<gameObject.GetComponents<ObstacleAvoidanceForceFieldVF>().Length; j++) {
                    writer.Write("ObstacleAvoidanceForceFieldVF_X"+j+",");
                    writer.Write("ObstacleAvoidanceForceFieldVF_Y"+j+",");
                    writer.Write("ObstacleAvoidanceForceFieldVF_Z"+j+",");
                }
            }
            if (activeConstraints.Contains(gameObject.GetComponent<SurfaceAvoidanceVF>())) {
                for (int j=0; j<gameObject.GetComponents<SurfaceAvoidanceVF>().Length; j++) {
                    writer.Write("SurfaceAvoidanceVF_X"+j+",");
                    writer.Write("SurfaceAvoidanceVF_Y"+j+",");
                    writer.Write("SurfaceAvoidanceVF_Z"+j+",");
                }
            }
            if (activeConstraints.Contains(gameObject.GetComponent<SurfaceGuidanceVF>())) {
                for (int j=0; j<gameObject.GetComponents<SurfaceGuidanceVF>().Length; j++) {
                    writer.Write("SurfaceGuidanceVF_X"+j+",");
                    writer.Write("SurfaceGuidanceVF_Y"+j+",");
                    writer.Write("SurfaceGuidanceVF_Z"+j+",");
                }
            }
            if (activeConstraints.Contains(gameObject.GetComponent<ConeApproachGuidanceVF>())) {
                for (int j=0; j<gameObject.GetComponents<ConeApproachGuidanceVF>().Length; j++) {
                    writer.Write("ConeApproachGuidanceVF_X"+j+",");
                    writer.Write("ConeApproachGuidanceVF_Y"+j+",");
                    writer.Write("ConeApproachGuidanceVF_Z"+j+",");
                }
            }
            if (activeConstraints.Contains(gameObject.GetComponent<SumForces>())) {
                for (int j=0; j<gameObject.GetComponents<SumForces>().Length; j++) {
                    writer.Write("TotalForce_X"+j+",");
                    writer.Write("TotalForce_Y"+j+",");
                    writer.Write("TotalForce_Z"+j+",");
                }
            }

            writer.Write("\n");
        }  

        // ADD GENERATION OF SCRIPT FOR GRAPHICS [MATLAB/PYTHON]
    }

    void Update()
    {
        FileStream stream = new FileStream(path, FileMode.Append);  
        using (StreamWriter writer = new StreamWriter(stream))  
        {  
            // Writes the current position
            writer.Write(gameObject.transform.position.x); writer.Write(","); 
            writer.Write(gameObject.transform.position.y); writer.Write(","); 
            writer.Write(gameObject.transform.position.z); writer.Write(","); 
            Vector3 f;
            if (activeConstraints.Contains(gameObject.GetComponent<TrajectoryGuidanceVF>())) {
                for (int j=0; j<gameObject.GetComponents<TrajectoryGuidanceVF>().Length; j++) {
                    f = gameObject.GetComponents<TrajectoryGuidanceVF>()[j].force;
                    writer.Write(f.x); writer.Write(","); 
                    writer.Write(f.y); writer.Write(","); 
                    writer.Write(f.z); writer.Write(","); 
                }
            }
            if (activeConstraints.Contains(gameObject.GetComponent<ObstacleAvoidanceForceFieldVF>())) {
                for (int j=0; j<gameObject.GetComponents<ObstacleAvoidanceForceFieldVF>().Length; j++) {
                    f = gameObject.GetComponents<ObstacleAvoidanceForceFieldVF>()[j].force;
                    writer.Write(f.x); writer.Write(","); 
                    writer.Write(f.y); writer.Write(","); 
                    writer.Write(f.z); writer.Write(","); 
                }
            }
            if (activeConstraints.Contains(gameObject.GetComponent<SurfaceAvoidanceVF>())) {
                for (int j=0; j<gameObject.GetComponents<SurfaceAvoidanceVF>().Length; j++) {
                    f = gameObject.GetComponents<SurfaceAvoidanceVF>()[j].force;
                    writer.Write(f.x); writer.Write(","); 
                    writer.Write(f.y); writer.Write(","); 
                    writer.Write(f.z); writer.Write(","); 
                }
            }
            if (activeConstraints.Contains(gameObject.GetComponent<SurfaceGuidanceVF>())) {
                for (int j=0; j<gameObject.GetComponents<SurfaceGuidanceVF>().Length; j++) {
                    f = gameObject.GetComponents<SurfaceGuidanceVF>()[j].force;
                    writer.Write(f.x); writer.Write(","); 
                    writer.Write(f.y); writer.Write(","); 
                    writer.Write(f.z); writer.Write(","); 
                }
            }
            if (activeConstraints.Contains(gameObject.GetComponent<ConeApproachGuidanceVF>())) {
                for (int j=0; j<gameObject.GetComponents<ConeApproachGuidanceVF>().Length; j++) {
                    f = gameObject.GetComponents<ConeApproachGuidanceVF>()[j].force;
                    writer.Write(f.x); writer.Write(","); 
                    writer.Write(f.y); writer.Write(","); 
                    writer.Write(f.z); writer.Write(","); 
                }
            }
            if (activeConstraints.Contains(gameObject.GetComponent<SumForces>())) {
                for (int j=0; j<gameObject.GetComponents<SumForces>().Length; j++) {
                    f = gameObject.GetComponents<SumForces>()[j].totalForce;
                    writer.Write(f.x); writer.Write(","); 
                    writer.Write(f.y); writer.Write(","); 
                    writer.Write(f.z); writer.Write(","); 
                }
            }
            
            writer.Write("\n");
        }  
    }
}
