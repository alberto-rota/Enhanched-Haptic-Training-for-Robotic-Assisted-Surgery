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
using RosSharp.RosBridgeClient;



public class LogData : MonoBehaviour
{

    public string TASKNAME;
    public List<MonoBehaviour> activeConstraints;
    public string saveTo;
    public Transform subject;
    string foldername;
    string folderpath;
    string path;
    GameObject robot;

    string GetUniqueName(string name, string folderPath) {
        string validatedName = name+"_0";
        int tries = 0;
        while (Directory.Exists(folderPath+"\\"+validatedName)) {
            tries++;
            validatedName = name+"_"+tries.ToString();
        }
        return validatedName;
    }

    public void OnEnable() {
        TASKNAME = SceneManager.GetActiveScene().name;
        robot = GameObject.FindWithTag("ROBOT");

        // Checks which VFs are activated and enabled
        activeConstraints = new List<MonoBehaviour>();
        foreach (MonoBehaviour s in robot.GetComponents<MonoBehaviour>()) {
            if ((s.GetType().Name == "ConeApproachGuidanceVF"||
                 s.GetType().Name == "TrajectoryGuidanceVF"||
                 s.GetType().Name == "TrajectoryGuidanceVFRL"||
                 s.GetType().Name == "TrajectoryOrientationGuidanceVF"||
                 s.GetType().Name == "TrajectoryOrientationGuidanceVFRL"||
                 s.GetType().Name == "ObstacleAvoidanceForceFieldVF"||
                 s.GetType().Name == "SurfaceGuidanceVF"||
                 s.GetType().Name == "SurfaceOrientationGuidanceVF"||
                 s.GetType().Name == "SurfaceAvoidanceVF")
                && s.enabled == true) {
                activeConstraints.Add(s);
            }
        }   

        saveTo = "C:\\Users\\alber\\Desktop\\Active_Constraints\\Assessment\\Pre-Study\\Subject";
        saveTo += Global.subjectID;
        // Creates the UNIQUE folder to save the logs
        foldername = GetUniqueName(SceneManager.GetActiveScene().name, saveTo+"\\"+TASKNAME);
        folderpath = saveTo+"\\"+TASKNAME+"\\"+foldername;

        // Creates the folder
        System.IO.Directory.CreateDirectory(folderpath);    
        Debug.Log("Task data will be saved to: "+folderpath);
        Global.DebugOnHRSV("Task data will be saved to: "+folderpath);
        // Creates the .m file to save the logs
        string originalpostpath = "C:\\Users\\alber\\Desktop\\Active_Constraints\\Assessment\\PostProcessingScripts";
        File.Copy(originalpostpath+"\\"+TASKNAME+"\\"+TASKNAME+"_post.py", folderpath+"\\"+TASKNAME+"_post.py");
        File.Copy(originalpostpath+"\\"+TASKNAME+"\\"+TASKNAME+"_post.bat", folderpath+"\\"+TASKNAME+"_post.bat");
        File.Copy(originalpostpath+"\\"+TASKNAME+"\\"+TASKNAME+"_graph.py", folderpath+"\\"+TASKNAME+"_graph.py");
        File.Copy(originalpostpath+"\\"+TASKNAME+"\\"+TASKNAME+"_graph.bat", folderpath+"\\"+TASKNAME+"_graph.bat");

        // SAVES NON-CHANGING DATA (TRAJECTORIES, OBSTACLES, ...)
        path = folderpath+"\\"+foldername+"_scenetransform.csv";
        Matrix4x4 sceneTransform = GameObject.Find(TASKNAME).transform.worldToLocalMatrix;
        FileStream streamtransf = new FileStream(path, FileMode.Append);  
        using (StreamWriter writer = new StreamWriter(streamtransf))  
        {  
            writer.WriteLine(sceneTransform[0,0]+","+sceneTransform[0,1]+","+sceneTransform[0,2]+","+sceneTransform[0,3]);
            writer.WriteLine(sceneTransform[1,0]+","+sceneTransform[1,1]+","+sceneTransform[1,2]+","+sceneTransform[1,3]);
            writer.WriteLine(sceneTransform[2,0]+","+sceneTransform[2,1]+","+sceneTransform[2,2]+","+sceneTransform[2,3]);
            writer.WriteLine(sceneTransform[3,0]+","+sceneTransform[3,1]+","+sceneTransform[3,2]+","+sceneTransform[3,3]);
        }

        if (activeConstraints.Contains(robot.GetComponent<TrajectoryGuidanceVF>())) {
            for (int j=0; j<robot.GetComponents<TrajectoryGuidanceVF>().Length; j++) {
                path = folderpath+"\\"+foldername+"_traj"+j+".csv";
                FileStream streamtraj = new FileStream(path, FileMode.Append);  
                using (StreamWriter writer = new StreamWriter(streamtraj))  
                {  
                    writer.Write("X,Y,Z\n");
                    for (int i=0; i<robot.GetComponents<TrajectoryGuidanceVF>()[j].Trajectory.GetComponent<LineRenderer>().positionCount; i++) {
                        Vector3 point = robot.GetComponents<TrajectoryGuidanceVF>()[j].Trajectory.GetComponent<LineRenderer>().GetPosition(i);
                        writer.Write(point.x.ToString()+",");
                        writer.Write(point.y.ToString()+",");
                        writer.Write(point.z.ToString()+"\n");
                    }
                }
            }

        }
        if (activeConstraints.Contains(robot.GetComponent<TrajectoryGuidanceVFRL>())) {
            for (int j=0; j<robot.GetComponents<TrajectoryGuidanceVFRL>().Length; j++) {
                path = folderpath+"\\"+foldername+"_traj"+j+".csv";
                FileStream streamtraj = new FileStream(path, FileMode.Append);  
                using (StreamWriter writer = new StreamWriter(streamtraj))  
                {  
                    writer.Write("X,Y,Z\n");
                    for (int i=0; i<robot.GetComponents<TrajectoryGuidanceVFRL>()[j].Trajectory.GetComponent<LineRenderer>().positionCount; i++) {
                        Vector3 point = robot.GetComponents<TrajectoryGuidanceVFRL>()[j].Trajectory.GetComponent<LineRenderer>().GetPosition(i);
                        writer.Write(point.x.ToString()+",");
                        writer.Write(point.y.ToString()+",");
                        writer.Write(point.z.ToString()+"\n");
                    }
                }
            }

        }
        if (activeConstraints.Contains(robot.GetComponent<TrajectoryOrientationGuidanceVF>())) {
            for (int j=0; j<robot.GetComponents<TrajectoryOrientationGuidanceVF>().Length; j++) {
                path = folderpath+"\\"+foldername+"_traj"+j+".csv";
                FileStream streamtraj = new FileStream(path, FileMode.Append);  
                using (StreamWriter writer = new StreamWriter(streamtraj))  
                {  
                    writer.Write("X,Y,Z\n");
                    for (int i=0; i<robot.GetComponents<TrajectoryOrientationGuidanceVF>()[j].Trajectory.GetComponent<LineRenderer>().positionCount; i++) {
                        Vector3 point = robot.GetComponents<TrajectoryOrientationGuidanceVF>()[j].Trajectory.GetComponent<LineRenderer>().GetPosition(i);
                        writer.Write(point.x.ToString()+",");
                        writer.Write(point.y.ToString()+",");
                        writer.Write(point.z.ToString()+"\n");
                    }
                }
            }

        }
        if (activeConstraints.Contains(robot.GetComponent<TrajectoryOrientationGuidanceVFRL>())) {
            for (int j=0; j<robot.GetComponents<TrajectoryOrientationGuidanceVFRL>().Length; j++) {
                path = folderpath+"\\"+foldername+"_traj"+j+".csv";
                FileStream streamtraj = new FileStream(path, FileMode.Append);  
                using (StreamWriter writer = new StreamWriter(streamtraj))  
                {  
                    writer.Write("X,Y,Z\n");
                    for (int i=0; i<robot.GetComponents<TrajectoryOrientationGuidanceVFRL>()[j].Trajectory.GetComponent<LineRenderer>().positionCount; i++) {
                        Vector3 point = robot.GetComponents<TrajectoryOrientationGuidanceVFRL>()[j].Trajectory.GetComponent<LineRenderer>().GetPosition(i);
                        writer.Write(point.x.ToString()+",");
                        writer.Write(point.y.ToString()+",");
                        writer.Write(point.z.ToString()+"\n");
                    }
                }
            }

        }
        if (activeConstraints.Contains(robot.GetComponent<ObstacleAvoidanceForceFieldVF>())) {

        }
        if (activeConstraints.Contains(robot.GetComponent<SurfaceAvoidanceVF>())) {
            for (int j=0; j<robot.GetComponents<SurfaceAvoidanceVF>().Length; j++) {
                path = folderpath+"\\"+foldername+"_surfavoid"+j+".csv";
                FileStream streamtraj = new FileStream(path, FileMode.Append);  
                using (StreamWriter writer = new StreamWriter(streamtraj))  
                {  
                    writer.Write("X,Y,Z\n");
                    Mesh obst_mesh = robot.GetComponents<SurfaceAvoidanceVF>()[j].surface.GetComponent<MeshFilter>().sharedMesh;
                    for (int i=0; i<obst_mesh.vertices.Length; i++) {
                        Vector3 point = robot.GetComponents<SurfaceAvoidanceVF>()[j].surface.transform.TransformPoint(obst_mesh.vertices[i]);
                        writer.Write(point.x.ToString()+",");
                        writer.Write(point.y.ToString()+",");
                        writer.Write(point.z.ToString()+"\n");
                    }
                }
            }
        }
        if (activeConstraints.Contains(robot.GetComponent<SurfaceGuidanceVF>())) {

        }
        if (activeConstraints.Contains(robot.GetComponent<ConeApproachGuidanceVF>())) {
            for (int j=0; j<robot.GetComponents<ConeApproachGuidanceVF>().Length; j++) {
                path = folderpath+"\\"+foldername+"_coneapproach"+j+".csv";
                FileStream streamtraj = new FileStream(path, FileMode.Append);  
                using (StreamWriter writer = new StreamWriter(streamtraj))  
                {  
                        Vector3 start = robot.GetComponents<ConeApproachGuidanceVF>()[j].target.position-robot.GetComponents<ConeApproachGuidanceVF>()[j].delta;
                        Vector3 end = robot.GetComponents<ConeApproachGuidanceVF>()[j].target.position;
                        writer.Write("X,Y,Z\n");
                        writer.Write(start.x.ToString()+",");
                        writer.Write(start.y.ToString()+",");
                        writer.Write(start.z.ToString()+"\n");
                        writer.Write(end.x.ToString()+",");
                        writer.Write(end.y.ToString()+",");
                        writer.Write(end.z.ToString()+"\n");
                    
                }
            }
        }
        if (activeConstraints.Contains(robot.GetComponent<SumForces>())) {
        }

// ! --- HEADERS --- 
        path = folderpath+"\\"+foldername+"_VFs.csv";
        FileStream stream = new FileStream(path, FileMode.Append);  
        using (StreamWriter writer = new StreamWriter(stream))  
        {  
            // TIME HEADER
            writer.Write("Time,");

            // VFs HEADERS
            if (activeConstraints.Contains(robot.GetComponent<TrajectoryGuidanceVF>())) {
                for (int j=0; j<robot.GetComponents<TrajectoryGuidanceVF>().Length; j++) {
                    writer.Write("TrajectoryGuidanceVF_forceX"+j+",");
                    writer.Write("TrajectoryGuidanceVF_forceY"+j+",");
                    writer.Write("TrajectoryGuidanceVF_forceZ"+j+",");
                    writer.Write("TrajectoryGuidanceVF_dist"+j+",");
                }
            }
            if (activeConstraints.Contains(robot.GetComponent<TrajectoryGuidanceVFRL>())) {
                for (int j=0; j<robot.GetComponents<TrajectoryGuidanceVFRL>().Length; j++) {
                    writer.Write("TrajectoryGuidanceVFRL_forceX"+j+",");
                    writer.Write("TrajectoryGuidanceVFRL_forceY"+j+",");
                    writer.Write("TrajectoryGuidanceVFRL_forceZ"+j+",");
                    writer.Write("TrajectoryGuidanceVFRL_dist"+j+",");
                    writer.Write("TrajectoryGuidanceVFRL_hand"+j+",");
                    writer.Write("TrajectoryGuidanceVFRL_miss"+j+",");
                }
            }
            if (activeConstraints.Contains(robot.GetComponent<TrajectoryOrientationGuidanceVF>())) {
                for (int j=0; j<robot.GetComponents<TrajectoryOrientationGuidanceVF>().Length; j++) {
                    writer.Write("TrajectoryOrientationGuidanceVF_forceX"+j+",");
                    writer.Write("TrajectoryOrientationGuidanceVF_forceY"+j+",");
                    writer.Write("TrajectoryOrientationGuidanceVF_forceZ"+j+",");
                    writer.Write("TrajectoryOrientationGuidanceVF_torqueX"+j+",");
                    writer.Write("TrajectoryOrientationGuidanceVF_torqueY"+j+",");
                    writer.Write("TrajectoryOrientationGuidanceVF_torqueZ"+j+",");
                    writer.Write("TrajectoryOrientationGuidanceVF_dist"+j+",");
                    writer.Write("TrajectoryOrientationGuidanceVF_angle"+j+",");
                }
            }
            if (activeConstraints.Contains(robot.GetComponent<TrajectoryOrientationGuidanceVFRL>())) {
                for (int j=0; j<robot.GetComponents<TrajectoryOrientationGuidanceVFRL>().Length; j++) {
                    writer.Write("TrajectoryOrientationGuidanceVFRL_forceX"+j+",");
                    writer.Write("TrajectoryOrientationGuidanceVFRL_forceY"+j+",");
                    writer.Write("TrajectoryOrientationGuidanceVFRL_forceZ"+j+",");
                    writer.Write("TrajectoryOrientationGuidanceVFRL_torqueX"+j+",");
                    writer.Write("TrajectoryOrientationGuidanceVFRL_torqueY"+j+",");
                    writer.Write("TrajectoryOrientationGuidanceVFRL_torqueZ"+j+",");
                    writer.Write("TrajectoryOrientationGuidanceVFRL_dist"+j+",");
                    writer.Write("TrajectoryOrientationGuidanceVFRL_angle"+j+",");
                    writer.Write("TrajectoryOrientationGuidanceVFRL_hand"+j+",");
                    writer.Write("TrajectoryOrientationGuidanceVFRL_miss"+j+",");
                }
            }
            if (activeConstraints.Contains(robot.GetComponent<ObstacleAvoidanceForceFieldVF>())) {
                for (int j=0; j<robot.GetComponents<ObstacleAvoidanceForceFieldVF>().Length; j++) {
                    writer.Write("ObstacleAvoidanceForceFieldVF_forceX"+j+",");
                    writer.Write("ObstacleAvoidanceForceFieldVF_forceY"+j+",");
                    writer.Write("ObstacleAvoidanceForceFieldVF_forceZ"+j+",");
                    writer.Write("ObstacleAvoidanceForceFieldVF_dist"+j+",");
                }
            }
            if (activeConstraints.Contains(robot.GetComponent<SurfaceAvoidanceVF>())) {
                for (int j=0; j<robot.GetComponents<SurfaceAvoidanceVF>().Length; j++) {
                    writer.Write("SurfaceAvoidanceVF_forceX"+j+",");
                    writer.Write("SurfaceAvoidanceVF_forceY"+j+",");
                    writer.Write("SurfaceAvoidanceVF_forceZ"+j+",");
                    writer.Write("SurfaceAvoidanceVF_dist"+j+",");
                }
            }
            if (activeConstraints.Contains(robot.GetComponent<SurfaceGuidanceVF>())) {
                for (int j=0; j<robot.GetComponents<SurfaceGuidanceVF>().Length; j++) {
                    writer.Write("SurfaceGuidanceVF_forceX"+j+",");
                    writer.Write("SurfaceGuidanceVF_forceY"+j+",");
                    writer.Write("SurfaceGuidanceVF_forceZ"+j+",");
                    writer.Write("SurfaceGuidanceVF_dist"+j+",");
                }
            }
            if (activeConstraints.Contains(robot.GetComponent<SurfaceOrientationGuidanceVF>())) {
                for (int j=0; j<robot.GetComponents<SurfaceOrientationGuidanceVF>().Length; j++) {
                    writer.Write("SurfaceOrientationGuidanceVF_forceX"+j+",");
                    writer.Write("SurfaceOrientationGuidanceVF_forceY"+j+",");
                    writer.Write("SurfaceOrientationGuidanceVF_forceZ"+j+",");
                    writer.Write("SurfaceOrientationGuidanceVF_torqueX"+j+",");
                    writer.Write("SurfaceOrientationGuidanceVF_torqueY"+j+",");
                    writer.Write("SurfaceOrientationGuidanceVF_torqueZ"+j+",");
                    writer.Write("SurfaceOrientationGuidanceVF_dist"+j+",");
                    writer.Write("SurfaceOrientationGuidanceVF_angle"+j+",");
                }
            }
            if (activeConstraints.Contains(robot.GetComponent<ConeApproachGuidanceVF>())) {
                for (int j=0; j<robot.GetComponents<ConeApproachGuidanceVF>().Length; j++) {
                    writer.Write("ConeApproachGuidanceVF_forceX"+j+",");
                    writer.Write("ConeApproachGuidanceVF_forceY"+j+",");
                    writer.Write("ConeApproachGuidanceVF_forceZ"+j+",");
                    writer.Write("ConeApproachGuidanceVF_dist"+j+",");
                }
            }
            // Assistance YES/NO
            writer.Write("Assisted,");
            // Assistance Factor
            writer.Write("Assistance,");
            
            // Subject Transform Headers
            writer.Write("PositionX,PositionY,PositionZ,");
            writer.Write("ForwardX,ForwardY,ForwardZ,");

            // Clutch pressed YES/NO
            writer.Write("Clutch,");

            //Distance from camera
            writer.Write("FromCamera");
            
            writer.Write("\n");
        }  

    }

    void Update()
    {
        FileStream stream = new FileStream(path, FileMode.Append);  
        using (StreamWriter writer = new StreamWriter(stream))  
        {  
            // Writes the current time since startup
            writer.Write(Time.realtimeSinceStartup); writer.Write(",");

            if (subject == null) {
                subject = GameObject.Find(Global.tooltip_path).transform;
            }

            Vector3 f;
            Vector3 t;
            if (activeConstraints.Contains(robot.GetComponent<TrajectoryGuidanceVF>())) {
                subject = robot.GetComponent<TrajectoryGuidanceVF>().subject;
                for (int j=0; j<robot.GetComponents<TrajectoryGuidanceVF>().Length; j++) {
                    f = robot.GetComponents<TrajectoryGuidanceVF>()[j].force;
                    writer.Write(f.x); writer.Write(","); 
                    writer.Write(f.y); writer.Write(","); 
                    writer.Write(f.z); writer.Write(","); 
                    writer.Write(robot.GetComponents<TrajectoryGuidanceVF>()[j].distance);writer.Write(",");
                }
            }
            if (activeConstraints.Contains(robot.GetComponent<TrajectoryGuidanceVFRL>())) {
                subject = robot.GetComponent<TrajectoryGuidanceVFRL>().subject;
                for (int j=0; j<robot.GetComponents<TrajectoryGuidanceVFRL>().Length; j++) {
                    f = robot.GetComponents<TrajectoryGuidanceVFRL>()[j].force;
                    writer.Write(f.x); writer.Write(","); 
                    writer.Write(f.y); writer.Write(","); 
                    writer.Write(f.z); writer.Write(","); 
                    writer.Write(robot.GetComponents<TrajectoryGuidanceVFRL>()[j].distance);writer.Write(",");
                    if (robot.GetComponents<TrajectoryGuidanceVFRL>()[j].left) {
                        writer.Write("left");writer.Write(",");
                    } else {                                                      
                        writer.Write("right");writer.Write(",");
                    } 
                    writer.Write(robot.GetComponents<TrajectoryGuidanceVFRL>()[j].missExchanges);writer.Write(",");
                }
            }
            if (activeConstraints.Contains(robot.GetComponent<TrajectoryOrientationGuidanceVF>())) {
                subject = robot.GetComponent<TrajectoryOrientationGuidanceVF>().subject;
                for (int j=0; j<robot.GetComponents<TrajectoryOrientationGuidanceVF>().Length; j++) {
                    f = robot.GetComponents<TrajectoryOrientationGuidanceVF>()[j].force;
                    writer.Write(f.x); writer.Write(","); 
                    writer.Write(f.y); writer.Write(","); 
                    writer.Write(f.z); writer.Write(","); 
                    t = robot.GetComponents<TrajectoryOrientationGuidanceVF>()[j].torque;
                    writer.Write(t.x); writer.Write(","); 
                    writer.Write(t.y); writer.Write(","); 
                    writer.Write(t.z); writer.Write(","); 
                    writer.Write(robot.GetComponents<TrajectoryOrientationGuidanceVF>()[j].distance);writer.Write(",");
                    writer.Write(robot.GetComponents<TrajectoryOrientationGuidanceVF>()[j].angle);writer.Write(",");
                }
            }
            if (activeConstraints.Contains(robot.GetComponent<TrajectoryOrientationGuidanceVFRL>())) {
                subject = robot.GetComponent<TrajectoryOrientationGuidanceVFRL>().subject;
                for (int j=0; j<robot.GetComponents<TrajectoryOrientationGuidanceVFRL>().Length; j++) {
                    f = robot.GetComponents<TrajectoryOrientationGuidanceVFRL>()[j].force;
                    writer.Write(f.x); writer.Write(","); 
                    writer.Write(f.y); writer.Write(","); 
                    writer.Write(f.z); writer.Write(","); 
                    t = robot.GetComponents<TrajectoryOrientationGuidanceVFRL>()[j].torque;
                    writer.Write(t.x); writer.Write(","); 
                    writer.Write(t.y); writer.Write(","); 
                    writer.Write(t.z); writer.Write(","); 
                    writer.Write(robot.GetComponents<TrajectoryOrientationGuidanceVFRL>()[j].distance);writer.Write(",");
                    writer.Write(robot.GetComponents<TrajectoryOrientationGuidanceVFRL>()[j].angle);writer.Write(",");
                    if (robot.GetComponents<TrajectoryOrientationGuidanceVFRL>()[j].left) {
                        writer.Write("left");writer.Write(",");
                    } else {                                                                 
                        writer.Write("right");writer.Write(",");
                    }
                    writer.Write(robot.GetComponents<TrajectoryOrientationGuidanceVFRL>()[j].missExchanges);writer.Write(",");
                }
            }
            if (activeConstraints.Contains(robot.GetComponent<ObstacleAvoidanceForceFieldVF>())) {
                subject = robot.GetComponent<ObstacleAvoidanceForceFieldVF>().subject;
                for (int j=0; j<robot.GetComponents<ObstacleAvoidanceForceFieldVF>().Length; j++) {
                    f = robot.GetComponents<ObstacleAvoidanceForceFieldVF>()[j].force;
                    writer.Write(f.x); writer.Write(","); 
                    writer.Write(f.y); writer.Write(","); 
                    writer.Write(f.z); writer.Write(","); 
                    writer.Write(robot.GetComponents<ObstacleAvoidanceForceFieldVF>()[j].distance);writer.Write(",");
                }
            }
            if (activeConstraints.Contains(robot.GetComponent<SurfaceAvoidanceVF>())) {
                subject = robot.GetComponent<SurfaceAvoidanceVF>().subject;
                for (int j=0; j<robot.GetComponents<SurfaceAvoidanceVF>().Length; j++) {
                    f = robot.GetComponents<SurfaceAvoidanceVF>()[j].force;
                    writer.Write(f.x); writer.Write(","); 
                    writer.Write(f.y); writer.Write(","); 
                    writer.Write(f.z); writer.Write(","); 
                    writer.Write(robot.GetComponents<SurfaceAvoidanceVF>()[j].distance);writer.Write(",");
                }
            }
            if (activeConstraints.Contains(robot.GetComponent<SurfaceGuidanceVF>())) {
                subject = robot.GetComponent<SurfaceGuidanceVF>().subject;
                for (int j=0; j<robot.GetComponents<SurfaceGuidanceVF>().Length; j++) {
                    f = robot.GetComponents<SurfaceGuidanceVF>()[j].force;
                    writer.Write(f.x); writer.Write(","); 
                    writer.Write(f.y); writer.Write(","); 
                    writer.Write(f.z); writer.Write(","); 
                    writer.Write(robot.GetComponents<SurfaceGuidanceVF>()[j].distance);writer.Write(",");
                }
            }
            if (activeConstraints.Contains(robot.GetComponent<SurfaceOrientationGuidanceVF>())) {
                subject = robot.GetComponent<SurfaceOrientationGuidanceVF>().subject;
                for (int j=0; j<robot.GetComponents<SurfaceOrientationGuidanceVF>().Length; j++) {
                    f = robot.GetComponents<SurfaceOrientationGuidanceVF>()[j].force;
                    writer.Write(f.x); writer.Write(","); 
                    writer.Write(f.y); writer.Write(","); 
                    writer.Write(f.z); writer.Write(","); 
                    t = robot.GetComponents<SurfaceOrientationGuidanceVF>()[j].torque;
                    writer.Write(t.x); writer.Write(","); 
                    writer.Write(t.y); writer.Write(","); 
                    writer.Write(t.z); writer.Write(","); 
                    writer.Write(robot.GetComponents<SurfaceOrientationGuidanceVF>()[j].distance);writer.Write(",");
                    writer.Write(robot.GetComponents<SurfaceOrientationGuidanceVF>()[j].angle);writer.Write(",");
                }
            }
            if (activeConstraints.Contains(robot.GetComponent<ConeApproachGuidanceVF>())) {
                subject = robot.GetComponent<ConeApproachGuidanceVF>().subject;
                for (int j=0; j<robot.GetComponents<ConeApproachGuidanceVF>().Length; j++) {
                    f = robot.GetComponents<ConeApproachGuidanceVF>()[j].force;
                    writer.Write(f.x); writer.Write(","); 
                    writer.Write(f.y); writer.Write(","); 
                    writer.Write(f.z); writer.Write(","); 
                    writer.Write(robot.GetComponents<ConeApproachGuidanceVF>()[j].distance);writer.Write(",");
                }
            }
            
            // Assistance ON or OFF
            if (robot.GetComponent<SumForces>() != null) {
                writer.Write(robot.GetComponent<SumForces>().enabled);
            } else if (robot.GetComponent<SumForcesRL>() != null) {
                writer.Write(robot.GetComponent<SumForcesRL>().enabled);
            }
            writer.Write(",");

            // Assistance factor
            writer.Write(Global.assistance); writer.Write(",");


            // Subject Transform Data
            writer.Write(subject.position.x); writer.Write(",");
            writer.Write(subject.position.y); writer.Write(",");
            writer.Write(subject.position.z); writer.Write(",");

            writer.Write(subject.forward.x); writer.Write(",");
            writer.Write(subject.forward.y); writer.Write(",");
            writer.Write(subject.forward.z); writer.Write(",");

            // Clutch pressed YES or NO
            writer.Write(gameObject.GetComponent<PedalClutchSubscriber>().pressed); writer.Write(",");

            writer.Write(Vector3.Distance(subject.position, 
                GameObject.Find("Camera").transform.position));
            // Line End
            writer.Write("\n");
        }  
    }
}
