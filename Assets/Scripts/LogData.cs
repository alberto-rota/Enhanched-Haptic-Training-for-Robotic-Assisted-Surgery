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


public class LogData : MonoBehaviour
{

    public List<MonoBehaviour> activeConstraints;
    public string saveTo = @"C:\Users\alber\Desktop\MSc_Thesis\Active Constraints\Assets\Logs";
    string path;

    string GetUniqueName(string name, string folderPath) {
        string validatedName = folderPath+"\\"+name+"_0.csv";
        int tries = 0;
        while (File.Exists(validatedName)) {
            tries++;
            validatedName = folderPath+"\\"+name+"_"+tries.ToString()+".csv";
        }
        return validatedName;
    }

    void Start() {
            
        path = GetUniqueName(SceneManager.GetActiveScene().name,saveTo);
        Debug.Log("Task data will be saved to: "+path);
        // Checks which VFs are activated and enabled
        activeConstraints = new List<MonoBehaviour>();
        foreach (MonoBehaviour s in gameObject.GetComponents<MonoBehaviour>()) {
            if (s.GetType().Name != "LogData" && s.enabled == true) {
                activeConstraints.Add(s);
            }
        }   

        // Opens the filestream
        FileStream stream = new FileStream(path, FileMode.Append);  
        using (StreamWriter writer = new StreamWriter(stream))  
        {  
            // POSITION HEADER
            writer.Write("PositionX,PositionY,PositionZ,");

            // VFs HEADERS
            if (activeConstraints.Contains(gameObject.GetComponent<TrajectoryGuidanceVF>())) {
                writer.Write("TrajectoryGuidanceVF_X,");
                writer.Write("TrajectoryGuidanceVF_Y,");
                writer.Write("TrajectoryGuidanceVF_Z,");
            }
            if (activeConstraints.Contains(gameObject.GetComponent<ObstacleAvoidanceForceFieldVF>())) {
                writer.Write("ObstacleAvoidanceForceFieldVF_X,");
                writer.Write("ObstacleAvoidanceForceFieldVF_Y,");
                writer.Write("ObstacleAvoidanceForceFieldVF_Z,");
            }
            if (activeConstraints.Contains(gameObject.GetComponent<SurfaceAvoidanceVF>())) {
                writer.Write("SurfaceAvoidanceVF_X,");
                writer.Write("SurfaceAvoidanceVF_Y,");
                writer.Write("SurfaceAvoidanceVF_Z,");
            }
            if (activeConstraints.Contains(gameObject.GetComponent<SurfaceGuidanceVF>())) {
                writer.Write("SurfaceGuidanceVF_X,");
                writer.Write("SurfaceGuidanceVF_Y,");
                writer.Write("SurfaceGuidanceVF_Z,");
            }
            if (activeConstraints.Contains(gameObject.GetComponent<ConeApproachGuidanceVF>())) {
                writer.Write("ConeApproachGuidanceVF_X,");
                writer.Write("ConeApproachGuidanceVF_Y,");
                writer.Write("ConeApproachGuidanceVF_Z,");
            }
            if (activeConstraints.Contains(gameObject.GetComponent<SumForces>())) {
                writer.Write("TotalForce_X,");
                writer.Write("TotalForce_Y,");
                writer.Write("TotalForce_Z,");
            }

            writer.Write("\n");
        }  
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
                f = gameObject.GetComponent<TrajectoryGuidanceVF>().force;
                writer.Write(f.x); writer.Write(","); 
                writer.Write(f.y); writer.Write(","); 
                writer.Write(f.z); writer.Write(","); 
            }
            if (activeConstraints.Contains(gameObject.GetComponent<ObstacleAvoidanceForceFieldVF>())) {
                f = gameObject.GetComponent<ObstacleAvoidanceForceFieldVF>().force;
                writer.Write(f.x); writer.Write(","); 
                writer.Write(f.y); writer.Write(","); 
                writer.Write(f.z); writer.Write(","); 
            }
            if (activeConstraints.Contains(gameObject.GetComponent<SurfaceAvoidanceVF>())) {
                f = gameObject.GetComponent<SurfaceAvoidanceVF>().force;
                writer.Write(f.x); writer.Write(","); 
                writer.Write(f.y); writer.Write(","); 
                writer.Write(f.z); writer.Write(","); 
            }
            if (activeConstraints.Contains(gameObject.GetComponent<SurfaceGuidanceVF>())) {
                f = gameObject.GetComponent<SurfaceGuidanceVF>().force;
                writer.Write(f.x); writer.Write(","); 
                writer.Write(f.y); writer.Write(","); 
                writer.Write(f.z); writer.Write(","); 
            }
            if (activeConstraints.Contains(gameObject.GetComponent<ConeApproachGuidanceVF>())) {
                f = gameObject.GetComponent<ConeApproachGuidanceVF>().force;
                writer.Write(f.x); writer.Write(","); 
                writer.Write(f.y); writer.Write(","); 
                writer.Write(f.z); writer.Write(","); 
            }
            if (activeConstraints.Contains(gameObject.GetComponent<SumForces>())) {
                f = gameObject.GetComponent<SumForces>().totalForce;
                writer.Write(f.x); writer.Write(","); 
                writer.Write(f.y); writer.Write(","); 
                writer.Write(f.z); writer.Write(","); 
            }
            
            writer.Write("\n");
        }  
    }
}
