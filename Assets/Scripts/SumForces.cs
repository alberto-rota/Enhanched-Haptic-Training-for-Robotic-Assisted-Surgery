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

[ExecuteInEditMode]
public class SumForces : MonoBehaviour
{
    public List<MonoBehaviour> activeConstraints;
    public Vector3 totalForce = Vector3.zero;
    public float totalForceMagnitude;
    public bool graphics = true;
    Vector3 PrevPos; 
    Vector3 NewPos; 
    public Queue<Vector3> vf = new Queue<Vector3>();
    int filtersize = 10;

    [Range(0,5f)]
    public float minForce = 0f;
    [Range(0,5f)]
    public float maxForce = 3f;
    [Range(0,5)]
    public float graphicVectorGain = 1;

    void Start()
    {
        activeConstraints = new List<MonoBehaviour>();
        foreach (MonoBehaviour s in gameObject.GetComponents<MonoBehaviour>()) {
            if ((s.GetType().Name == "TrajectoryGuidanceVF" ||
                s.GetType().Name == "ObstacleAvoidanceForceFieldVF" ||
                s.GetType().Name == "SurfaceAvoidanceVF" ||
                s.GetType().Name == "SurfaceGuidanceVF" ||
                s.GetType().Name == "ConeApproachGuidanceVF" )
                && s.enabled == true) {
                activeConstraints.Add(s);
            }
        }   
        PrevPos = GameObject.Find(Global.tooltip_path).transform.position;
        NewPos = GameObject.Find(Global.tooltip_path).transform.position;

        for (int i = 0; i<filtersize; i++) {
            vf.Enqueue(Vector3.zero);
        }

    }

    void FixedUpdate()
    {
        totalForce = Vector3.zero;
        if (activeConstraints.Contains(gameObject.GetComponent<TrajectoryGuidanceVF>())) {
            foreach (TrajectoryGuidanceVF vf in gameObject.GetComponents<TrajectoryGuidanceVF>() ) {
                if (!(float.IsNaN(vf.force.x) || float.IsNaN(vf.force.y) || float.IsNaN(vf.force.z))) totalForce = totalForce + vf.force;
            }
        }
        if (activeConstraints.Contains(gameObject.GetComponent<ObstacleAvoidanceForceFieldVF>())) {
            foreach (ObstacleAvoidanceForceFieldVF vf in gameObject.GetComponents<ObstacleAvoidanceForceFieldVF>() ) {
                if (!(float.IsNaN(vf.force.x) || float.IsNaN(vf.force.y) || float.IsNaN(vf.force.z))) totalForce = totalForce + vf.force;
            }
        }
        if (activeConstraints.Contains(gameObject.GetComponent<SurfaceAvoidanceVF>())) {
            foreach (SurfaceAvoidanceVF vf in gameObject.GetComponents<SurfaceAvoidanceVF>() ) {
                if (!(float.IsNaN(vf.force.x) || float.IsNaN(vf.force.y) || float.IsNaN(vf.force.z))) totalForce = totalForce + vf.force;
            }
        }
        if (activeConstraints.Contains(gameObject.GetComponent<SurfaceGuidanceVF>())) {
            foreach (SurfaceGuidanceVF vf in gameObject.GetComponents<SurfaceGuidanceVF>() ) {
                if (!(float.IsNaN(vf.force.x) || float.IsNaN(vf.force.y) || float.IsNaN(vf.force.z))) totalForce = totalForce + vf.force;
            }
        }
        if (activeConstraints.Contains(gameObject.GetComponent<ConeApproachGuidanceVF>())) {
            foreach (ConeApproachGuidanceVF vf in gameObject.GetComponents<ConeApproachGuidanceVF>() ) {
                if (!(float.IsNaN(vf.force.x) || float.IsNaN(vf.force.y) || float.IsNaN(vf.force.z))) totalForce = totalForce + vf.force;
            }
        }
        totalForceMagnitude  = totalForce.magnitude;
        if (graphics) {
            Global.Arrow(GameObject.Find(Global.tooltip_path).transform.position, GameObject.Find(Global.tooltip_path).transform.position+totalForce*graphicVectorGain, Color.white);
        }
        //force applied only if big enough
        if (totalForceMagnitude < minForce) {
            totalForce = Vector3.zero;
        }

        if (totalForceMagnitude > maxForce) {
            totalForce = totalForce.normalized * maxForce;
        }
        totalForceMagnitude  = totalForce.magnitude;
        //Adds a damp contribution
        NewPos = GameObject.Find(Global.tooltip_path).transform.position;  

        Vector3 velocity = (NewPos - PrevPos) / Time.fixedDeltaTime;  
        vf.Enqueue(velocity);  
        vf.Dequeue();  

        Vector3 smoothVelocity = Vector3.zero;  
        foreach(Vector3 v in vf) {
            smoothVelocity+=v/filtersize;
        }
        PrevPos = NewPos;  

        if (totalForceMagnitude > 0) {
            totalForce = totalForce + (-smoothVelocity * 1f);
        }
        Debug.Log(velocity.magnitude);
        Global.Arrow(GameObject.Find(Global.tooltip_path).transform.position, GameObject.Find(Global.tooltip_path).transform.position+smoothVelocity, Color.green);
    }
    
}