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

    [Header("Bounds")]
    [Range(0,5f)]
    public float minForce = 0f;
    [Range(0,5f)]
    public float maxForce = 3f;
    [Range(0,0.0001f)]
    public float minTorque = 0f;
    [Range(0,0.1f)]
    public float maxTorque = 0.1f;

    [Header("Damp")]
    [Range(0,10f)]
    public static float damp = 0f;

    [Header("Output")]
    public Vector3 totalForce = Vector3.zero;
    public float totalForceMagnitude;
    public Vector3 totalTorque = Vector3.zero;
    public float totalTorqueMagnitude;

    void Start()
    {
        activeConstraints = new List<MonoBehaviour>();
        foreach (MonoBehaviour s in gameObject.GetComponents<MonoBehaviour>()) {
            if ((s.GetType().Name == "TrajectoryGuidanceVF" ||
                s.GetType().Name == "TrajectoryOrientationGuidanceVF" ||
                s.GetType().Name == "ObstacleAvoidanceForceFieldVF" ||
                s.GetType().Name == "SurfaceAvoidanceVF" ||
                s.GetType().Name == "SurfaceGuidanceVF" ||
                s.GetType().Name == "SurfaceOrientationGuidanceVF" ||
                s.GetType().Name == "ConeApproachGuidanceVF" )
                && s.enabled == true) {
                activeConstraints.Add(s);
            }
        }
    }

    void FixedUpdate()
    {
        totalForce = Vector3.zero;
        totalTorque = Vector3.zero;
        if (activeConstraints.Contains(gameObject.GetComponent<TrajectoryGuidanceVF>())) {
            foreach (TrajectoryGuidanceVF vf in gameObject.GetComponents<TrajectoryGuidanceVF>() ) {
                if (!(float.IsNaN(vf.force.x) || float.IsNaN(vf.force.y) || float.IsNaN(vf.force.z))) totalForce = totalForce + vf.force;
            }
        }
        if (activeConstraints.Contains(gameObject.GetComponent<TrajectoryOrientationGuidanceVF>())) {
            foreach (TrajectoryOrientationGuidanceVF vf in gameObject.GetComponents<TrajectoryOrientationGuidanceVF>() ) {
                if (!(float.IsNaN(vf.force.x) || float.IsNaN(vf.force.y) || float.IsNaN(vf.force.z))) totalForce = totalForce + vf.force;
                if (!(float.IsNaN(vf.torque.x) || float.IsNaN(vf.torque.y) || float.IsNaN(vf.torque.z))) totalTorque = totalTorque + vf.torque;
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
        if (activeConstraints.Contains(gameObject.GetComponent<SurfaceOrientationGuidanceVF>())) {
            foreach (SurfaceOrientationGuidanceVF vf in gameObject.GetComponents<SurfaceOrientationGuidanceVF>() ) {
                if (!(float.IsNaN(vf.force.x) || float.IsNaN(vf.force.y) || float.IsNaN(vf.force.z))) totalForce = totalForce + vf.force;
                if (!(float.IsNaN(vf.torque.x) || float.IsNaN(vf.torque.y) || float.IsNaN(vf.torque.z))) totalTorque = totalTorque + vf.torque;
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
        if (totalForceMagnitude > 0) {
            totalForce-=GameObject.Find(Global.tooltip_path).GetComponent<Velocity>().velocity*damp;
        }

        if (totalForceMagnitude < minForce) {
            totalForce = Vector3.zero;
        }
        if (totalForceMagnitude > maxForce) {
            totalForce = totalForce.normalized * maxForce;
        }
        totalForceMagnitude  = totalForce.magnitude;

        totalTorqueMagnitude  = totalTorque.magnitude;
        if (totalTorqueMagnitude < minTorque) {
            totalTorque = Vector3.zero;
        }
        if (totalTorqueMagnitude > maxTorque) {
            totalTorque = totalTorque.normalized * maxTorque;
        }
        totalTorqueMagnitude  = totalTorque.magnitude;

        GameObject.Find("/Text/CanvasBarL/Bar").transform.localScale = new Vector3(5*totalForceMagnitude/maxForce+5*totalTorqueMagnitude/maxTorque,
            GameObject.Find("/Text/CanvasBarL/Bar").transform.localScale.y,GameObject.Find("/Text/CanvasBarL/Bar").transform.localScale.z);
        GameObject.Find("/Text/CanvasBar/Bar").transform.localScale = new Vector3(5*totalForceMagnitude/maxForce+5*totalTorqueMagnitude/maxTorque,
            GameObject.Find("/Text/CanvasBar/Bar").transform.localScale.y,GameObject.Find("/Text/CanvasBar/Bar").transform.localScale.z);
        GameObject.Find("/Text/CanvasBarL/Bar").GetComponent<UnityEngine.UI.Image>().color = Color.Lerp(Color.yellow, Color.red, totalForceMagnitude/maxForce);
        GameObject.Find("/Text/CanvasBar/Bar").GetComponent<UnityEngine.UI.Image>().color = Color.Lerp(Color.yellow, Color.red, totalForceMagnitude/maxForce);
    }

    
}