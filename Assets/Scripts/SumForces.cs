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
    [Range(0,5)]
    public float graphicVectorGain = 1;

    [Header("Damp")]
    [Range(0,10f)]
    public float damp = 1.5f;

    [Header("Output")]
    public Vector3 totalForce = Vector3.zero;
    public float totalForceMagnitude;

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

        totalForce+=GameObject.Find(Global.tooltip_path).GetComponent<Velocity>().velocity*damp;
        totalForceMagnitude  = totalForce.magnitude;

        if (totalForceMagnitude < minForce) {
            totalForce = Vector3.zero;
        }
        if (totalForceMagnitude > maxForce) {
            totalForce = totalForce.normalized * maxForce;
        }
        totalForceMagnitude  = totalForce.magnitude;

    }
    
}