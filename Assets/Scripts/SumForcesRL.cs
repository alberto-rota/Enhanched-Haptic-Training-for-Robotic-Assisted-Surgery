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
public class SumForcesRL : MonoBehaviour
{
    public List<MonoBehaviour> activeConstraints;

    [Header("Bounds")]
    [Range(0,5f)]
    public float minForce = 0f;
    [Range(0,5f)]
    public float maxForce = 3f;

    [Header("Damp")]
    [Range(0,10f)]
    public static float damp = 0f;

    [Header("Output")]
    public Vector3 totalForceRight = Vector3.zero;
    public Vector3 totalForceLeft = Vector3.zero;
    public float totalForceMagnitudeRight;
    public float totalForceMagnitudeLeft;

    void Start()
    {
        activeConstraints = new List<MonoBehaviour>();
        foreach (MonoBehaviour s in gameObject.GetComponents<MonoBehaviour>()) {
            if ((s.GetType().Name == "TrajectoryGuidanceVFRL" ||
                s.GetType().Name == "ObstacleAvoidanceForceFieldVFRL" ||
                s.GetType().Name == "SurfaceAvoidanceVFRL" ||
                s.GetType().Name == "SurfaceGuidanceVFRL" ||
                s.GetType().Name == "ConeApproachGuidanceVFRL" )
                && s.enabled == true) {
                activeConstraints.Add(s);
            }
        }
    }

    void FixedUpdate()
    {
        totalForceRight = Vector3.zero;
        totalForceLeft = Vector3.zero;
        if (activeConstraints.Contains(gameObject.GetComponent<TrajectoryGuidanceVFRL>())) {
            foreach (TrajectoryGuidanceVFRL vf in gameObject.GetComponents<TrajectoryGuidanceVFRL>() ) {
                if (!(float.IsNaN(vf.force.x) || float.IsNaN(vf.force.y) || float.IsNaN(vf.force.z))) {
                    if (vf.left) totalForceLeft = totalForceLeft + vf.force;
                    else totalForceRight = totalForceRight + vf.force;
                }
            }
        }
        // if (activeConstraints.Contains(gameObject.GetComponent<ObstacleAvoidanceForceFieldVFRL>())) {
        //     foreach (ObstacleAvoidanceForceFieldVFRL vf in gameObject.GetComponents<ObstacleAvoidanceForceFieldVFRL>() ) {
        //         if (!(float.IsNaN(vf.force.x) || float.IsNaN(vf.force.y) || float.IsNaN(vf.force.z))) {
        //             if (vf.left) totalForceLeft = totalForceLeft + vf.force;
        //             else totalForceRight = totalForceRight + vf.force;
        //         }
        //     }
        // }
        // if (activeConstraints.Contains(gameObject.GetComponent<SurfaceAvoidanceVFRL>())) {
        //     foreach (SurfaceAvoidanceVFRL vf in gameObject.GetComponents<SurfaceAvoidanceVFRL>() ) {
        //         if (!(float.IsNaN(vf.force.x) || float.IsNaN(vf.force.y) || float.IsNaN(vf.force.z))) {
        //             if (vf.left) totalForceLeft = totalForceLeft + vf.force;
        //             else totalForceRight = totalForceRight + vf.force;
        //         }
        //     }
        // }
        // if (activeConstraints.Contains(gameObject.GetComponent<SurfaceGuidanceVFRL>())) {
        //     foreach (SurfaceGuidanceVFRL vf in gameObject.GetComponents<SurfaceGuidanceVFRL>() ) {
        //         if (!(float.IsNaN(vf.force.x) || float.IsNaN(vf.force.y) || float.IsNaN(vf.force.z))) {
        //             if (vf.left) totalForceLeft = totalForceLeft + vf.force;
        //             else totalForceRight = totalForceRight + vf.force;
        //         }
        //     }
        // }
        // if (activeConstraints.Contains(gameObject.GetComponent<ConeApproachGuidanceVFRL>())) {
        //     foreach (ConeApproachGuidanceVFRL vf in gameObject.GetComponents<ConeApproachGuidanceVFRL>() ) {
        //         if (!(float.IsNaN(vf.force.x) || float.IsNaN(vf.force.y) || float.IsNaN(vf.force.z))) {
        //             if (vf.left) totalForceLeft = totalForceLeft + vf.force;
        //             else totalForceRight = totalForceRight + vf.force;
        //         }
        //     }
        // }

        totalForceMagnitudeRight  = totalForceRight.magnitude;
        totalForceMagnitudeLeft  = totalForceLeft.magnitude;
        if (totalForceMagnitudeRight > 0) {
            totalForceRight-=GameObject.Find(Global.tooltip_path).GetComponent<Velocity>().velocity*damp;
        }

        if (totalForceMagnitudeRight < minForce) {
            totalForceRight = Vector3.zero;
        }
        if (totalForceMagnitudeRight > maxForce) {
            totalForceRight = totalForceRight.normalized * maxForce;
        }
        totalForceMagnitudeRight  = totalForceRight.magnitude;

        if (totalForceMagnitudeLeft > 0) {
            totalForceLeft-=GameObject.Find(Global.tooltip_path2).GetComponent<Velocity>().velocity*damp;
        }

        if (totalForceMagnitudeLeft < minForce) {
            totalForceLeft = Vector3.zero;
        }
        if (totalForceMagnitudeLeft > maxForce) {
            totalForceLeft = totalForceLeft.normalized * maxForce;
        }
        totalForceMagnitudeLeft  = totalForceLeft.magnitude;
    }
    
}