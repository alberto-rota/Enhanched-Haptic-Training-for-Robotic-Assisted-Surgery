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
    [Range(0,0.0001f)]
    public float minTorque = 0f;
    [Range(0,3f)]
    public float maxTorque = 0.01f;

    [Header("Damp")]
    [Range(0,10f)]
    public static float damp = 0f;

    [Header("Output")]
    public Vector3 totalForceRight = Vector3.zero;
    public Vector3 totalForceLeft = Vector3.zero;
    public float totalForceMagnitudeRight;
    public float totalForceMagnitudeLeft;
    public Vector3 totalTorqueRight = Vector3.zero;
    public Vector3 totalTorqueLeft = Vector3.zero;
    public float totalTorqueMagnitudeRight;
    public float totalTorqueMagnitudeLeft;

    void Awake()
    {
        activeConstraints = new List<MonoBehaviour>();
        foreach (MonoBehaviour s in gameObject.GetComponents<MonoBehaviour>()) {
            if ((s.GetType().Name == "TrajectoryGuidanceVFRL" ||
                s.GetType().Name == "TrajectoryOrientationGuidanceVFRL" ||
                s.GetType().Name == "SurfaceOrientationGuidanceVFRL")
                && s.enabled == true) {
                activeConstraints.Add(s);
            }
        }
    }

    void FixedUpdate()
    {
        totalForceRight = Vector3.zero;
        totalForceLeft = Vector3.zero;        
        totalTorqueRight = Vector3.zero;
        totalTorqueLeft = Vector3.zero;
        if (activeConstraints.Contains(gameObject.GetComponent<TrajectoryGuidanceVFRL>())) {
            foreach (TrajectoryGuidanceVFRL vf in gameObject.GetComponents<TrajectoryGuidanceVFRL>() ) {
                if (!(float.IsNaN(vf.force.x) || float.IsNaN(vf.force.y) || float.IsNaN(vf.force.z))) {
                    if (vf.left) {
                        totalForceLeft = totalForceLeft + vf.force;                        
                        // totalTorqueLeft = totalTorqueLeft + vf.torque;
                    } else { 
                        totalForceRight = totalForceRight + vf.force;
                        // totalTorqueRight = totalTorqueRight + vf.torque;
                    }
                }
            }
        }
        if (activeConstraints.Contains(gameObject.GetComponent<TrajectoryOrientationGuidanceVFRL>())) {
            foreach (TrajectoryOrientationGuidanceVFRL vf in gameObject.GetComponents<TrajectoryOrientationGuidanceVFRL>() ) {
                if (!(float.IsNaN(vf.force.x) || float.IsNaN(vf.force.y) || float.IsNaN(vf.force.z))) {
                    if (vf.left) {
                        totalForceLeft = totalForceLeft + vf.force;                        
                        totalTorqueLeft = totalTorqueLeft + vf.torque;
                    } else { 
                        totalForceRight = totalForceRight + vf.force;
                        totalTorqueRight = totalTorqueRight + vf.torque;
                    }
                }
            }
        }


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

        if (totalTorqueMagnitudeLeft < minTorque) {
            totalTorqueLeft = Vector3.zero;
        }
        if (totalTorqueMagnitudeLeft > maxTorque) {
            totalTorqueLeft = totalTorqueLeft.normalized * maxTorque;
        }
        totalTorqueMagnitudeLeft  = totalTorqueLeft.magnitude;

        if (totalTorqueMagnitudeRight < minTorque) {
            totalTorqueRight = Vector3.zero;
        }
        if (totalTorqueMagnitudeRight > maxTorque) {
            totalTorqueRight = totalTorqueRight.normalized * maxTorque;
        }
        totalTorqueMagnitudeRight  = totalTorqueRight.magnitude;

        GameObject.Find("/Text/CanvasBarL/Bar").transform.localScale = new Vector3(5*totalForceMagnitudeLeft/maxForce+5*totalTorqueMagnitudeLeft/maxTorque+5*totalForceMagnitudeRight/maxForce+5*totalTorqueMagnitudeRight/maxTorque,
            GameObject.Find("/Text/CanvasBarL/Bar").transform.localScale.y,GameObject.Find("/Text/CanvasBarL/Bar").transform.localScale.z);
        GameObject.Find("/Text/CanvasBar/Bar").transform.localScale = new Vector3(5*totalForceMagnitudeLeft/maxForce+5*totalTorqueMagnitudeLeft/maxTorque+5*totalForceMagnitudeRight/maxForce+5*totalTorqueMagnitudeRight/maxTorque,
            GameObject.Find("/Text/CanvasBar/Bar").transform.localScale.y,GameObject.Find("/Text/CanvasBar/Bar").transform.localScale.z);
        GameObject.Find("/Text/CanvasBarL/Bar").GetComponent<UnityEngine.UI.Image>().color = Color.Lerp(Color.yellow, Color.red, (totalForceMagnitudeLeft+totalForceMagnitudeRight)/maxForce);
        GameObject.Find("/Text/CanvasBar/Bar").GetComponent<UnityEngine.UI.Image>().color = Color.Lerp(Color.yellow, Color.red, (totalForceMagnitudeLeft+totalForceMagnitudeRight)/maxForce);

         
    }
    
}