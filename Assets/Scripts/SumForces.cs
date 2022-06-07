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
    public bool graphics = true;
    
    [Range(0,5f)]
    public float forceThreshold = 0.1f;
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

    }

    void Update()
    {
        totalForce = Vector3.zero;
        if (activeConstraints.Contains(gameObject.GetComponent<TrajectoryGuidanceVF>())) {
            totalForce = totalForce + gameObject.GetComponent<TrajectoryGuidanceVF>().force;   
        }
        if (activeConstraints.Contains(gameObject.GetComponent<ObstacleAvoidanceForceFieldVF>())) {
            totalForce = totalForce + gameObject.GetComponent<ObstacleAvoidanceForceFieldVF>().force;   
        }
        if (activeConstraints.Contains(gameObject.GetComponent<SurfaceAvoidanceVF>())) {
            totalForce = totalForce + gameObject.GetComponent<SurfaceAvoidanceVF>().force;   
        }
        if (activeConstraints.Contains(gameObject.GetComponent<SurfaceGuidanceVF>())) {
            totalForce = totalForce + gameObject.GetComponent<SurfaceGuidanceVF>().force;   
        }
        if (activeConstraints.Contains(gameObject.GetComponent<ConeApproachGuidanceVF>())) {
            totalForce = totalForce + gameObject.GetComponent<ConeApproachGuidanceVF>().force;   
        }
        if (graphics) {
            Arrow(gameObject.transform.position, gameObject.transform.position+totalForce*graphicVectorGain, Color.white);
        }
        //force applied only if big enough
        if (totalForce.magnitude > forceThreshold) {
            gameObject.GetComponent<Rigidbody>().AddForce(totalForce);
        }
    }
    void Arrow(Vector3 from, Vector3 to, Color color) {
        int coneResolution=30;
        float deltaTheta = 360f/coneResolution;

        Vector3 stem = (to-from)*0.9f;
        Vector3 tip = to-(from+stem)*0.1f;
        float tipradius = 0.1f*(to-from).magnitude;
        List<Vector3> tipBasePoints = new List<Vector3>();
        Vector3 b = Vector3.Cross(tip, Vector3.up)*tipradius;
        tipBasePoints.Add(b);

        for (int i=0; i<coneResolution-1; i++) {
            float theta = deltaTheta*i; 
            b = Quaternion.AngleAxis(deltaTheta,tip.normalized)*b;
            tipBasePoints.Add(b);
        }
        Vector3 tipcenter = from+stem;
        //DRAWING THE STEM
        Debug.DrawLine(from,tipcenter, color);
        // DRAWING THE TIP
        for (int i=0; i<coneResolution; i++) {
            Debug.DrawLine(tipcenter+tipBasePoints[i],to, color);
            if (i==coneResolution-1) Debug.DrawLine(tipcenter+tipBasePoints[i],tipcenter+tipBasePoints[0],color);
            else Debug.DrawLine(tipcenter+tipBasePoints[i],tipcenter+tipBasePoints[i+1],color);
        }
    }
}