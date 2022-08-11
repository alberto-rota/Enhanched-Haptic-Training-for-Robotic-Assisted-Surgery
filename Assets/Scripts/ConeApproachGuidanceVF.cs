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

//    IMPLEMENTED FROM = {Vision-assisted control for manipulation using virtual fixtures},
//    author = {Alessandro Bettini and Panadda Marayong and Samuel Lang and Allison M. Okamura and Gregory D. Hager},
//    year = {2004},


using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[ExecuteInEditMode]
public class ConeApproachGuidanceVF : MonoBehaviour
{
    [Header("Transforms")]
    public Transform subject;
    public Transform target;

    [Header("Virtual Fixture")]
    public bool configuring;
    [Range(0,30)]
    public float coneApertureDegrees = 10;
    [Range(0,5f)]
    public float gain = 3;
    [Range(0,0.2f)]
    public float half = 0.002f;
    [Range(0,10000f)]
    public float slope = 1f;

    public float relaxDistance = 8;

    [Header("Graphics")]
    public bool graphics = true;
    [Range(0,0.05f)]
    public float graphicVectorGain = 0.01f;
    public int coneResolution = 10;

    [Header("Output")]
    public Vector3 force;
    public float forceMagnitude;
    public float distance;
    public Vector3 delta;
    public Color color = Color.yellow;

    List<Vector3> coneBasePoints;
    Color coneColor;
    Vector3 coneCenter;
    Vector3 tp;
    Vector3 ee;
    Vector3 prevl;
    Vector3 preva;
    Vector3 velocity;


    void Start(){
        configuring = false;
        prevl = Vector3.zero;
        preva = Vector3.zero;
        force = Vector3.zero;
        coneColor = color;
        float deltaTheta = 360f/coneResolution;

        if (subject == null) {
            subject = GameObject.Find(Global.tooltip_path).transform;
        }

        delta = target.position-subject.position;  
        coneBasePoints = new List<Vector3>();
        Vector3 b = Vector3.Cross(delta.normalized, Vector3.up)*delta.magnitude*Mathf.Tan(coneApertureDegrees*Mathf.PI/180f);
        coneBasePoints.Add(b);
        for (int i=0; i<coneResolution-1; i++) {
            float theta = deltaTheta*i; 
            b = Quaternion.AngleAxis(deltaTheta,delta.normalized)*b;
            coneBasePoints.Add(b);
        }
        coneCenter = subject.position;
        tp = target.position;
    }

    void FixedUpdate()
    {   
        if (half < 0 ) {half=0;}
        if (slope < 1/half) {slope=1/half;}

        Vector3 v = GameObject.Find(Global.tooltip_path).GetComponent<Rigidbody>().velocity;
        Vector3 vl = Vector3.Dot(v,delta.normalized)*delta.normalized;
        Vector3 va = v-vl;
        if (target == null) {
            Debug.LogWarning("The Target must be assigned!");
            return;
        } 
        
        if (configuring) { // THE CONE MOVES WITH THE EE, NO VF APPLIED
            coneColor = color;
            float deltaTheta = 360f/coneResolution;
            delta = target.position-subject.position;  
            coneBasePoints = new List<Vector3>();
            Vector3 b = Vector3.Cross(delta.normalized, Vector3.up)*delta.magnitude*Mathf.Tan(coneApertureDegrees*Mathf.PI/180f);
            coneBasePoints.Add(b);
            for (int i=0; i<coneResolution-1; i++) {
                float theta = deltaTheta*i; 
                b = Quaternion.AngleAxis(deltaTheta,delta.normalized)*b;
                coneBasePoints.Add(b);
            }
            coneCenter = subject.position;

        } else { // THE CONE IS ASSUMED SELECTED AND STATIONARY, THE VF IS APPLIED
            ee = subject.position;
            Vector3 l = Vector3.Dot(ee-tp,delta.normalized)*delta.normalized;
            Vector3 a = ee-tp-l;

            float aperture = Mathf.Tan(coneApertureDegrees*Mathf.PI/180f);
            // Changing color the cone if outside of the border
            if (a.magnitude >= aperture*l.magnitude) {
                coneColor = Color.red;
            } else {
                coneColor = color; 
            }
            Vector3 eta = Vector3.Cross(delta,Vector3.Cross(delta,a));
            float f_mag;
            float eps = 0.05f*a.magnitude; // Switching region is 5% of the radial coordinate 

            distance = Global.DistMapAttraction(a.magnitude, aperture*l.magnitude, half, slope);

            f_mag = gain*distance;  
            force = (f_mag*-1*a.normalized);
            
            distance = a.magnitude;
            // When close enough to the target, force is reduced 
            if (Vector3.Distance(ee,tp)/delta.magnitude < relaxDistance/100) {
                force = force*0.2f;
                coneColor = Color.green;
            }
            forceMagnitude = force.magnitude;

        }

        // DRAWING THE CONE
        for (int i=0; i<coneResolution; i++) {
            Debug.DrawLine(coneCenter+coneBasePoints[i],tp,coneColor);
            if (i==coneResolution-1) Debug.DrawLine(coneCenter+coneBasePoints[i],coneCenter+coneBasePoints[0],coneColor);
            else Debug.DrawLine(coneCenter+coneBasePoints[i],coneCenter+coneBasePoints[i+1],coneColor);
        }

        if (graphics) {
            Global.Arrow(ee, ee+vl*graphicVectorGain, Color.green);
            Global.Arrow(ee, ee+va*graphicVectorGain, Color.green);
            // Global.Arrow(ee, ee+v*graphicVectorGain, Color.green);
            Global.Arrow(ee, ee+force*graphicVectorGain, Color.blue);
            Debug.DrawLine(tp,tp+vl,Color.white);
            Debug.DrawLine(tp+vl,tp+vl+va,Color.white);
        }
    }
    
}
