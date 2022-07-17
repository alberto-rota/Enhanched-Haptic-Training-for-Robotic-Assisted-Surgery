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
    // Draw draw = new Draw();

    public bool configuring;
    public Transform target;
    Vector3 coneCenter;
    [Range(0,30)]
    public float coneApertureDegrees = 10;
    [Range(0,10f)]
    public float dampInsideOfCone = 0.02f; 
    [Range(0,10f)]
    public float gainOutsideOfCone = 0.02f; 
    public Vector3 delta;
    List<Vector3> coneBasePoints;
    public float maxForce = 3;
    public Vector3 force;
    Vector3 tp;
    Vector3 ee;
    Vector3 prevl;
    Vector3 preva;
    Vector3 velocity;
    public bool graphics = true;
    [Range(0,5)]
    public float graphicVectorGain = 1;
    Color coneColor;
    public Color color = Color.yellow;
    public int coneResolution = 10;


    void Start(){
        configuring = false;
        prevl = Vector3.zero;
        preva = Vector3.zero;
        force = Vector3.zero;
        coneColor = color;

        float deltaTheta = 360f/coneResolution;
        delta = target.position-GameObject.Find(Global.tooltip_path).transform.position;  
        coneBasePoints = new List<Vector3>();
        Vector3 b = Vector3.Cross(delta.normalized, Vector3.up)*delta.magnitude*Mathf.Tan(coneApertureDegrees*Mathf.PI/180f);
        coneBasePoints.Add(b);
        for (int i=0; i<coneResolution-1; i++) {
            float theta = deltaTheta*i; 
            b = Quaternion.AngleAxis(deltaTheta,delta.normalized)*b;
            coneBasePoints.Add(b);
        }
        coneCenter = GameObject.Find(Global.tooltip_path).transform.position;
        tp = target.position;
    }

    void Update()
    {   
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
            delta = target.position-GameObject.Find(Global.tooltip_path).transform.position;  
            coneBasePoints = new List<Vector3>();
            Vector3 b = Vector3.Cross(delta.normalized, Vector3.up)*delta.magnitude*Mathf.Tan(coneApertureDegrees*Mathf.PI/180f);
            coneBasePoints.Add(b);
            for (int i=0; i<coneResolution-1; i++) {
                float theta = deltaTheta*i; 
                b = Quaternion.AngleAxis(deltaTheta,delta.normalized)*b;
                coneBasePoints.Add(b);
            }
            coneCenter = GameObject.Find(Global.tooltip_path).transform.position;

        } else { // THE CONE IS ASSUMED SELECTED AND STATIONARY, THE VF IS APPLIED
            ee = GameObject.Find(Global.tooltip_path).transform.position;
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

            if (a.magnitude < aperture*l.magnitude) { // INSIDE OF THE CONE,
                f_mag = dampInsideOfCone;
                force = (f_mag*-1*va);
            } else if (a.magnitude < aperture*l.magnitude+eps) { // OUTSIDE OF THE CONE, IN THE SWITCHING REGION
                f_mag = gainOutsideOfCone/eps*(a.magnitude-(aperture*l.magnitude-eps));
                force = (f_mag*-1*a.normalized);
            } else { // Outside of cone
                f_mag = gainOutsideOfCone;  
                force = (f_mag*-1*a.normalized);
            }

            // Rescaling the force the max magnitude if exceeded
            if (force.magnitude > maxForce) {
                force = force.normalized * maxForce;
            }
            // When close enough to the target, force is reduced (the cone is too nGlobal.Arrow)
            if (Vector3.Distance(ee,tp)/delta.magnitude < 0.2f) {
                force = force*0.2f;
                coneColor = Color.green;
            }

            // Debug.DrawLine(gameObject.transform.position, gameObject.transform.position+vl*graphicVectorGain, Color.magenta);
            // Debug.DrawLine(gameObject.transform.position+vl*graphicVectorGain, gameObject.transform.position+(vl+va)*graphicVectorGain, Color.magenta);
            // Debug.DrawLine(gameObject.transform.position, gameObject.transform.position+(vl+va)*graphicVectorGain, Color.magenta);
            if (graphics) {
                Debug.DrawLine(tp,tp+l,Color.white);
                Debug.DrawLine(tp+l,tp+l+a,Color.white);
            }
        }

        // DRAWING THE CONE
        for (int i=0; i<coneResolution; i++) {
            Debug.DrawLine(coneCenter+coneBasePoints[i],tp,coneColor);
            if (i==coneResolution-1) Debug.DrawLine(coneCenter+coneBasePoints[i],coneCenter+coneBasePoints[0],coneColor);
            else Debug.DrawLine(coneCenter+coneBasePoints[i],coneCenter+coneBasePoints[i+1],coneColor);
        }

        if (graphics) {
            Debug.DrawLine(ee, ee+vl*graphicVectorGain, Color.green);
            Debug.DrawLine(ee, ee+va*graphicVectorGain, Color.green);
            Global.Arrow(ee, ee+v*graphicVectorGain, Color.green);
            Debug.DrawLine(ee, ee+force*graphicVectorGain, Color.blue);
        }
    }
    
}
