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

    public bool configuring = true;
    public Transform target;
    Vector3 coneCenter;
    [Range(1,30)]
    public float coneApertureDegrees = 10;
    [Range(0,0.1f)]
    public float gainInsideOfCone = 0.02f; 
    [Range(0,0.1f)]
    public float gainOutsideOfCone = 0.02f; 
    Vector3 delta;
    List<Vector3> coneBasePoints;
    public Vector3 force;
    Color savedColor;
    Vector3 tp;
    Vector3 ee;
    Vector3 prevl;
    Vector3 preva;
    Vector3 velocity;
    public bool graphics = true;
    [Range(0,5)]
    public float graphicVectorGain = 1;
    public Color coneColor = Color.yellow;
    public int coneResolution = 10;


    void Start(){
        prevl = Vector3.zero;
        preva = Vector3.zero;
        savedColor = coneColor;
        force = Vector3.zero;
    }

    void Update()
    {   
        Vector3 v = gameObject.GetComponent<Rigidbody>().velocity;
        Vector3 vl = Vector3.Dot(v,delta.normalized)*delta.normalized;
        Vector3 va = v-vl;
        if (target == null) {
            Debug.LogWarning("The Target must be assigned!");
            return;
        } 
        
        tp = target.position;
        if (configuring) { // THE CONE MOVES WITH THE EE, NO VF APPLIED
            coneColor = Color.yellow;
            float deltaTheta = 360f/coneResolution;
            delta = target.position-gameObject.transform.position;  
            coneBasePoints = new List<Vector3>();
            Vector3 b = Vector3.Cross(delta.normalized, Vector3.up)*delta.magnitude*Mathf.Tan(coneApertureDegrees*Mathf.PI/180f);
            coneBasePoints.Add(b);
            for (int i=0; i<coneResolution-1; i++) {
                float theta = deltaTheta*i; 
                b = Quaternion.AngleAxis(deltaTheta,delta.normalized)*b;
                coneBasePoints.Add(b);
            }
            coneCenter = gameObject.transform.position;

        } else { // THE CONE IS ASSUMED SELECTED AND STATIONARY, THE VF IS APPLIED
            ee = gameObject.transform.position;
            Vector3 l = Vector3.Dot(ee-tp,delta.normalized)*delta.normalized;
            Vector3 a = ee-tp-l;

            float aperture = Mathf.Tan(coneApertureDegrees*Mathf.PI/180f);

            // CHECKING IF INSIDE OF CONE
            if (a.magnitude >= aperture*l.magnitude) {
                coneColor = Color.red;
            } else {
                coneColor = Color.yellow; 
            }
            Vector3 eta = Vector3.Cross(delta,Vector3.Cross(delta,a));
            float f_mag;
            float eps = 0.05f*a.magnitude; // Switching region is 5% of the radial coordinate 

            if (a.magnitude < aperture*l.magnitude - eps) { // INSIDE OF THE CONE, NOT IN THE SWITCHING REGION
                f_mag = gainInsideOfCone;

            } else if (a.magnitude < aperture*l.magnitude) { // INSIDE OF THE CONE, IN THE SWITCHING REGION
                f_mag = gainInsideOfCone + (gainOutsideOfCone-gainInsideOfCone)/eps*(a.magnitude-(aperture*l.magnitude-eps));

            } else {
                f_mag = gainOutsideOfCone;  
            }

            // Debug.DrawLine(gameObject.transform.position, gameObject.transform.position+vl*graphicVectorGain, Color.magenta);
            // Debug.DrawLine(gameObject.transform.position+vl*graphicVectorGain, gameObject.transform.position+(vl+va)*graphicVectorGain, Color.magenta);
            // Debug.DrawLine(gameObject.transform.position, gameObject.transform.position+(vl+va)*graphicVectorGain, Color.magenta);
            force = (f_mag*-1*va.normalized);
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
            Debug.DrawLine(gameObject.transform.position, gameObject.transform.position+vl*graphicVectorGain, Color.green);
            Debug.DrawLine(gameObject.transform.position, gameObject.transform.position+va*graphicVectorGain, Color.green);
            Arrow(gameObject.transform.position, gameObject.transform.position+v*graphicVectorGain, Color.green);
            Debug.DrawLine(gameObject.transform.position, gameObject.transform.position+force*graphicVectorGain, Color.blue);
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
        //SRAWING THE STEM
        Debug.DrawLine(from,tipcenter, color);
        // DRAWING THE TIP
        for (int i=0; i<coneResolution; i++) {
            Debug.DrawLine(tipcenter+tipBasePoints[i],to, color);
            if (i==coneResolution-1) Debug.DrawLine(tipcenter+tipBasePoints[i],tipcenter+tipBasePoints[0],color);
            else Debug.DrawLine(tipcenter+tipBasePoints[i],tipcenter+tipBasePoints[i+1],color);
        }
    }
}
