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
    public bool configuring = true;
    public Transform target;
    Vector3 coneCenter;
    [Range(1,30)]
    public float coneApertureDegrees = 10;
    [Range(0,1)]
    public float gainInsideOfCone = 0.5f; 
    [Range(0,1)]
    public float gainOutsideOfCone = 0.5f; 
    Vector3 delta;
    List<Vector3> coneBasePoints;
    public Vector3 force;
    Color savedColor;
    Vector3 prevnpar;
    Vector3 prevnperp;
    Vector3 velocity;
    public bool graphics = true;
    [Range(0,5)]
    public float graphicVectorGain = 1;
    public Color coneColor = Color.yellow;
    public int coneResolution = 10;

    void Start(){
        prevnpar = Vector3.zero;
        prevnperp = Vector3.zero;
        savedColor = coneColor;
        force = Vector3.zero;
    }

    void Update()
    {   

        if (target == null) {
            Debug.LogWarning("The Target must be assigned!");
            return;
        } 
        
        if (configuring) { // THE CONE MOVES WITH THE EE, NO VF APPLIED
            float deltaTheta = 360f/coneResolution;
            coneBasePoints = new List<Vector3>();
            delta = gameObject.transform.position-target.position;  
            Vector3 b = Vector3.Cross(delta.normalized, Vector3.up)*delta.magnitude*Mathf.Tan(coneApertureDegrees*Mathf.PI/180f);
            coneBasePoints.Add(b);
            for (int i=0; i<coneResolution-1; i++) {
                float theta = deltaTheta*i; 
                b = Quaternion.AngleAxis(deltaTheta,delta.normalized)*b;
                coneBasePoints.Add(b);
            }
            coneCenter = gameObject.transform.position;
        } else { // THE CONE IS ASSUMED SELECTED AND STATIONARY, THE VF IS APPLIED
            Vector3 ee = gameObject.transform.position;
            Vector3 n_par = Vector3.Dot(target.position-ee,delta.normalized)*delta.normalized*-1;
            Vector3 n_perp = -n_par-(target.position-ee);
            Vector3 v_par = (n_par-prevnpar)/Time.deltaTime;
            Vector3 v_perp = (n_perp-prevnperp)/Time.deltaTime;
            prevnpar = n_par;
            prevnperp = n_perp;
            float coneWidthHere = Mathf.Tan(coneApertureDegrees*Mathf.PI/180f);
            // CollisionCheck
            if (n_perp.magnitude >= coneWidthHere*n_par.magnitude) {
                coneColor = Color.red;
            } else {
                coneColor = Color.yellow; 
            }
            Vector3 eta = Vector3.Cross(delta,Vector3.Cross(n_perp,delta));
            float c;
            float eps = 0.01f*n_perp.magnitude;
            if (n_perp.magnitude < coneWidthHere*n_par.magnitude - eps) { // INSIDE OF THE CONE
                c = gainInsideOfCone;
            } else if (n_perp.magnitude < coneWidthHere*n_par.magnitude && Vector3.Dot(v_par+v_perp,eta) >= 0) { // TRANSITION REGION
                c = gainInsideOfCone - ((coneWidthHere*n_par.magnitude - n_perp.magnitude)/eps)*(gainInsideOfCone-gainOutsideOfCone);
            } else if (Vector3.Dot(v_par+v_perp,n_perp)>0) { // OUTSIDE OF THE CONE AND MOVING INWARDS
                c = gainOutsideOfCone; 
            } else { // OUTSIDE OF THE CONE AND MOVING INWARDS
                c = gainInsideOfCone;
            }

            // Debug.DrawLine(gameObject.transform.position, gameObject.transform.position+v_par*graphicVectorGain, Color.magenta);
            // Debug.DrawLine(gameObject.transform.position+v_par*graphicVectorGain, gameObject.transform.position+(v_par+v_perp)*graphicVectorGain, Color.magenta);
            // Debug.DrawLine(gameObject.transform.position, gameObject.transform.position+(v_par+v_perp)*graphicVectorGain, Color.magenta);
            force = (c*v_perp.normalized+v_par)*-1;
            if (graphics) {
                Debug.DrawLine(target.position,target.position+n_par,Color.white);
                Debug.DrawLine(target.position+n_par,target.position+n_par+n_perp,Color.white);
            }
        }

        // DRAWING THE CONE
        for (int i=0; i<coneResolution; i++) {
            Debug.DrawLine(coneCenter+coneBasePoints[i],target.position,coneColor);
            if (i==coneResolution-1) Debug.DrawLine(coneCenter+coneBasePoints[i],coneCenter+coneBasePoints[0],coneColor);
            else Debug.DrawLine(coneCenter+coneBasePoints[i],coneCenter+coneBasePoints[i+1],coneColor);
        }

        if (graphics) {
            Debug.DrawLine(gameObject.transform.position, gameObject.transform.position+velocity*graphicVectorGain, Color.green);
            // Debug.DrawLine(gameObject.transform.position, closest, Color.red);
            Debug.DrawLine(gameObject.transform.position, gameObject.transform.position+force*graphicVectorGain, Color.blue);
        }
    }
}
