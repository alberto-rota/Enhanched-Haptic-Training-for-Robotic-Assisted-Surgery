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

//    IMPLEMENTED FROM = {Dynamic 3-D virtual fixtures for minimally invasive beating heart procedures},
//    author = {Jing Ren and Rajni V. Patel and Kenneth A. McIsaac and Gerard Guiraudon and Terry M. Peters},
//    year = {2008},

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[ExecuteInEditMode]
public class SurfaceOrientationGuidanceVF : MonoBehaviour
{
    [Header("Transforms")]
    public Transform subject;
    public Transform surface;

    [Header("Mesh")]
    public List<Vector3> surfacePoints;
    public List<Vector3> surfaceNormals;

    [Header("Virtual Fixture")]
    [Range(0,100000f)]
    public float forceGain = 0.001f;
    [Range(0,0.1f)]
    public float distanceThreshold = 0.002f;
    [Range(0,0.2f)]
    public float distanceHalf = 0.002f;
    [Range(0,10000f)]
    public float distanceSlope = 1f;
    [Range(0,3f)]
    public float torqueGain = 0.01f;
    [Range(0,90f)]
    public float angleThreshold = 5f;
    [Range(0,90f)]
    public float angleHalf = 15f;
    [Range(0,10000f)]
    public float angleSlope = 0f;
    [Range(0,100)]
    public float damp = 10;

    [Header("Graphics")]
    public bool vectorsGraphics = true;
    [Range(0,0.05f)]
    public float graphicVectorGain = 0.01f;
    public bool distanceGraphics = true;

    
    [Header("Output")]
    public Vector3 force;
    public float forceMagnitude;
    public Vector3 torque;
    public float torqueMagnitude;
    public Vector3 closestPcom = Vector3.zero;
    public Vector3 closestP = Vector3.zero;
    public float distance;
    public float pdistance;
    public float ddistance;
    public float distMapped;
    public float angle;

    Material colorok;
    Material colorred;
    Vector3 tool;

    float SqDist(Vector3 a, Vector3 b) {
        return Vector3.Dot((a-b),(a-b));
    }

    void Start()
    {
        if (subject == null) {
            subject = GameObject.Find(Global.tooltip_path).transform;
        }

        colorok = Resources.Load<Material>("Materials/SurfaceGreen");
        colorred = Resources.Load<Material>("Materials/SurfaceRed");

        surfacePoints = surface.GetComponent<CorrectMeshNormals>().surfacePoints;
        surfaceNormals = surface.GetComponent<CorrectMeshNormals>().surfaceNormals;

    }

    void FixedUpdate()
    {
        if (distanceHalf < 0 ) {distanceHalf=0;}
        if (distanceSlope < 1/distanceHalf) {distanceSlope=1/distanceHalf;}
        if (angleHalf < 0 ) {angleHalf=0;}
        if (angleSlope < 1/angleHalf) {angleSlope=1/angleHalf;}

        if (surfacePoints.Count<=0) {
            Debug.LogWarning(@"Mesh is not properly defined, check that 'Read/Write enabled = TRUE'"+
             "in the import settings, the Transform is instaced in the Inspector or try re-initialing Play mode");
        }
        
        int n_close = 0;
        closestPcom = Vector3.zero;
        closestP = Vector3.zero;
        int idx_closest = 0;
        force = Vector3.zero;
        float mind = 1000;

        foreach (Vector3 p in surfacePoints) {
            float d = SqDist(p, subject.position);
            float dcom = Mathf.Pow(distanceThreshold+distanceHalf+5/distanceSlope,2);
            // Debug.Log(dcom);
            if (d <= dcom) {
                closestPcom = closestPcom + p;
                n_close++;
            }
            if (d < mind) {
                mind = d;
                closestP = p;
                idx_closest = surfacePoints.IndexOf(p);
            }
        }   
        closestPcom=closestPcom/n_close;
        if (float.IsNaN(closestPcom.x)) {
            closestPcom = closestP;
        }
        distance = Vector3.Distance(closestP, subject.position);
        distMapped = Global.DistMapAttraction(distance, distanceThreshold, distanceHalf, distanceSlope);

        Vector3 f_dir = (surfaceNormals[idx_closest]).normalized;
        float f_mag = forceGain*distMapped;
        force = f_mag*f_dir;
        

        if (vectorsGraphics) {
            Global.Arrow(subject.position, subject.position+force*graphicVectorGain, Color.blue);
            // Global.Arrow(subject.position, closestPcom, Color.yellow);
        }
        if (distanceGraphics) {
            Vector3 conj = (subject.position-closestPcom).normalized;
            Debug.DrawLine(closestPcom,closestPcom+conj*distanceThreshold+conj*distanceHalf-conj/distanceSlope, Color.green);
            Global.Arrow(  closestPcom+conj*distanceThreshold-conj/distanceSlope+conj*distanceHalf, closestPcom+conj*distanceThreshold+conj*distanceHalf, Color.yellow);
            Global.Arrow(  closestPcom+conj*distanceThreshold+conj/distanceSlope+conj*distanceHalf, closestPcom+conj*distanceThreshold+conj*distanceHalf, Color.yellow);
            Debug.DrawLine(closestPcom+conj*distanceThreshold+conj/distanceSlope+conj*distanceHalf, closestPcom+conj*distanceThreshold+conj/distanceSlope+conj*distanceHalf+conj*4/distanceSlope, Color.red);
        }

        // ADDING VISCOUS COMPONENT (Only if we are already applying an elastic component)
        ddistance = (distance-pdistance)/Time.deltaTime;
        if (force.magnitude > forceGain/3)
            force -= damp*ddistance*f_dir;


        Vector3 n = surfaceNormals[idx_closest];
        Vector3 f_proj = subject.forward - n*Vector3.Dot(subject.forward,n);
        Global.Arrow(closestPcom, closestPcom+f_proj*graphicVectorGain, Color.magenta);
        Global.Arrow(closestPcom, closestPcom+subject.forward*graphicVectorGain, Color.cyan);

        Vector3 rotaxis = Vector3.Cross(subject.forward,f_proj).normalized;
        angle = Vector3.Angle(subject.forward,f_proj);
        torque = rotaxis*Global.AngleMapAttraction(angle, angleThreshold, angleHalf, angleSlope)*torqueGain*(-1);
        // Global.Arrow(closestPcom,closestPcom+rotaxis.normalized*0.01f , Color.yellow);
        // Global.Arrow(GameObject.Find(Global.tooltip_path).transform.position,GameObject.Find(Global.tooltip_path).transform.position+torque.normalized*0.01f , Color.yellow);

        // if (subject.GetComponent<IsPinchableDuo>() != null) {
        //     if (subject.GetComponent<IsPinchableDuo>().pinched == false) {
        //         torque = Vector3.zero;
        //     }
        // } else if (subject.transform.parent.gameObject.GetComponent<IsPinchableDuo>().pinched == false) {
        //     torque = Vector3.zero;
        // }

        // Global.Arrow(closestPcom, closestPcom+rotaxis*graphicVectorGain, Color.yellow);

    
        // CHECHING IF EE IS INSIDE THE SURFACE
        if (distance < distanceThreshold ) {
            force=Vector3.zero;
            // surface.GetComponent<MeshRenderer>().material = colorok;
        } 
        if (distance > distanceThreshold+distanceHalf) {
            surface.GetComponent<MeshRenderer>().material = colorred;
        }else{
            surface.GetComponent<MeshRenderer>().material = colorok;
        }

        pdistance = distance;

        // CleanUp and Loggings
        forceMagnitude = force.magnitude;
        torqueMagnitude = torque.magnitude;
    }
}
