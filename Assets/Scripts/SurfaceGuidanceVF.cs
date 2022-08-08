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
public class SurfaceGuidanceVF : MonoBehaviour
{
    [Header("Transforms")]
    public Transform subject;
    public Transform surface;

    [Header("Mesh")]
    public List<Vector3> surfacePoints;
    public List<Vector3> surfaceNormals;

    [Header("Distance Map")]
    [Range(0,100000f)]
    public float gain = 0.001f;
    [Range(0,0.1f)]
    public float threshold = 0.002f;
    [Range(0,0.2f)]
    public float half = 0.002f;
    [Range(0,10000f)]
    public float slope = 1f;

    [Header("Graphics")]
    public bool vectorsGraphics = true;
    [Range(0,0.05f)]
    public float graphicVectorGain = 0.01f;
    public bool distanceGraphics = true;

    
    [Header("Output")]
    public Vector3 force;
    public float forceMagnitude;
    public Vector3 closestPcom = Vector3.zero;
    public Vector3 closestP = Vector3.zero;
    public float distance;
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

    void Update()
    {
        if (half < 0 ) {half=0;}
        if (slope < 1/half) {slope=1/half;}

        if (gameObject.GetComponent<OVF_UniversalParameters>() != null) {
            gain = gameObject.GetComponent<OVF_UniversalParameters>().gain;
            threshold = gameObject.GetComponent<OVF_UniversalParameters>().threshold;
            half = gameObject.GetComponent<OVF_UniversalParameters>().half;
            slope = gameObject.GetComponent<OVF_UniversalParameters>().slope;
        }
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
            float dcom = Mathf.Pow(threshold+half,2);
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
        distMapped = Global.DistMapAttraction(distance, threshold, half, slope);


        Vector3 f_dir = -(subject.position-closestPcom).normalized;
        float f_mag = gain*distMapped;
        

        if (vectorsGraphics) {
            Global.Arrow(subject.position, subject.position+force*graphicVectorGain, Color.blue);
            Global.Arrow(subject.position, closestPcom, Color.yellow);
        }
        if (distanceGraphics) {
            Vector3 conj = (subject.position-closestPcom).normalized;
            Debug.DrawLine(closestPcom,closestPcom+conj*threshold+conj*half-conj/slope, Color.green);
            Global.Arrow(  closestPcom+conj*threshold-conj/slope+conj*half, closestPcom+conj*threshold+conj*half, Color.yellow);
            Global.Arrow(  closestPcom+conj*threshold+conj/slope+conj*half, closestPcom+conj*threshold+conj*half, Color.yellow);
            Debug.DrawLine(closestPcom+conj*threshold+conj/slope+conj*half, closestPcom+conj*threshold+conj/slope+conj*half+conj*4/slope, Color.red);
        }
        force = f_mag*f_dir;

        // Vector3 rotaxis = Vector3.Cross(subject.forward,tangent).normalized;
        float angle = 90-Mathf.Acos(Vector3.Dot(subject.forward,surfaceNormals[idx_closest]))*180/Mathf.PI;
        // Quaternion rot = Quaternion.AngleAxis(angle*180f/Mathf.PI,rotaxis);
        // torque = rot.eulerAngles*torquegain;

        // CHECHING IF EE IS INSIDE THE SURFACE
        if (distance < threshold ) {
            force=Vector3.zero;
            // surface.GetComponent<MeshRenderer>().material = colorok;
        } 
        if (distance > threshold+half) {
            surface.GetComponent<MeshRenderer>().material = colorred;
        }else{
            surface.GetComponent<MeshRenderer>().material = colorok;
        }

        // CleanUp and Loggings
        forceMagnitude = force.magnitude;
    }
}
