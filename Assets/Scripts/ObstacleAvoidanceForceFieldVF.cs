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

//    IMPLEMENTED FROM = {Haptically Augmented Teleoperation},
//    author = {Nicolas Turro, Oussama Khatib and Eve Coste-Maniere },
//    year = {2001},

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;


[ExecuteInEditMode]
public class ObstacleAvoidanceForceFieldVF : MonoBehaviour
{
    [Header("Transforms")]
    public Transform subject;
    public Transform obstacle;
    // [Space(20)]

    [Header("Mesh")]
    public List<Vector3> obstaclePoints;
    public List<Vector3> obstacleNormals;
    public Material materialown;
    public Material materialhit;
    // [Space(20)]

    [Header("Distance Map")]
    [Range(0,100000f)]
    public float gain = 0.001f;
    [Range(0,0.1f)]
    public float threshold = 0.002f;
    [Range(0,0.2f)]
    public float half = 0.002f;
    [Range(0,10000)]
    public float slope = 1f;

    [Header("Graphics")]
    public bool vectorsGraphics = true;
    [Range(0,5)]
    public float graphicVectorGain = 1;
    public bool distanceGraphics = true;

    
    [Header("Output")]
    public Vector3 force;
    public float forceMagnitude;
    public Vector3 closestPcom = Vector3.zero;
    public Vector3 closestP = Vector3.zero;
    public float dist;
    public float distMapped;

    Vector3 p;
    Vector3 t;

    void Start()
    {
        if (subject == null) {
            subject = GameObject.Find(Global.tooltip_path).transform;
        }
        materialown = obstacle.GetComponent<MeshRenderer>().sharedMaterial;
        materialhit = Resources.Load<Material>("Materials/ObstacleHit");

        obstaclePoints = obstacle.GetComponent<CorrectMeshNormals>().surfacePoints;
        obstacleNormals = obstacle.GetComponent<CorrectMeshNormals>().surfaceNormals;
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
        if (obstaclePoints.Count<=0) {
            Debug.LogWarning(@"Mesh is not properly defined, check that 'Read/Write enabled = TRUE'"+
             "in the import settings, the Transform is instaced in the Inspector or try re-initialing Play mode");
        }
        
        int n_close = 0;
        closestPcom = Vector3.zero;
        closestP = Vector3.zero;
        int idx_closest = 0;
        force = Vector3.zero;
        float mind = 1000;

        foreach (Vector3 p in obstaclePoints) {
            float d = Vector3.Distance(p, subject.position);
            float dcom = threshold+half+5/slope;
            if (d <= dcom) {
                closestPcom = closestPcom + p;
                n_close++;
            }
            if (d < mind) {
                mind = d;
                closestP = p;
                idx_closest = obstaclePoints.IndexOf(p);
            }
        }   
        closestPcom=closestPcom/n_close;
        dist = Vector3.Distance(closestP, subject.position);
        distMapped = Global.DistMapRepulsion(Vector3.Distance(closestP, subject.position), threshold, half, slope);

        Vector3 f_dir = (subject.position-closestPcom).normalized;
        float f_mag = gain*distMapped;
        

        if (vectorsGraphics) {
            Global.Arrow(subject.position, subject.position+force*graphicVectorGain, Color.blue);
            Global.Arrow(subject.position, closestPcom, Color.yellow);
        }
        if (distanceGraphics) {
            Vector3 conj = (subject.position-closestPcom).normalized;
            Debug.DrawLine(closestPcom,closestPcom+conj*threshold+conj*half-conj/slope, Color.red);
            Global.Arrow(  closestPcom+conj*threshold-conj/slope+conj*half, closestPcom+conj*threshold+conj*half, Color.yellow);
            Global.Arrow(  closestPcom+conj*threshold+conj/slope+conj*half, closestPcom+conj*threshold+conj*half, Color.yellow);
            Debug.DrawLine(closestPcom+conj*threshold+conj/slope+conj*half, closestPcom+conj*threshold+conj/slope+conj*half+conj*4/slope, Color.green);
        }
        force = f_mag*f_dir;

        // CHECHING IF EE IS INSIDE OF ORGAN
        if (Vector3.Dot(closestP-subject.position, obstacleNormals[idx_closest])>=0 || dist < threshold) {
            force=gain*obstacleNormals[idx_closest];
            obstacle.GetComponent<MeshRenderer>().material = materialhit;
        } else {
            obstacle.GetComponent<MeshRenderer>().material = materialown;
        }

        // CleanUp and Loggings
        forceMagnitude = force.magnitude;
    }
    
}
