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

[ExecuteInEditMode]
public class ObstacleAvoidanceForceFieldVF : MonoBehaviour
{
    public Transform subject;
    public Transform obstacle;
    public List<Vector3> obstaclePoints;
    public List<Vector3> surfaceNormals;
    [Range(0,5f)]
    public float forceFieldGain = 0.001f;
    [Range(0,4)]
    public int forceFieldDegree = 2;
    [Range(0,1)]
    public float thresholdDistance = 0.0005f;
    [Range(0,100)]
    public float maxForce=100;
    public Vector3 force;
    public bool graphics = true;
    // [Range(0,1  )]  
    // public float normalVectorsLength = 0;
    [Range(0,5)]
    public float graphicVectorGain = 1;
    Transform EndEffector;
    Material materialown;
    Material materialhit;

    void Start()
    {
        if (subject == null) {
            subject = GameObject.Find("PSM").transform;
        }
        EndEffector = gameObject.transform;
        obstaclePoints = new List<Vector3>();
        surfaceNormals= new List<Vector3>();
        materialown = obstacle.GetComponent<MeshRenderer>().sharedMaterial;
        materialhit = Resources.Load<Material>("Materials/ObstacleHit");

        Mesh surgicalMesh;
        surgicalMesh = obstacle.GetComponent<MeshFilter>().sharedMesh;
        Vector3[] meshVertices = surgicalMesh.vertices;
        Vector3[] meshNormals = surgicalMesh.normals;
        
        for (var i = 0; i < meshVertices.Length; i++){
            obstaclePoints.Add(obstacle.TransformPoint(meshVertices[i]));
        }
        for (var i = 0; i < meshNormals.Length; i++){
            surfaceNormals.Add(obstacle.TransformDirection(meshNormals[i]));
        }
    }

    void Update()
    {
        if (obstaclePoints.Count<=0) {
            Debug.LogWarning(@"Mesh is not properly defined, check that 'Read/Write enabled = TRUE' in the import settings, the Transform is instaced in the Inspector or try re-initialing Play mode");
        }
        

        int n_close=0;
        Vector3 com = Vector3.zero;
        Vector3 closestP = Vector3.zero;
        float min_dist = 10000;
        int idx_closest = 0;
        int j=0;
        force = Vector3.zero;
        foreach (Vector3 p in obstaclePoints) {
            float d = Vector3.Distance(p, subject.position);
            if (d < min_dist) {
                min_dist = d;
                closestP = p;
                idx_closest = j;
            }
            if (d < thresholdDistance) {
                com=com+p;
                n_close++;
            }
            j++;
        }   
        com = com/n_close;

        // ADD DAMP
        force = (subject.position-com).normalized*forceFieldGain/Mathf.Pow(Vector3.Distance(subject.position,com),forceFieldDegree);

        // Rescaling the force the max magnitude if exceeded
        if (force.magnitude > maxForce) {
            force = force.normalized * maxForce;
        }
        if (graphics) {
            Arrow(subject.position, subject.position+force*graphicVectorGain, Color.blue);
        }

        // CHECHING IF EE IS INSIDE OF ORGAN
        if (Vector3.Dot(closestP-EndEffector.position,surfaceNormals[idx_closest])>=0) {
            obstacle.GetComponent<MeshRenderer>().material = materialhit;
            force = Vector3.zero;
        } else {
            obstacle.GetComponent<MeshRenderer>().material = materialown;
        }

    }
    void Arrow(Vector3 from, Vector3 to, Color color) {
        int coneResolution=20;
        float deltaTheta = 360f/coneResolution;

        Vector3 stem = (to-from)*0.9f;
        Vector3 tip = (to-from)-stem;
        float tipradius = 0.05f*(to-from).magnitude;
        List<Vector3> tipBasePoints = new List<Vector3>();
        Vector3 b = Vector3.Cross(tip.normalized, Vector3.up)*tipradius;
        tipBasePoints.Add(b);

        for (int i=0; i<coneResolution-1; i++) {
            float theta = deltaTheta*i; 
            b = Quaternion.AngleAxis(deltaTheta,tip.normalized)*b;
            tipBasePoints.Add(b);
        }
        Vector3 tipcenter = from+stem;
        //SRAWING THE STEM
        Debug.DrawLine(from, tipcenter, color);
        // DRAWING THE TIP
        for (int i=0; i<coneResolution; i++) {
            Debug.DrawLine(tipcenter+tipBasePoints[i],to, color);
            if (i==coneResolution-1) Debug.DrawLine(tipcenter+tipBasePoints[i],tipcenter+tipBasePoints[0],color);
            else Debug.DrawLine(tipcenter+tipBasePoints[i],tipcenter+tipBasePoints[i+1],color);
        }
    }
}
