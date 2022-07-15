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
    float min_dist;
    public Transform subject;
    public Transform obstacle;
    public List<Vector3> obstaclePoints;
    public List<Vector3> surfaceNormals;
    [Range(0,1000f)]
    public float forceFieldGain = 0.001f;
    [Range(0,4)]
    public float forceFieldDegree = 2;
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
    Material materialown;
    Material materialhit;
    public Vector3 p;
    public Vector3 t;

    void Start()
    {
        if (subject == null) {
            subject = GameObject.Find(Global.tooltip_path).transform;
        }
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
            // obstaclePoints.Add(obstacle.TransformPoint(meshVertices[i]));
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
        min_dist = 10000;
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
        if (thresholdDistance > Vector3.Distance(subject.position,com)) {
        force = (subject.position-com).normalized*forceFieldGain*(thresholdDistance-Vector3.Distance(subject.position,com));
        }else{force=Vector3.zero;}

        // Rescaling the force the max magnitude if exceeded
        if (force.magnitude > maxForce) {
            force = force.normalized * maxForce;
        }
        if (graphics) {
            Global.Arrow(subject.position, subject.position+force*graphicVectorGain, Color.blue);
            Global.Arrow(subject.position, com, Color.yellow);
        }
        // CHECHING IF EE IS INSIDE OF ORGAN
        if (Vector3.Dot(closestP-subject.position,surfaceNormals[idx_closest])>=0) {
            obstacle.GetComponent<MeshRenderer>().material = materialhit;
            force = Vector3.zero;
            Debug.Log("INSIDE");
        } else {
            obstacle.GetComponent<MeshRenderer>().material = materialown;
        }
        p = subject.position;
        t = com;
    }
    
}
