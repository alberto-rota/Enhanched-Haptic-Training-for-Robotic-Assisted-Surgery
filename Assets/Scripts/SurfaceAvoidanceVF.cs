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

//////////////////////////////////////////////////////////////////////////////////
//                                  OBSOLETE                                    //
//                      Use "ObstacleAvoidanceForce" instead                    //
//////////////////////////////////////////////////////////////////////////////////



using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[ExecuteInEditMode]
public class SurfaceAvoidanceVF : MonoBehaviour
{
    public Transform subject;
    public Transform surface;
    public List<Vector3> surfacePoints;
    List<Vector3> surfaceNormals;
    [Range(0,100)]
    public float gamma = 1; 
    public float distance10Perc;
    [Range(0,100)]
    public float forceOnSurface = 1;
    public float maxForce=100;
    public Vector3 force;
    public bool graphics = true;
    // [Range(0,1  )]
    // public float normalVectorsLength = 0;
    [Range(0,0.05f)]
    public float graphicVectorGain = 0.01f;
    public float distance;
    Material colorok;
    Material colorred;
    Vector3 tool;

    void Start()
    {
        if (subject == null) {
            subject = GameObject.Find(Global.tooltip_path).transform;
        }
        tool = subject.position;
        surfacePoints = new List<Vector3>();
        surfaceNormals= new List<Vector3>();
        colorok = Resources.Load<Material>("Materials/Organ");
        colorred = Resources.Load<Material>("Materials/OrganRed");

        Mesh surgicalMesh;
        surgicalMesh = surface.GetComponent<MeshFilter>().sharedMesh;
        Vector3[] meshVertices = surgicalMesh.vertices;
        Vector3[] meshNormals = surgicalMesh.normals;
        
        for (var i = 0; i < meshVertices.Length; i++){
            surfacePoints.Add(surface.TransformPoint(meshVertices[i]));
            surfaceNormals.Add(surface.TransformDirection(meshNormals[i]));
        }
    }

    void Update()
    {
        if (surfacePoints.Count<=0) {
            Debug.LogWarning(@"Mesh is not properly defined, check that 'Read/Write enabled = TRUE' in 
            the import settings, the Transform is instaced in the Inspector or try re-initialing Play mode");
            return;
        }
        distance10Perc = -Mathf.Log(0.1f)/gamma;
        float mindist = 100000;
        Vector3 closestP = Vector3.zero;
        int idx_closest=0;
        int j=0;
        foreach (Vector3 p in surfacePoints) {
            float d = Vector3.Distance(p, tool);
            if (d < mindist) {
                mindist = d;
                closestP = p;
                idx_closest=j;
            }
            j++;
        }
        float f_mag = forceOnSurface*Mathf.Exp(-gamma*mindist);
        Vector3 f_dir = surfaceNormals[idx_closest];
        force = f_mag*f_dir.normalized;
        if (f_mag > maxForce) {
            f_mag = maxForce;
        }

        // CHECHING IF EE IS INSIDE OF ORGAN
        if (Vector3.Dot(closestP-tool,surfaceNormals[idx_closest])>=0) {
            surface.GetComponent<MeshRenderer>().material = colorred;
        } else {    
            surface.GetComponent<MeshRenderer>().material = colorok;
        }

        if (graphics) {
            Global.Arrow(tool, closestP, Color.red);
            Global.Arrow(tool, tool+force*graphicVectorGain, Color.blue);
        }

        // DRAWING MESH NORMALS [DEPRECATED]
        // if (normalVectorsLength > 0 && graphics) {
        //     for (int i = 0; i<surfaceNormals.Count; i++) {
        //         Debug.DrawLine(surfacePoints[i], surfacePoints[i]+surfaceNormals[i]*normalVectorsLength);
        //     }
        // }
    }
    
}
