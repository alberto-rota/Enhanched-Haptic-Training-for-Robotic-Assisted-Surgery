using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[ExecuteInEditMode]
public class CheckMeshNormals : MonoBehaviour
{
    
    [Range(0,1  )]
    public float normalVectorsLength = 1;
    Transform surface;
    public List<Vector3> surfacePoints;
    public List<Vector3> surfaceNormals;

    void Awake()
    {
        surface = gameObject.transform;
        surfacePoints = new List<Vector3>();
        surfaceNormals= new List<Vector3>();


        Mesh mesh;
        mesh = surface.GetComponent<MeshFilter>().sharedMesh;
        Vector3[] meshVertices = mesh.vertices;
        Vector3[] meshNormals = mesh.normals;
        
        for (var i = 0; i < meshVertices.Length; i++){
            surfacePoints.Add(surface.TransformPoint(meshVertices[i]));
        }
        for (var i = 0; i < meshNormals.Length; i++){
            surfaceNormals.Add(surface.TransformDirection(meshNormals[i]));
        }
        // for (int i = 0; i<surfaceNormals.Count; i++) {
        //     Debug.DrawLine(surfacePoints[i], surfacePoints[i]+surfaceNormals[i]*normalVectorsLength, Color.cyan);
        // }
        // Group-4 average of surface normals
        // Vector3 averageNormal = Vector3.zero;
        // int NperV = 4;
        
        for (int i = 0; i < surfacePoints.Count; i++){
            List<int> sameV = new List<int>();
            Vector3 averageNormal = Vector3.zero;
            for (int j = 0; j < surfacePoints.Count; j++){
                if (surfacePoints[i] == surfacePoints[j]){
                    averageNormal += surfaceNormals[j];
                    sameV.Add(j);
                }
            }
            if(sameV.Count>1){
                averageNormal /= sameV.Count;
                for (int j = 0; j < sameV.Count; j++){
                    surfaceNormals[sameV[j]] = averageNormal;
                }
            }
        }

        // for (int i = 0; i < surfaceNormals.Count-NperV; i+=NperV){
        //     for (int j = 0; j < NperV; j++){    
        //         averageNormal += surfaceNormals[i+j];
        //     }
        //     averageNormal /= NperV;
        //     for (int j = 0; j < NperV; j++){
        //         surfaceNormals[i+j] = averageNormal;
        //     }
        // }

    }

    void Update() {
        if (normalVectorsLength>0) 
            for (int i = 0; i<surfaceNormals.Count; i++) {
                Debug.DrawLine(surfacePoints[i], surfacePoints[i]+surfaceNormals[i]*normalVectorsLength, Color.magenta);
            }
    }
}
