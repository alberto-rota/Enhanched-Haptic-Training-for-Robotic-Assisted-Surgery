using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// [ExecuteInEditMode]
public class CorrectMeshNormals : MonoBehaviour
{
    
    [Range(0,0.01f)]
    public float normalVectorsLength = 0.002f;
    Transform surface;
    public List<Vector3> surfacePoints;
    public List<Vector3> surfaceNormals;
    
    public bool invert = false;

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
            for (int j = i; j < surfacePoints.Count; j++){
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

        if (invert) {
            for (int i = 0; i < surfaceNormals.Count; i++) {
                surfaceNormals[i] = -surfaceNormals[i];
            }
        }
    }

    void Update() {
        if (normalVectorsLength>0) 
            for (int i = 0; i<surfaceNormals.Count; i++) {
                Debug.DrawLine(surfacePoints[i], surfacePoints[i]+surfaceNormals[i]*normalVectorsLength, Color.magenta);
            }
    }
}
