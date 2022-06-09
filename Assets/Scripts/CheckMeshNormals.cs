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

    void Start()
    {        
    }

    void Update()
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
        
        for (int i = 0; i<surfaceNormals.Count; i++) {
            Debug.DrawLine(surfacePoints[i], surfacePoints[i]+surfaceNormals[i]*normalVectorsLength);
        }
    }
}
