using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[ExecuteInEditMode]
public class IsObstacle : MonoBehaviour
{
    Material materialown;
    Material materialhit;
    public Vector3 psm;
    float d;
    public float mindist;
    public bool hit = false;
    public Transform subject;
    public List<Vector3> obstaclePoints;
    public List<Vector3> surfaceNormals;

    void Start()
    {
        if (subject == null) {
            subject = GameObject.Find("PSM").transform;
        }

        obstaclePoints = new List<Vector3>();
        surfaceNormals = new List<Vector3>();
        materialown = gameObject.GetComponent<MeshRenderer>().sharedMaterial;
        materialhit = Resources.Load<Material>("Materials/ObstacleHit");

        Mesh surgicalMesh;
        surgicalMesh = gameObject.GetComponent<MeshFilter>().sharedMesh;
        Vector3[] meshVertices = surgicalMesh.vertices;
        Vector3[] meshNormals = surgicalMesh.normals;
        
        for (var i = 0; i < meshVertices.Length; i++){
            obstaclePoints.Add(gameObject.transform.TransformPoint(meshVertices[i]));
        }
        for (var i = 0; i < meshNormals.Length; i++){
            surfaceNormals.Add(gameObject.transform.TransformDirection(meshNormals[i]));
        }

    }

    void Update()
    {
        psm = subject.transform.position;

        mindist = 100000;
        Vector3 closestP = Vector3.zero;
        int idx_closest=0;
        int j=0;
        foreach (Vector3 p in obstaclePoints) {
            d = Vector3.Distance(p, psm);
            if (d < mindist) {
                mindist = d;
                closestP = p;
                idx_closest=j;
            }
            j++;
        }   

        // CHECHING IF EE IS INSIDE OF ORGAN
        if (Vector3.Dot(closestP-psm,surfaceNormals[idx_closest])>=0) {
            hit = true;
        } else {
            hit = false;
        }

        if (hit==true) {
            gameObject.GetComponent<Renderer>().material = materialhit;
        } else {
            gameObject.GetComponent<Renderer>().material = materialown;
        }
    }
}
