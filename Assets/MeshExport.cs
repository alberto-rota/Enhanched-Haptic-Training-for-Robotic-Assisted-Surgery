using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;

public class MeshExport : MonoBehaviour
{
    public string path;
    public List<Vector3> surfacePoints;
    public List<Vector3> surfaceNormals;

    void Start()
    {
        path+="\\"+gameObject.name+".stl";
        FileStream streamtraj = new FileStream(path, FileMode.Append);  
        using (StreamWriter writer = new StreamWriter(streamtraj))  
        {  
            writer.Write("X,Y,Z,\n");
            Mesh obst_mesh = gameObject.GetComponent<MeshFilter>().sharedMesh;
            for (int i=0; i<obst_mesh.vertices.Length; i++) {
                Vector3 point = gameObject.transform.TransformPoint(obst_mesh.vertices[i]);
                writer.Write(point.x.ToString()+",");
                writer.Write(point.y.ToString()+",");
                writer.Write(point.z.ToString()+",\n");
            }
        }
    }

    void Update()
    {
        
    }
}
