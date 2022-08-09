using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using UnityEngine.SceneManagement;

public class MeshExportCorrect : MonoBehaviour
{
    public List<Vector3> surfacePoints;
    public List<Vector3> surfaceNormals;
    public string path;

    void Start()
    {
        surfacePoints = gameObject.GetComponent<CorrectMeshNormals>().surfacePoints;
        surfaceNormals = gameObject.GetComponent<CorrectMeshNormals>().surfaceNormals;

        FileStream points = new FileStream(path+"\\"+gameObject.name+"_points.csv", FileMode.Create);  
        using (StreamWriter writer = new StreamWriter(points))  
        {  
            writer.Write("X,Y,Z\n");
            foreach (Vector3 point in surfacePoints) {
                writer.Write(point.x.ToString()+",");
                writer.Write(point.y.ToString()+",");
                writer.Write(point.z.ToString()+"\n");
            }
            writer.Write("\n");
        }

        FileStream normals = new FileStream(path+"\\"+gameObject.name+"_normals.csv", FileMode.Create);  
        using (StreamWriter writer = new StreamWriter(normals))  
        {  
            writer.Write("X,Y,Z\n");
            foreach (Vector3 normal in surfaceNormals) {
                writer.Write(normal.x.ToString()+",");
                writer.Write(normal.y.ToString()+",");
                writer.Write(normal.z.ToString()+"\n");
            }
            writer.Write("\n");
        }    

    }

    void Update()
    {
        
    }
}
