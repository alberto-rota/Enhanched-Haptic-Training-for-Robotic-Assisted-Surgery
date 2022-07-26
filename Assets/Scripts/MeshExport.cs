using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using UnityEngine.SceneManagement;

public class MeshExport : MonoBehaviour
{
    public string folderpath;
    string path;
    public List<Vector3> surfacePoints;
    public List<Vector3> surfaceNormals;

    void Start()
    {
        folderpath = "C:\\Users\\alber\\Desktop\\Active_Constraints\\Task_Data\\"+SceneManager.GetActiveScene().name+"\\SCENESTLS";
        path=folderpath+"\\"+gameObject.name+"_global.csv";
        FileStream streamstlglobal = new FileStream(path, FileMode.Append);  
        using (StreamWriter writer = new StreamWriter(streamstlglobal))  
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
        
        path=folderpath+"\\"+gameObject.name+"_tri.csv";
        FileStream streamstllocal = new FileStream(path, FileMode.Append);  
        using (StreamWriter writer = new StreamWriter(streamstllocal))  
        {  
            writer.Write("ID0,ID1,ID2,\n");
            int[] tris = gameObject.GetComponent<MeshFilter>().sharedMesh.triangles;
                for (int i=0; i<tris.Length-3; i+=3) {
                    writer.Write(tris[i+0].ToString()+",");
                    writer.Write(tris[i+1].ToString()+","); 
                    writer.Write(tris[i+2].ToString()+",\n");
                }
        }
    }

    void Update()
    {
        
    }
}
