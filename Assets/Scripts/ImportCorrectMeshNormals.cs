using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// [ExecuteInEditMode]
public class ImportCorrectMeshNormals : MonoBehaviour
{
    
    public string path;    
    public List<Vector3> surfacePoints;
    public List<Vector3> surfaceNormals;

    void Awake()
    {

        string alltext = System.IO.File.ReadAllText(path+"\\"+gameObject.name+"_points.csv");
        Debug.Log(alltext);
        string header = alltext.Split("\n"[0])[0];
        int row = 0;
        foreach (string line in alltext.Split("\n"[0]))
        {
            if (line != header)
            {
                string[] line_split = line.Split(","[0]);
                Vector3 point = new Vector3(float.Parse(line_split[0]), float.Parse(line_split[1]), float.Parse(line_split[2]));
                Debug.Log(row+" "+gameObject.name);
                surfacePoints.Add(point);
            }
            row++;
        }
        alltext = System.IO.File.ReadAllText(path+"\\"+gameObject.name+"_normals.csv");
        Debug.Log(alltext);
        header = alltext.Split("\n"[0])[0];
        foreach (string line in alltext.Split("\n"[0]))
        {
            if (line != header)
            {
                string[] line_split = line.Split(","[0]);
                Vector3 normal = new Vector3(float.Parse(line_split[0]), float.Parse(line_split[1]), float.Parse(line_split[2]));
                surfaceNormals.Add(normal);
            }
        }


        // string[] = (lines[0].Trim()).Split(","[0]);
        // var x : float;
        // float.TryParse(lineData[0], x);


        // obstaclePoints = obstacle.GetComponent<CorrectMeshNormals>().surfacePoints;
        // obstacleNormals = obstacle.GetComponent<CorrectMeshNormals>().surfaceNormals;
    }

    void Update() 
    
    {

    }
}
