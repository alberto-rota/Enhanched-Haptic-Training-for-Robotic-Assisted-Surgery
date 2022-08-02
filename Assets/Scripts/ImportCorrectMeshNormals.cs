using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class ImportCorrectMeshNormals : MonoBehaviour
{    
    [Range(0,0.01f)]
    public float normalVectorsLength = 0.002f;
    
    public string path;    
    public List<Vector3> surfacePoints;
    public List<Vector3> surfaceNormals;

    void Awake()
    {

        string alltext = System.IO.File.ReadAllText(path+"\\"+gameObject.name+"_points.csv");
        string header = alltext.Split("\n"[0])[0];
        foreach (string line in alltext.Split("\n"[0]))
        {
            if (line != header)
            {
                // To be honest I'm using a try-catch here because I have no fucking idea of why it doesnt read the last line
                // of the CSV correctly
                try {
                    string[] line_split = line.Split(","[0]);
                    Vector3 point = new Vector3(float.Parse(line_split[0]), float.Parse(line_split[1]), float.Parse(line_split[2]));
                    surfacePoints.Add(point);
                } catch (Exception e) { 
                    if (e == null) {}
                }
            }
        }
        alltext = System.IO.File.ReadAllText(path+"\\"+gameObject.name+"_normals.csv");
        header = alltext.Split("\n"[0])[0];
        foreach (string line in alltext.Split("\n"[0]))
        {
            if (line != header)
            {
                try{
                    string[] line_split = line.Split(","[0]);
                    Vector3 normal = new Vector3(float.Parse(line_split[0]), float.Parse(line_split[1]), float.Parse(line_split[2]));
                    surfaceNormals.Add(normal);
                } catch (Exception e) { 
                    if (e == null) {}
                }
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

