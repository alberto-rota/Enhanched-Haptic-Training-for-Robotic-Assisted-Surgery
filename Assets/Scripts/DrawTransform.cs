using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[ExecuteInEditMode]
public class DrawTransform : MonoBehaviour
{
    Vector3 pos;
    Vector3 x;
    Vector3 y;
    Vector3 z;
    public bool boolx = true;
    public bool booly = true;
    public bool boolz = true;
    [Range(0,0.05f)]
    public float sc=0.05f;
    
    void Start()
    {
    }


    void Update()
    {
        pos = gameObject.transform.position;
        x = gameObject.transform.right;
        y = gameObject.transform.up;   
        z = gameObject.transform.forward;
        if (boolx)
            Global.Arrow(pos,pos+x*sc,Color.red);
        if (booly)
            Global.Arrow(pos,pos+y*sc,Color.green);
        if (boolz)
            Global.Arrow(pos,pos+z*sc,Color.blue);
    }
}
