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
    float sc;
    // Start is called before the first frame update
    void Start()
    {
        pos = gameObject.transform.position;
        x = gameObject.transform.right;
        y = gameObject.transform.up;   
        z = gameObject.transform.forward;
        sc = 0.1f;
    }

    // Update is called once per frame
    void Update()
    {
        Global.Arrow(pos,pos+x*sc,Color.red);
        Global.Arrow(pos,pos+y*sc,Color.green);
        Global.Arrow(pos,pos+z*sc,Color.blue);
    }
}
