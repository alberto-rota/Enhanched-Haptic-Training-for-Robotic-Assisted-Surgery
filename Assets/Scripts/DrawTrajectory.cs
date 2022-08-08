using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[ExecuteInEditMode]
public class DrawTrajectory : MonoBehaviour
{
    public Transform from;
    public Transform to; 
    [Range(0,0.05f)]
    public float width=0.05f;
    
    void Start()
    {
    }


    void Update()
    {

    }
}
