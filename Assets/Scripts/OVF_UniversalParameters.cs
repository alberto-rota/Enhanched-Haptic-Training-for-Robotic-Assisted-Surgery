using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[ExecuteInEditMode]
public class OVF_UniversalParameters : MonoBehaviour
{
    [Range(0,100000f)]
    public float gain = 0.001f;
    [Range(0,0.1f)]
    public float threshold = 0.1f;
    [Range(0,0.2f)]
    public float half = 0.5f;
    [Range(0,10000)]
    public float slope = 0.1f;
    [Range(0,100)]
    public float damp = 10f;

    void Start()
    {
        
    }

    void Update()
    {
        if (half < 0 ) {half=0;}
        if (slope < 1/half) {slope=1/half;}
    }
}
