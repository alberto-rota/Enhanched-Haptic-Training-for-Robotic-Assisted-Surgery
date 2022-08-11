using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RandomSpawn : MonoBehaviour
{
    public Vector3 positionNoise = Vector3.zero;
    public Vector3 orientationNoise = Vector3.zero;

    void Start()
    {
        gameObject.transform.localPosition += new Vector3(Random.Range(-positionNoise.x,positionNoise.x), Random.Range(-positionNoise.y,positionNoise.y), Random.Range(-positionNoise.z,positionNoise.z));
        gameObject.transform.eulerAngles += new Vector3(Random.Range(-orientationNoise.x,orientationNoise.x), Random.Range(-orientationNoise.y,orientationNoise.y), Random.Range(-orientationNoise.z,orientationNoise.z));
        
    }

    void Update()
    {
    }
}
