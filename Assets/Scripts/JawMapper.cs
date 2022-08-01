using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class JawMapper : MonoBehaviour
{
    public GameObject jawMessageReciever;
    public float scale = 1;
    public float offset = 0;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        Vector3 angls = Vector3.zero;
        angls.y = jawMessageReciever.GetComponent<RosSharp.RosBridgeClient.JointJawSubscriber>().jawPosition*scale+offset;
        gameObject.transform.localEulerAngles = angls;
    }
}
