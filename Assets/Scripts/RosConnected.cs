using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RosConnected : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        if (GameObject.FindWithTag("ROBOT").GetComponent<RosSharp.RosBridgeClient.RosConnector>().connected)
            GameObject.Find("/Text/CanvasROS/ROSConnected").GetComponent<UnityEngine.UI.Image>().color = Color.green;
        else
            GameObject.Find("/Text/CanvasROS/ROSConnected").GetComponent<UnityEngine.UI.Image>().color = Color.red;
    }
}
