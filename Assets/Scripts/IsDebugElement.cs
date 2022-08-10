using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class IsDebugElement : MonoBehaviour
{
    void Start()
    {
        
    }

    void Update()
    {
        if (!Global.debugmode) {
            gameObject.transform.localPosition = new Vector3(gameObject.transform.localPosition.x, 860, gameObject.transform.localPosition.z);
        } else {
            gameObject.transform.localPosition = new Vector3(gameObject.transform.localPosition.x, 475, gameObject.transform.localPosition.z);
        }
    }
}
