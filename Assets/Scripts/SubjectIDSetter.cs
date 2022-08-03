using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[ExecuteInEditMode]
public class SubjectIDSetter : MonoBehaviour
{
    void Start()
    {
    }

    void Update()
    {
        if (gameObject.GetComponent<UnityEngine.UI.Text>().text == "" && Global.subjectID != "X") {
            gameObject.GetComponent<UnityEngine.UI.Text>().text = Global.subjectID;
        }else if (gameObject.GetComponent<UnityEngine.UI.Text>().text != ""){
            Global.subjectID = gameObject.GetComponent<UnityEngine.UI.Text>().text;
        }

        GameObject.Find("/Text/CanvasID/SubjectIDLabel").GetComponent<UnityEngine.UI.Text>().text = "SubjectID: " + Global.subjectID;

    }
}
