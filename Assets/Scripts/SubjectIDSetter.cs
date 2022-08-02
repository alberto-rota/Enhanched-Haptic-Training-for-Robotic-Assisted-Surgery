using System.Collections;
using System.Collections.Generic;
using UnityEngine;

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
        // if (gameObject.GetComponent<UnityEngine.UI.Text>().text != "" && Global.subjectID == "X"){
        //     Global.subjectID = gameObject.GetComponent<UnityEngine.UI.Text>().text;
        // }


        if (Global.subjectID != "X") {
            GameObject.Find("/Text/Canvas/SubjectIDLabel").GetComponent<UnityEngine.UI.Text>().color = Color.black;
        } else {
            GameObject.Find("/Text/Canvas/SubjectIDLabel").GetComponent<UnityEngine.UI.Text>().color = Color.red;
        }
    }
}
