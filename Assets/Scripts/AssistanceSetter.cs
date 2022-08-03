using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AssistanceSetter : MonoBehaviour
{
    void Start()
    {
    }

    void Update()
    {
        if (gameObject.GetComponent<UnityEngine.UI.Text>().text == ""){
            gameObject.GetComponent<UnityEngine.UI.Text>().text = Global.assistance.ToString();
        }else if (gameObject.GetComponent<UnityEngine.UI.Text>().text != ""){
            Global.assistance = float.Parse(gameObject.GetComponent<UnityEngine.UI.Text>().text);
        }

        GameObject.Find("/Text/CanvasAssistance/AssistanceLabel").GetComponent<UnityEngine.UI.Text>().text = "A: " + Global.assistance.ToString();

    }
}
