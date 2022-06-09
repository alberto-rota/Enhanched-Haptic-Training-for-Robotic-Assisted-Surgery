using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[ExecuteInEditMode, RequireComponent(typeof(SphereCollider))]
public class IsTarget : MonoBehaviour
{
    Material colorreached;
    Material colornotreached;
    Vector3 psm;
    Vector3 p;
    float d;
    float targetRadius; 
    public bool reached = false;

    void Start()
    {
        colorreached = Resources.Load<Material>("Materials/TargetReached");
        colornotreached = Resources.Load<Material>("Materials/Target");
        //Disable the collider
        gameObject.GetComponent<SphereCollider>().enabled = false;
    }

    void Update()
    {
        targetRadius = gameObject.GetComponent<SphereCollider>().radius*gameObject.transform.localScale.x;
        psm = GameObject.Find("PSM").transform.TransformPoint(GameObject.Find("PSM").transform.position);
        p = GameObject.Find("PSM").transform.TransformPoint(gameObject.transform.position);
        d = Vector3.Distance(p,psm);
        if (d < targetRadius) {
            reached = true;
            gameObject.GetComponent<Renderer>().material = colorreached;
            // gameObject.GetComponent<Halo>().color = colorreached.albedo;

        }
        if (reached==false) {
            gameObject.GetComponent<Renderer>().material = colornotreached;
            // gameObject.GetComponent<Halo>().color = colornotreached.albedo;
        }
    }
}
