// Copyright (c) 2022 Alberto Rota
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[ExecuteInEditMode, RequireComponent(typeof(SphereCollider)), RequireComponent(typeof(Rigidbody)),]
public class IsPinchableTarget : MonoBehaviour
{
    Material materialtarget;
    Material materialtargethit;
    Material materialtargetpinchable;

    public bool graphics = false;
    public Transform pincherObject;
    float d;
    float targetRadius; 
    public bool reached = false;
    bool pinchable = false;
   
    void Start()
    {
        if (pincherObject == null){
            pincherObject = GameObject.Find(Global.tooltip_path).transform;
        }
        
        materialtarget = Resources.Load<Material>("Materials/Target");
        materialtargethit = Resources.Load<Material>("Materials/TargetReached");
        materialtargetpinchable = Resources.Load<Material>("Materials/TargetPinchable");
        //Disable the collider
        gameObject.GetComponent<SphereCollider>().enabled = false;
        gameObject.GetComponent<Rigidbody>().mass = 0;
        gameObject.GetComponent<Rigidbody>().constraints = RigidbodyConstraints.FreezeAll;
        gameObject.GetComponent<Rigidbody>().useGravity = false;
        // gameObject.GetComponent<Rigidbody>().useGravity = false;
    }

    void Update()
    {
        bool pinchingAction  = false;
        if (pincherObject.GetComponent<RosSharp.RosBridgeClient.JointJawSubscriber>().jawPosition <  -1.5f) {
            pinchingAction = true;
        }
        
        targetRadius = gameObject.GetComponent<SphereCollider>().radius*gameObject.transform.lossyScale.x;
        Vector3 tool = pincherObject.position;
        Vector3 target = gameObject.transform.position;
        d = Vector3.Distance(target,tool);
        if (d < targetRadius) {
            pinchable = true;
        } else pinchable = false;
        if (pinchable && pinchingAction) {
            reached = true;
        } 

        if (graphics) {
            Global.Arrow(tool,target,Color.yellow);
        }

        if (reached) {
            gameObject.GetComponent<Renderer>().material = materialtargethit;
        } else if (pinchable) {
            gameObject.GetComponent<Renderer>().material = materialtargetpinchable;
        }
        if (!reached && !pinchable) {
            gameObject.GetComponent<Renderer>().material = materialtarget;
        }
    }
}
