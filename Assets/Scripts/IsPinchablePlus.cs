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

[ExecuteInEditMode,RequireComponent(typeof(SphereCollider)),RequireComponent(typeof(Rigidbody))]
// ,RequireComponent(typeof(FixedJoint))]
public class IsPinchablePlus : MonoBehaviour
{
    Material materialpinched;
    Material materialown;
    Material materialpinchable;
    Vector3 tool;
    Vector3 tool1;
    Vector3 target;
    float d;
    float d1;
    float targetRadius; 
    public bool graphics = false;
    public int whopinched = 0;
    public int whopinchable = 0;
    public Transform pincherObject;
    public Transform pincherObject1;
    public bool restoreGravity = false;

    public bool pinched = false;
    bool pinchable = false;

    void Start()
    {
        materialpinched = Resources.Load<Material>("Materials/Pinched");
        materialown = gameObject.GetComponent<Renderer>().sharedMaterial;
        materialpinchable = Resources.Load<Material>("Materials/Pinchable");
        //Disable the collider
        gameObject.GetComponent<SphereCollider>().enabled = false;
        gameObject.GetComponent<Rigidbody>().mass = 0;
        gameObject.GetComponent<Rigidbody>().useGravity = false;

        pincherObject = GameObject.Find(Global.tooltip_path).transform;
        pincherObject1 = GameObject.Find(Global.tooltip_path1).transform;
    }

    void Update()
    {
        bool pinchingAction  = false;
        if (GameObject.Find("ROBOT").GetComponent<RosSharp.RosBridgeClient.JointJawSubscriber>().jawPosition <  0.2f) {
            pinchingAction = true;
        }

        targetRadius = gameObject.GetComponent<SphereCollider>().radius*gameObject.transform.lossyScale.x;
        tool = pincherObject.position;
        tool1 = pincherObject1.position;
        target = gameObject.transform.position;
        d = Vector3.Distance(target,tool);
        d1 = Vector3.Distance(target,tool1);
        if (d < targetRadius) {
            pinchable = true;
            whopinchable = 0;
            if (pinchingAction) {
                pinched = true;
                whopinchable = 0;
            } else {
                pinched = false;
            }
        } else pinchable = false;

        
        if (d1 < targetRadius) {
            pinchable = true;
            whopinchable = 1;
            if (pinchingAction) {
                pinched = true;
                whopinched = 1;
            } else {
                pinched = false;
            }
        } else pinchable = false;
       
        if (graphics) {
            Global.Arrow(tool,target,Color.yellow);
        }

        if (pinched) {
            if (gameObject.GetComponent<Renderer>().sharedMaterial != materialpinched) {
                gameObject.GetComponent<Renderer>().sharedMaterial = materialpinched;
            }
            if (gameObject.GetComponent<FixedJoint>() == null) {
                gameObject.AddComponent<FixedJoint>();
                if (whopinched == 0) {
                    gameObject.GetComponent<FixedJoint>().connectedBody = pincherObject.gameObject.GetComponent<Rigidbody>();
                } else if (whopinched == 1) {
                    gameObject.GetComponent<FixedJoint>().connectedBody = pincherObject1.gameObject.GetComponent<Rigidbody>();
                }
            }
        } else {
            if (gameObject.GetComponent<FixedJoint>() != null) {
                DestroyImmediate(gameObject.GetComponent<FixedJoint>());
            }
            // gameObject.GetComponent<FixedJoint>().connectedBody = null;
            if (restoreGravity) {
                gameObject.GetComponent<Rigidbody>().useGravity = true;
            }else{
                gameObject.GetComponent<Rigidbody>().useGravity = false;
            }

        }
        if (pinchable && !pinched) {
            gameObject.GetComponent<Renderer>().sharedMaterial = materialpinchable;
        }
        if (!pinchable && !pinched) {
            
            if (gameObject.GetComponent<Renderer>().sharedMaterial != materialown) {
                gameObject.GetComponent<Renderer>().sharedMaterial = materialown;
            }
        }
    }
}
