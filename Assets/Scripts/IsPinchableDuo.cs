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
public class IsPinchableDuo : MonoBehaviour
{
    Material materialpinched;
    Material materialown;
    Material materialpinchable;
    Material materialswitchable;
    Material materialmissed;
    Vector3 tool;
    Vector3 tool2;
    Vector3 target;
    float d;
    float d2;
    float targetRadius; 
    public bool graphics = false;
    public int whopinched = 0;
    public int whopinchable = 0;
    public Transform pincherObject1;
    public Transform pincherObject2;
    public bool restoreGravity = false;

    public bool pinchingAction1  = false;
    public bool pinchingAction2  = false;
    public bool pinched1  = false;
    public bool pinched2  = false;
    public bool pinchable1  = false;
    public bool pinchable2  = false;
    public bool drop1 = false;
    public bool drop2 = false;

    public bool missexchange = false;
    public bool pinched = false;
    bool pinchable = false;

    void Start()
    {
        materialown = gameObject.GetComponent<Renderer>().sharedMaterial;
        materialpinched = Resources.Load<Material>("Materials/Pinched");
        materialpinchable = Resources.Load<Material>("Materials/Pinchable");
        materialswitchable = Resources.Load<Material>("Materials/Switchable");
        materialmissed = Resources.Load<Material>("Materials/MissExchange");
        //Disable the collider
        // gameObject.GetComponent<SphereCollider>().enabled = false;
        // gameObject.GetComponent<Rigidbody>().mass = 0;
        gameObject.GetComponent<Rigidbody>().useGravity = false;

        
        if (pincherObject1 == null) {
            pincherObject1 = GameObject.Find(Global.tooltip_path).transform;
        }
        
        if (pincherObject2 == null) {
            pincherObject2 = GameObject.Find(Global.tooltip_path2).transform;
        }
    }

    void Update()
    {

        if (pincherObject1.GetComponent<RosSharp.RosBridgeClient.JointJawSubscriber>().jawPosition <  -1.5f) {
            pinchingAction1 = true;
        } else {
            pinchingAction1 = false;
        }
        if (pincherObject2.GetComponent<RosSharp.RosBridgeClient.JointJawSubscriber>().jawPosition <  -1.5f) {
            pinchingAction2 = true;
        } else {
            pinchingAction2 = false;
        }

        // Has the subject been dropped?
        drop1 = false;
        drop2 = false;
        if (pinched1 && !pinchingAction1) {
            drop1 = true;
        }
        if (pinched2 && !pinchingAction2) {
            drop2 = true;
        }

        targetRadius = gameObject.GetComponent<SphereCollider>().radius*gameObject.transform.lossyScale.x;
        tool = pincherObject1.position;
        tool2 = pincherObject2.position;
        target = gameObject.transform.position;
        d = Vector3.Distance(target,tool);
        d2 = Vector3.Distance(target,tool2);

        if (d < targetRadius) {
            pinchable1 = true;
            whopinchable = 1;
            if (pinchingAction1) {
                pinched1 = true;
                whopinched = 1;
            } else {
                whopinched = 0;
                pinched1 = false;
            }
        } else {
            pinchable1 = false;
            whopinchable = 0;
        }

        
        if (d2 < targetRadius) {
            pinchable2 = true;
            whopinchable = 2;
            if (pinchingAction2) {
                pinched2 = true;
                whopinched = 2;
            } else {
                pinched2 = false;
            }
        } else {
            pinchable2 = false;
        }

        if (!pinchingAction1) {
            pinched1 = false;
        }   
        if (!pinchingAction2) {
            pinched2 = false;
        }
        missexchange = false;
        // Miss per pinch "a vuoto"
        if (pinched1 && !pinched2) {
            if (d2 < targetRadius*1.5f && d2 > targetRadius && pinchingAction2) {
                missexchange = true;
                return;
            } 
        }
        if (pinched2 && !pinched1) {
            if (d < targetRadius*1.5f && d > targetRadius && pinchingAction1) {
                missexchange = true;
                return;
            } 
        }
        // Miss per Drops
        if (d < targetRadius*1.5f  && drop2 && !pinchingAction1) missexchange = true;
        if (d2 < targetRadius*1.5f  && drop1 && !pinchingAction2) missexchange = true;


        pinchable = pinchable1 || pinchable2;
        pinched = pinched1 || pinched2;

        if (graphics) {
            Global.Arrow(tool,target,Color.yellow);
            Global.Arrow(tool2,target,Color.yellow);
        }

        if (pinched) {
            if (gameObject.GetComponent<Renderer>().sharedMaterial != materialpinched) {
                gameObject.GetComponent<Renderer>().sharedMaterial = materialpinched;
            }
            if (gameObject.GetComponent<FixedJoint>() == null) {
                gameObject.AddComponent<FixedJoint>();
                }
            if (whopinched == 1) {
                gameObject.GetComponent<FixedJoint>().connectedBody = pincherObject1.gameObject.GetComponent<Rigidbody>();
            } else if (whopinched == 2) {
                gameObject.GetComponent<FixedJoint>().connectedBody = pincherObject2.gameObject.GetComponent<Rigidbody>();
            }
        } else {
            if (gameObject.GetComponent<FixedJoint>() != null) {
                DestroyImmediate(gameObject.GetComponent<FixedJoint>());
            }
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
        if (d < targetRadius && d2 < targetRadius) {
            gameObject.GetComponent<Renderer>().sharedMaterial = materialswitchable;
        }

        if (missexchange) gameObject.GetComponent<Renderer>().sharedMaterial = materialmissed;
    }
}
