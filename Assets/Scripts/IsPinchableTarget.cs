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

    Vector3 psm;
    Vector3 p;
    public Transform subject;
    public float d;
    float targetRadius; 
    public bool reached = false;
    bool pinchable = false;

    void Start()
    {
        if (subject == null){
            subject = GameObject.Find("PSM").transform;
        }
        materialtarget = Resources.Load<Material>("Materials/Target");
        materialtargethit = Resources.Load<Material>("Materials/TargetReached");
        materialtargetpinchable = Resources.Load<Material>("Materials/TargetPinchable");
        //Disable the collider
        gameObject.GetComponent<SphereCollider>().enabled = false;
        gameObject.GetComponent<Rigidbody>().mass = 0;
        gameObject.GetComponent<Rigidbody>().useGravity = false;
    }

    void Update()
    {
        bool pinchingAction = Input.GetKeyDown(KeyCode.Space);
        bool releasingAction = Input.GetKeyUp(KeyCode.Space);
        string pinch2 = @"PSM/outer_yaw_joint/outer_yaw_joint_revolute/outer_pitch_joint"+
        "/outer_pitch_joint_revolute/outer_insertion_joint/outer_insertion_joint_prismatic/"+
        "outer_roll_joint/outer_roll_joint_revolute/outer_wrist_pitch_joint/"+
        "outer_wrist_pitch_joint_revolute/outer_wrist_yaw_joint/outer_wrist_yaw_joint_revolute/jaw_mimic_2_joint";
        string pinch1 = @"PSM/outer_yaw_joint/outer_yaw_joint_revolute/outer_pitch_joint"+
        "/outer_pitch_joint_revolute/outer_insertion_joint/outer_insertion_joint_prismatic/"+
        "outer_roll_joint/outer_roll_joint_revolute/outer_wrist_pitch_joint/"+
        "outer_wrist_pitch_joint_revolute/outer_wrist_yaw_joint/outer_wrist_yaw_joint_revolute/jaw_mimic_1_joint";
        targetRadius = gameObject.GetComponent<SphereCollider>().radius*gameObject.transform.localScale.x;
        p = subject.position;
        d = Vector3.Distance(p,gameObject.transform.position);
        if (d < targetRadius) {
            pinchable = true;
            if (pinchingAction) {
                reached = true;
            }
        } else pinchable = false;

        if (reached) {
            gameObject.GetComponent<Renderer>().material = materialtargethit;
        } else if (pinchable) {
            gameObject.GetComponent<Renderer>().material = materialtargetpinchable;
        }
        if (!reached && !pinchable) {
            gameObject.GetComponent<Renderer>().material = materialtarget;
        }
        if (Input.GetKeyDown(KeyCode.Space)) {
            GameObject.Find(pinch2).transform.Rotate(Vector3.up,-10*Mathf.PI/180);            
            GameObject.Find(pinch1).transform.Rotate(Vector3.up,+10*Mathf.PI/180);
        }else if (Input.GetKeyUp(KeyCode.Space)) {
            GameObject.Find(pinch2).transform.Rotate(Vector3.up,10*Mathf.PI/180);            
            GameObject.Find(pinch1).transform.Rotate(Vector3.up,-10*Mathf.PI/180);
        }
    }
}
