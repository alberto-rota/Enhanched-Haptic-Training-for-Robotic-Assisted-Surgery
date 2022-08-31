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

//    IMPLEMENTED FROM = {A dynamic non-energy-storing guidance constraint with motion redirection for robot-assisted surgery},
//    author = {Nima Enayati and Eva C.Alves Costa and Giancarlo Ferrigno and Elena De Momi},
//    year = {2016},

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[ExecuteInEditMode]
public class Graph : MonoBehaviour
{
    [Header("Transforms")]
    public Transform subject;

    [Header("Virtual Fixture")]
    public Transform Trajectory;
    [Range(0,100f)]
    public float gain = 30;
    [Range(0,0.1f)]
    public float torquegain = 0.001f;


    [Header("Graphics")]
    public bool graphics = true;
    [Range(0,0.05f)]   
    public float graphicVectorGain = 0.01f;

    [Header("Output")]
    Vector3 closest;
    public Vector3 velocity;
    public Vector3 deviation;
    public float distance;
    public Vector3 tangent;
    public float angle;
    public  Vector3 force;
    public float f_mag;
    public Vector3 f_dir; 
    public  Vector3 torque;
    public float t_mag;
    public Vector3 t_dir; 


    void Start()
    {
        if (subject == null) {
            subject = GameObject.Find(Global.tooltip_path).transform;
        }
        subject.GetComponent<Rigidbody>().solverVelocityIterations = 50;
    }

    void Update()
    {
        if (Trajectory == null) {
            Debug.LogWarning("The Reference Trajectory must be assigned!");
            return;
        }
        // DISTANCE AND ANGLE
        float mindist = 100000;
        for (int i=0; i<Trajectory.GetComponent<LineRenderer>().positionCount-1; i++) {
            Vector3 point = Trajectory.GetComponent<LineRenderer>().GetPosition(i);
            float d = Vector3.Distance(point, subject.position);
            if (d < mindist) {
                mindist = d;
                closest = point;
                tangent = Trajectory.GetComponent<LineRenderer>().GetPosition(i+1) - point;
            } 
        }

        deviation = closest - subject.position;
        distance = deviation.magnitude;

        // VELOCITY
        velocity = subject.GetComponent<Velocity>().velocity;

        // FORCE
        float b = gain*Mathf.Sqrt((1-Vector3.Dot(velocity.normalized,deviation.normalized))/2);
        f_mag = b*velocity.magnitude;

        if (Vector3.Dot(velocity.normalized, deviation.normalized)<0) { // When moving away
            f_dir = deviation.normalized;
        } else {  // When approaching
            f_dir = Quaternion.AngleAxis(
                (1+Vector3.Dot(velocity.normalized,deviation.normalized))*Mathf.PI/2*180f/Mathf.PI, // Rotation Angle (in degrees)
                Vector3.Cross(velocity.normalized,deviation.normalized)
                )*velocity.normalized;  //Rotation Axis
        }

        force = f_mag*f_dir;

        // TODO: FIX TORQUE CALCULATION AND SCALING
        Vector3 rotaxis = Vector3.Cross(subject.forward,tangent).normalized;
        angle = Mathf.Acos(Vector3.Dot(subject.forward,tangent))*180f/Mathf.PI;
        Quaternion rot = Quaternion.AngleAxis(angle,rotaxis);
        torque = rot.eulerAngles*torquegain;

        if (graphics) {
            velocity = 0.5f*(subject.right*0.5f+subject.forward*0.7f);
            Global.Arrow(subject.position, subject.position+velocity*graphicVectorGain, Color.green);
            Global.Arrow(subject.position, closest, Color.red);
            Global.Arrow(closest,closest+tangent.normalized*0.01f , Color.magenta);
            Global.Arrow(closest,closest+rotaxis.normalized*0.01f , Color.cyan);
            Global.Arrow(closest,closest+subject.forward*0.01f , Color.blue);
            // Global.Arrow(subject.position, subject.position+force*graphicVectorGain, Color.blue);
        }
    }
    
}
