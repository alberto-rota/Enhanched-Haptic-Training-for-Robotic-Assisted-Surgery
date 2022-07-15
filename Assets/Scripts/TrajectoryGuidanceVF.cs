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
public class TrajectoryGuidanceVF : MonoBehaviour
{
    public Transform subject;
    public Transform Trajectory;

    [Range(0,100000)]
    public float viscousCoefficient = 1;

    [SerializeField]
    float maxForce = 100;


    Vector3 closest;
    public Vector3 velocity;
    public Vector3 deviation;
    public  Vector3 force;
    public float f_mag;
    public Vector3 f_dir; 
    [SerializeField]
    bool graphics = true;
    [Range(0,5)]
    public float graphicVectorGain = 1;

    void Start()
    {
        if (subject == null) {
            subject = GameObject.Find("PSM").transform;
        }
        subject.GetComponent<Rigidbody>().solverVelocityIterations = 50;
    }

    void Update()
    {
        if (Trajectory == null) {
            Debug.LogWarning("The Reference Trajectory must be assigned!");
            return;
        }
        // DISTANCE
        float mindist = 100000;
        // Vector3[] extractPositions = new Vector3[GetComponent<LineRenderer>().positionCount];
        for (int i=0; i<Trajectory.GetComponent<LineRenderer>().positionCount; i++) {
            Vector3 point = Trajectory.GetComponent<LineRenderer>().GetPosition(i);
            float d = Vector3.Distance(point, subject.position);
            if (d < mindist) {
                mindist = d;
                closest = point;
            }
        }
        deviation = closest - subject.position;

        // VELOCITY
        velocity = subject.GetComponent<Rigidbody>().velocity;

        // FORCE
        float b = viscousCoefficient*Mathf.Sqrt((1-Vector3.Dot(velocity.normalized,deviation.normalized))/2);
        f_mag = b*velocity.magnitude;
        if (f_mag > maxForce) {
            f_mag = maxForce;
        }
        // f_dir;
        if (Vector3.Dot(velocity.normalized, deviation.normalized)<0) { // When moving away
            f_dir = deviation.normalized;
        } else {  // When approaching
            f_dir = Quaternion.AngleAxis(
                (1+Vector3.Dot(velocity.normalized,deviation.normalized))*Mathf.PI/2*180f/Mathf.PI, // Rotation Angle (in degrees)
                Vector3.Cross(velocity.normalized,deviation.normalized)
                )*velocity.normalized;  //Rotation Axis
        }
        force = f_mag*f_dir;

        if (graphics) {
            Global.Arrow(subject.position, subject.position+velocity*graphicVectorGain, Color.green);
            Global.Arrow(subject.position, closest, Color.red);
            Global.Arrow(subject.position, subject.position+force*graphicVectorGain, Color.blue);
        }
    }
    
}
