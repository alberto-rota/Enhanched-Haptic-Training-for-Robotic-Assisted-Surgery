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
public class TrajectoryOrientationGuidanceVFRL : MonoBehaviour  
{
    [Header("Transforms")]
    public Transform subject;
    public Transform Trajectory;
    public bool left = false;

    [Header("Viscous Virtual Fixture")]
    [Range(0,100f)]
    public float viscousGain = 30;
    [Range(0,3f)]
    [Header("Elastic Virtual Fixture")]
    public float elasticGain = 1f;
    [Range(0,0.01f)]
    public float distanceThreshold = 0.001f;
    [Range(0,0.01f)]
    public float distanceHalf = 0.001f;
    [Range(0,1000f)]
    public float distanceSlope = 0f;
    [Header("Angular Virtual Fixture")]
    [Range(0,0.1f)]
    public float torqueGain = 0.01f;
    [Range(0,20f)]
    public float torqueDamp = 1f;
    [Range(0,90f)]
    public float angleThreshold = 5f;
    [Range(0,90f)]
    public float angleHalf = 5f;
    [Range(0,10000f)]
    public float angleSlope = 2f;


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
    public Vector3 rotaxis;
    public float angle;
    public float pangle = 0;
    public float dangle = 0;
    public  Vector3 force;
    public float f_mag;
    public Vector3 f_dir; 
    public  Vector3 torque;
    public float t_mag;
    public Vector3 t_dir; 
    public int missExchanges = 0;
    bool stillmissing = false;


    void Start()
    {
        if (subject == null) {
            subject = GameObject.Find(Global.tooltip_path).transform;
        }
        subject.GetComponent<Rigidbody>().solverVelocityIterations = 50;
    }

    void FixedUpdate()
    {
        if (angleHalf < 0 ) {angleHalf=0;}
        if (angleSlope < 1/angleHalf) {angleSlope=1/angleHalf;}
        if (distanceHalf < 0 ) {distanceHalf=0;}
        if (distanceSlope < 1/distanceHalf) {distanceSlope=1/distanceHalf;}

        if (Trajectory == null) {
            Debug.LogWarning("The Reference Trajectory must be assigned!");
            return;
        }

        if (subject.GetComponent<IsPinchableDuo>() != null) {
            if (subject.GetComponent<IsPinchableDuo>().whopinched == 2) {
                left = true;
            } else {
                left = false;
            }
        } else {
            if (subject.transform.parent.gameObject.GetComponent<IsPinchableDuo>().whopinched == 2) {
                left = true;
            } else {
                left = false;
            }
        }
        // DISTANCE AND ANGLE
        float mindist = 100000;
        int idx_closest = 0;
        for (int i=0; i<Trajectory.GetComponent<LineRenderer>().positionCount-1; i++) {
            Vector3 point = Trajectory.GetComponent<LineRenderer>().GetPosition(i);
            float d = Vector3.Distance(point, subject.position);
            if (d < mindist) {
                mindist = d;
                closest = point;
                tangent = Trajectory.GetComponent<LineRenderer>().GetPosition(i+1) - point;
                idx_closest = i;
            } 
        }

        deviation = closest - subject.position;
        distance = deviation.magnitude;

        // VELOCITY
        velocity = subject.GetComponent<Velocity>().velocity;

        // FORCE
        float b = viscousGain*Mathf.Sqrt((1-Vector3.Dot(velocity.normalized,deviation.normalized))/2);
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
        force+=elasticGain*Global.DistMapAttraction(distance,distanceThreshold,distanceHalf,distanceSlope)*(closest-subject.position).normalized;

        // When trajectory path is completed, stop applying torque
        if (idx_closest > Trajectory.GetComponent<LineRenderer>().positionCount*0.95f) {
            torque = Vector3.zero;
            return;
        }

        rotaxis = Vector3.Cross(subject.forward,tangent).normalized;
        angle = Vector3.Angle(subject.forward,tangent);
        torque = rotaxis*Global.AngleMapAttraction(angle,angleThreshold,angleHalf,angleSlope)*torqueGain*(-1);
        dangle = (angle - pangle);
        torque -= rotaxis*torqueDamp*(dangle);
        if (subject.GetComponent<IsPinchableDuo>() != null) {
            if (subject.GetComponent<IsPinchableDuo>().pinched == false) {
                torque = Vector3.zero;                
                force = Vector3.zero;
            }
        } else if (subject.transform.parent.gameObject.GetComponent<IsPinchableDuo>().pinched == false) {
            torque = Vector3.zero;            
            force = Vector3.zero;
        }

        t_mag = torque.magnitude;
        t_dir = torque.normalized;

        if (graphics) {
            // Global.Arrow(subject.position, subject.position+velocity*graphicVectorGain, Color.green);
            // Global.Arrow(subject.position, closest, Color.red);
            Global.Arrow(closest,closest+tangent.normalized*0.01f , Color.magenta);
            Global.Arrow(closest,closest+subject.forward.normalized*0.01f , Color.cyan);
            // Global.Arrow(closest,closest+rotaxis.normalized*0.01f , Color.yellow);
            // Global.Arrow(GameObject.Find(Global.tooltip_path).transform.position,GameObject.Find(Global.tooltip_path).transform.position+torque.normalized*0.01f , Color.yellow);
            // Global.Arrow(subject.position, subject.position+force*graphicVectorGain, Color.blue);
        }

        // MISS EXCHANGES
        if (subject.GetComponent<IsPinchableDuo>() != null) {
            if (subject.GetComponent<IsPinchableDuo>().missexchange && stillmissing) {
                missExchanges++;
            } else {
                missExchanges = 0;
            }
            stillmissing = subject.GetComponent<IsPinchableDuo>().missexchange;
        } else {
            if (subject.transform.parent.gameObject.GetComponent<IsPinchableDuo>().missexchange && !stillmissing) {
                missExchanges++;
            } 
            stillmissing = subject.transform.parent.gameObject.GetComponent<IsPinchableDuo>().missexchange;
        }
    pangle = angle;

    // Global.DebugOnHRSV("Miss Exchanges: " + missExchanges);
    }
    
}
