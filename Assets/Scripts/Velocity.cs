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

public class Velocity : MonoBehaviour
{
    Vector3 PrevPos; 
    Vector3 NewPos;
    public Vector3 velocity;
    public int filterSize = 10; //The number of frames to average over
    Queue<Vector3> vf = new Queue<Vector3>();

    void Start()
    {  

        PrevPos = gameObject.transform.position;
        NewPos = gameObject.transform.position;

        for (int i = 0; i<filterSize; i++) {
            vf.Enqueue(Vector3.zero);
        }
    }


    void FixedUpdate()
    {
        NewPos = gameObject.transform.position;  
        velocity = (NewPos - PrevPos) / Time.fixedDeltaTime;  
        vf.Enqueue(velocity);  
        vf.Dequeue();  

        Vector3 smoothVelocity = Vector3.zero;  
        foreach(Vector3 v in vf) {
            smoothVelocity+=v/filterSize;
        }
        velocity = smoothVelocity; 
        if (velocity.magnitude < 0.0001f) {
            velocity = Vector3.zero;
        }
        PrevPos = NewPos;  
        
        // Global.Arrow(GameObject.Find(Global.tooltip_path).transform.position, GameObject.Find(Global.tooltip_path).transform.position+smoothVelocity, Color.green);
   
    }
}
