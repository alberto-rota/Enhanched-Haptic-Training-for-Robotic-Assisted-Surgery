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

[ExecuteInEditMode]
public class CheckIfReached : MonoBehaviour
{
    public Transform subject;
    Vector3 target;
    public float targetRadius = 1; 
    public bool REACHED = false;
    Material targetreached;
    Material targetnotreached;

    void Start()
    {
        targetreached = Resources.Load<Material>("Materials/TargetReached");   
        targetnotreached = Resources.Load<Material>("Materials/TargetNotReached");  
    }


    void Update()
    {
        Vector3 subjectp = subject.TransformPoint(subject.position);
        target = subject.TransformPoint(gameObject.transform.position);
        if (Vector3.Distance(subjectp, target) < targetRadius) {
            REACHED = true;
            gameObject.GetComponent<Renderer>().material = targetreached;
        }
        if (REACHED==false) {
            gameObject.GetComponent<Renderer>().material = targetnotreached;
        }
    }
}
