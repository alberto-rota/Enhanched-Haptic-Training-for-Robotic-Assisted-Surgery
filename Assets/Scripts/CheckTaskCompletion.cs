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

public class CheckTaskCompletion : MonoBehaviour
{
    public List<Transform> targets;
    public int targetReached = 0;
    GameObject canvas;

    void Start()
    {
        targets = new List<Transform>();
        foreach (Transform child in gameObject.transform)
        {
            targets.Add(child);
        }
        canvas = GameObject.Find("Text/Canvas");
    }

    void Update()
    {
        targetReached = 0;
        foreach (Transform child in gameObject.transform) {
            if (child.GetComponent<IsTarget>() != null) {
                if (child.GetComponent<IsTarget>().reached) {
                    targetReached++;
                }
            }
            if (child.GetComponent<IsPinchableTarget>() != null) {
                if (child.GetComponent<IsPinchableTarget>().reached) {
                    targetReached++;
                }
            }
        }
        if (targetReached == targets.Count) {
            canvas.SetActive(true);
        } else {
            canvas.SetActive(false);    
        }
    }
}
