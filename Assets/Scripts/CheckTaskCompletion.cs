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
    GameObject canvasr;
    GameObject canvasl;

    void Start()
    {
        targets = new List<Transform>();
        foreach (Transform child in gameObject.transform)
        {
            targets.Add(child);
        }
        canvasr = GameObject.Find("Text/Canvas");
        canvasl = GameObject.Find("Text/CanvasL");
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
            if (GameObject.FindWithTag("ROBOT").GetComponent<RosSharp.RosBridgeClient.WrenchFTPublisher>() != null) {
                GameObject.FindWithTag("ROBOT").GetComponent<RosSharp.RosBridgeClient.WrenchFTPublisher>().safetyOverride = true;
                // GameObject.FindWithTag("ROBOT").GetComponent<RosSharp.RosBridgeClient.WrenchFTPublisher>().totalTorque = Vector3.zero;
            }
            
            if (GameObject.FindWithTag("ROBOT").GetComponent<RosSharp.RosBridgeClient.WrenchFTPublisherRight>() != null) {
                GameObject.FindWithTag("ROBOT").GetComponent<RosSharp.RosBridgeClient.WrenchFTPublisherRight>().safetyOverride = true;            
                // GameObject.FindWithTag("ROBOT").GetComponent<RosSharp.RosBridgeClient.WrenchFTPublisherRight>().totalTorqueRight = Vector3.zero;   
            }
            if (GameObject.FindWithTag("ROBOT").GetComponent<RosSharp.RosBridgeClient.WrenchFTPublisherLeft>() != null) {
                GameObject.FindWithTag("ROBOT").GetComponent<RosSharp.RosBridgeClient.WrenchFTPublisherLeft>().safetyOverride = true;  
                // GameObject.FindWithTag("ROBOT").GetComponent<RosSharp.RosBridgeClient.WrenchFTPublisherLeft>().totalTorqueLeft = Vector3.zero;     
            }

            
            if (GameObject.FindWithTag("ROBOT").GetComponent<RosSharp.RosBridgeClient.WrenchPublisher>() != null) {
                GameObject.FindWithTag("ROBOT").GetComponent<RosSharp.RosBridgeClient.WrenchPublisher>().safetyOverride = true;
            }
            if (GameObject.FindWithTag("ROBOT").GetComponent<RosSharp.RosBridgeClient.WrenchPublisherLeft>() != null) {
                GameObject.FindWithTag("ROBOT").GetComponent<RosSharp.RosBridgeClient.WrenchPublisherLeft>().safetyOverride = true;
            }
            if (GameObject.FindWithTag("ROBOT").GetComponent<RosSharp.RosBridgeClient.WrenchPublisherRight>() != null) {
                GameObject.FindWithTag("ROBOT").GetComponent<RosSharp.RosBridgeClient.WrenchPublisherRight>().safetyOverride = true;
            }
            canvasr.SetActive(true);            
            canvasl.SetActive(true);  
        } else {
            canvasr.SetActive(false);            
            canvasl.SetActive(false);        
        }
    }
}
