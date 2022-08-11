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

public class LeftPSMON : MonoBehaviour
{
    void Start()
    {
        
    }

    void Update()
    {
        if (GameObject.FindWithTag("ROBOT").GetComponent<RosSharp.RosBridgeClient.EffortModeSubscriberLeft>() != null) {
            if (GameObject.Find("Manager").GetComponent<RosSharp.RosBridgeClient.PedalClutchSubscriber>().pressed == true)
                GameObject.Find("/Text/CanvasROS/LeftPSMON").GetComponent<UnityEngine.UI.Image>().color = Color.white;
            else if (GameObject.FindWithTag("ROBOT").GetComponent<RosSharp.RosBridgeClient.EffortModeSubscriberLeft>().mode == "Effort")
                GameObject.Find("/Text/CanvasROS/LeftPSMON").GetComponent<UnityEngine.UI.Image>().color = Color.green;
            else if (GameObject.FindWithTag("ROBOT").GetComponent<RosSharp.RosBridgeClient.EffortModeSubscriberLeft>().mode == "Trajectory")
                GameObject.Find("/Text/CanvasROS/LeftPSMON").GetComponent<UnityEngine.UI.Image>().color = new Color(254f/255f,184f/255f,107f/255f,1);
            else if (GameObject.FindWithTag("ROBOT").GetComponent<RosSharp.RosBridgeClient.EffortModeSubscriberLeft>().mode == "Undefined")
                GameObject.Find("/Text/CanvasROS/LeftPSMON").GetComponent<UnityEngine.UI.Image>().color = Color.red;
        } else { 
            if (GameObject.Find("Manager").GetComponent<RosSharp.RosBridgeClient.PedalClutchSubscriber>().pressed == true)
                GameObject.Find("/Text/CanvasROS/LeftPSMON").GetComponent<UnityEngine.UI.Image>().color = Color.white;
            else if (GameObject.FindWithTag("ROBOT").GetComponent<RosSharp.RosBridgeClient.EffortModeSubscriber>().mode == "Effort")
                GameObject.Find("/Text/CanvasROS/LeftPSMON").GetComponent<UnityEngine.UI.Image>().color = Color.green;
            else if (GameObject.FindWithTag("ROBOT").GetComponent<RosSharp.RosBridgeClient.EffortModeSubscriber>().mode == "Trajectory")
                GameObject.Find("/Text/CanvasROS/LeftPSMON").GetComponent<UnityEngine.UI.Image>().color = new Color(254f/255f,184f/255f,107f/255f,1);
            else if (GameObject.FindWithTag("ROBOT").GetComponent<RosSharp.RosBridgeClient.EffortModeSubscriber>().mode == "Undefined")
                GameObject.Find("/Text/CanvasROS/LeftPSMON").GetComponent<UnityEngine.UI.Image>().color = Color.red;
        }
    }
}
