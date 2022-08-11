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


using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class EffortModeSubscriberRight : UnitySubscriber<MessageTypes.Std.String>
    {
        private bool isMessageReceived;
        public string mode;

        protected override void Start()
        {
            base.Start();
        }

        protected override void ReceiveMessage(MessageTypes.Std.String message)
        {
            if (message.data == "MTMR: control CARTESIAN_SPACE/EFFORT_MODE" || message.data == "MTML: control CARTESIAN_SPACE/EFFORT_MODE") {
                mode = "Effort";
            }else if (message.data == "MTMR: control CARTESIAN_SPACE/TRAJECTORY_MODE" || message.data == "MTML: control CARTESIAN_SPACE/TRAJECTORY_MODE") {
                mode = "Trajectory";
            }else mode = "Undefined";
        }

        private void Update()
        {
            GameObject robot = GameObject.FindWithTag("ROBOT");

            if (mode == "Effort") {
                if (robot.GetComponent<WrenchPublisherRight>() != null && robot.GetComponent<WrenchPublisherRight>().enabled == false) 
                    robot.GetComponent<WrenchPublisherRight>().enabled = true;
                if (robot.GetComponent<WrenchFTPublisherRight>() != null && robot.GetComponent<WrenchFTPublisherRight>().enabled == false) 
                    robot.GetComponent<WrenchFTPublisherRight>().enabled = true;
                if (robot.GetComponent<WrenchPublisher>() != null && robot.GetComponent<WrenchPublisher>().enabled == false) 
                    robot.GetComponent<WrenchPublisher>().enabled = true;
                // if (robot.GetComponent<WrenchPublisherLeft>() != null) 
                //     robot.GetComponent<WrenchPublisherLeft>().enabled = true;
            }else if (mode == "Trajectory") {
                if (robot.GetComponent<WrenchPublisherRight>() != null && robot.GetComponent<WrenchPublisherRight>().enabled == true) 
                    robot.GetComponent<WrenchPublisherRight>().enabled = false;
                if (robot.GetComponent<WrenchFTPublisherRight>() != null && robot.GetComponent<WrenchFTPublisherRight>().enabled == true) 
                    robot.GetComponent<WrenchFTPublisherRight>().enabled = false;
                if (robot.GetComponent<WrenchPublisher>() != null && robot.GetComponent<WrenchPublisher>().enabled == true) 
                    robot.GetComponent<WrenchPublisher>().enabled = false;
                // if (robot.GetComponent<WrenchPublisherLeft>() != null) 
                //     robot.GetComponent<WrenchPublisherLeft>().enabled = false;
            }
        }
    }
}