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
    public class EffortModeSubscriber : UnitySubscriber<MessageTypes.Std.String>
    {
        private bool isMessageReceived;
        public string mode;

        protected override void Start()
        {
            base.Start();
        }

        protected override void ReceiveMessage(MessageTypes.Std.String message)
        {
            if (message.data == "MTMR: control CARTESIAN_SPACE/EFFORT_MODE") {
                mode = "Effort";
            }else if (message.data == "MTMR: control CARTESIAN_SPACE/TRAJECTORY_MODE") {
                mode = "Trajectory";
            }else mode = "Undefined";
        }

        private void Update()
        {
            if (mode == "Effort") {
                // GameObject.FindWithTag("ROBOT").GetComponent<WrenchPublisherRight>().enabled = true;                
                // GameObject.FindWithTag("ROBOT").GetComponent<WrenchPublisherLeft>().enabled = true;
                GameObject.FindWithTag("ROBOT").GetComponent<WrenchPublisher>().enabled = true;
            }else if (mode == "Trajectory") {
                // GameObject.FindWithTag("ROBOT").GetComponent<WrenchPublisherRight>().enabled = false;                
                // GameObject.FindWithTag("ROBOT").GetComponent<WrenchPublisherLeft>().enabled = false;
                GameObject.FindWithTag("ROBOT").GetComponent<WrenchPublisher>().enabled = false;
            }
        }
    }
}