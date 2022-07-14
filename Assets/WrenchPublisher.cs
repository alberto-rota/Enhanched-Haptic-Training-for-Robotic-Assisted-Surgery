/*
© Siemens AG, 2017-2018
Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
<http://www.apache.org/licenses/LICENSE-2.0>.
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

using UnityEngine;
using System.Collections.Generic;
using System.IO;
using System;

namespace RosSharp.RosBridgeClient
{
    public class WrenchPublisher : UnityPublisher<MessageTypes.Geometry.Wrench>
    {
        public GameObject Tool;
        private MessageTypes.Geometry.Wrench message;
        private MessageTypes.Geometry.Vector3 geometryPoint;

        protected override void Start()
        {
            base.Start();
            InitializeMessage();
        }

        private void FixedUpdate()
        {
            UpdateMessage();
        }

        private void InitializeMessage()
        {
            message = new MessageTypes.Geometry.Wrench();
            message.force = new MessageTypes.Geometry.Vector3();
            message.force.x = 0;
            message.torque = new MessageTypes.Geometry.Vector3();
        }
        private void UpdateMessage()
        {
            // Debug.Log("PUBLISHING START");
            // Vector3 force = Tool.GetComponent<SumForces>().totalForce;
            // if(force.x !=0.0 || force.y !=0.0 || force.z !=0.0)
            // {  
                // GetGeometryPoint(Tool.GetComponent<SumForces>().totalForce.Unity2Ros(), message.force);
                Publish(message);
                //Debug.Log("PUBLISHING OK");
            // }
        }

        private void GetGeometryPoint(Vector3 force, MessageTypes.Geometry.Vector3 geometryPoint)
        {
            geometryPoint.x = force.x;
            geometryPoint.y = force.y;
            geometryPoint.z = force.z;
        }
    }
}

