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
            // message.force.x = 0;
            message.torque = new MessageTypes.Geometry.Vector3();
        }
        private void UpdateMessage()
        {
            // TROUBLESHOOTING
            // message.force.x=0;
            // message.force.y=0;
            // message.force.z=1;

            // Publish(message);
            // Debug.Log("PUBLISHING START");


        ///////////////////////////////////////////////////////
        // THIS WORKS LOL
            // Vector3 unityforce = Tool.GetComponent<SumForces>().totalForce;
            // Vector3 rosforce = Vector3.zero;
            // rosforce.x = unityforce.x*-1;
            // rosforce.y = unityforce.z*-1*0;
            // rosforce.z = unityforce.y*+1*0;
            // // rosforce = rosforce*-1;
            // if(rosforce.x !=0.0 || rosforce.y !=0.0 || rosforce.z !=0.0 || float.IsNaN(rosforce.x))
            // {  
            //     GetGeometryPoint(rosforce, message.force);
            //    Publish(message);
            //     // Debug.Log("PUBLISHING OK");
            // } else {
            //     GetGeometryPoint(Vector3.zero.Unity2Ros(), message.force);
            //     Publish(message);
            //     // Debug.Log("PUBLISHING ZERO");
            // }
        ///////////////////////////////////////////////////////
        Vector3 unityforce;
            if (GameObject.Find("ROBOT").GetComponent<SumForces>() != null) {
                unityforce = GameObject.Find("ROBOT").GetComponent<SumForces>().totalForce;
            }else {
                unityforce = Vector3.zero;
            }
            Vector3 rosforce = Vector3.zero;
            rosforce.x = 0;
            rosforce.y = 15;
            rosforce.z = 0;
            // rosforce = rosforce*-1;
            if(rosforce.x !=0.0 || rosforce.y !=0.0 || rosforce.z !=0.0 || float.IsNaN(rosforce.x))
            {  
                GetGeometryPoint(unityforce*-1, message.force);
               Publish(message);
                // Debug.Log("PUBLISHING OK");
            } else {
                GetGeometryPoint(Vector3.zero.Unity2Ros(), message.force);
                Publish(message);
                // Debug.Log("PUBLISHING ZERO");
            }

        }

        private void GetGeometryPoint(Vector3 force, MessageTypes.Geometry.Vector3 geometryPoint)
        {
            geometryPoint.x = force.x;
            geometryPoint.y = force.y;
            geometryPoint.z = force.z;
        }
    }
}

