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
        public bool safetyOverride = false;
        private MessageTypes.Geometry.Wrench message;
        private MessageTypes.Geometry.Vector3 geometryPoint;
        private bool exiting = false;

        protected override void Start()
        {
            base.Start();
            InitializeMessage();
            Publish(message);
        }

        private void FixedUpdate()
        {
            UpdateMessage();
        }

        private void InitializeMessage()
        {
            message = new MessageTypes.Geometry.Wrench();
            message.force = new MessageTypes.Geometry.Vector3();
            message.torque = new MessageTypes.Geometry.Vector3();
        }
        private void UpdateMessage()
        {
            Vector3 unityforce;
            Vector3 rosforce = Vector3.zero;

            if (GameObject.FindWithTag("ROBOT").GetComponent<SumForces>() != null) {
                unityforce = GameObject.FindWithTag("ROBOT").GetComponent<SumForces>().totalForce*Global.assistance;
             }else {
                unityforce = Vector3.zero;
            }

            Vector3 tip = GameObject.Find(Global.tooltip_path).transform.position;
            // Global.Arrow(tip, tip+unityforce*0.3f, Color.blue);

            if (!safetyOverride && GameObject.FindWithTag("ROBOT").GetComponent<SumForces>().enabled) {
                if(unityforce.x !=0.0 || unityforce.y !=0.0 || unityforce.z !=0.0 || float.IsNaN(unityforce.x))
                {  
                    Vector3toMessage(unityforce.Unity2Ros(), message.force);
                    Publish(message);
                } else {
                    Vector3toMessage(Vector3.zero.Unity2Ros(), message.force);
                    Publish(message);
                }
            }else if (safetyOverride || exiting) {
                Vector3toMessage(Vector3.zero.Unity2Ros(), message.force);
                Publish(message);
            }
        }

        // void OnApplicationQuit() {
        //     exiting = true;
        //     if (safetyOverride || exiting) {
        //         Vector3toMessage(Vector3.zero.Unity2Ros(), message.force);
        //         Publish(message);
        //     }
        // }

        private void Vector3toMessage(Vector3 force, MessageTypes.Geometry.Vector3 geometryPoint)
        {
            geometryPoint.x = force.x;
            geometryPoint.y = force.y;
            geometryPoint.z = force.z;
        }
    }
}

