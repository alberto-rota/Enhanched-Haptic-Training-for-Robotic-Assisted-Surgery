/*
© Siemens AG, 2017-2019
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

using System;
using System.Threading;
using RosSharp.RosBridgeClient.Protocols;
using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class RosConnector : MonoBehaviour
    {
        public int SecondsTimeout = 10;

        public RosSocket RosSocket { get; private set; }
        public RosSocket.SerializerEnum Serializer;
        public Protocol protocol;
        public string RosBridgeServerUrl = "ws://192.168.1.1:9090";
        public bool debug = false;
        public bool connected = false;
        string whoisconnecting;

        public ManualResetEvent IsConnected { get; private set; }

        public virtual void Awake()
        {
            RosBridgeServerUrl = GameObject.FindWithTag("ROBOT").GetComponent<ROSBRIDGE_SERVER_URI>().getIP();
            IsConnected = new ManualResetEvent(false);
            new Thread(ConnectAndWait).Start();
            whoisconnecting=gameObject.name;
        }

        protected void ConnectAndWait()
        {
            RosSocket = ConnectToRos(protocol, RosBridgeServerUrl, OnConnected, OnClosed, Serializer);
            if (!IsConnected.WaitOne(SecondsTimeout * 1000) && debug)
                Debug.LogWarning(whoisconnecting+": FAILED to connect to RosBridge at @ " + RosBridgeServerUrl);

        }

        public static RosSocket ConnectToRos(Protocol protocolType, string serverUrl, EventHandler onConnected = null, EventHandler onClosed = null, RosSocket.SerializerEnum serializer = RosSocket.SerializerEnum.Microsoft)
        {
            IProtocol protocol = ProtocolInitializer.GetProtocol(protocolType, serverUrl);
            protocol.OnConnected += onConnected;
            protocol.OnClosed += onClosed;

            return new RosSocket(protocol, serializer);
        }


        // !!!! Change 'OnApplicationQuit()' to 'OnDestroy()' !!!!
        // Otherwise RosSocket are not closed when the scene is changed: this crowds the networks and produces 
        // a lot of lag on the ROS communication.
        private void OnDestroy()
        {
            RosSocket.Close();
        }

        private void OnConnected(object sender, EventArgs e)
        {
            IsConnected.Set();
            if (debug){
                Debug.Log(whoisconnecting+": CONNECTED to RosBridge @ " + RosBridgeServerUrl);
                connected = true;
                GameObject.Find("/Text/CanvasROS/Image").GetComponent<UnityEngine.UI.Image>().color = Color.green;
            }
        }

        private void OnClosed(object sender, EventArgs e)
        {
            IsConnected.Reset();
            if (debug){
                Debug.Log(whoisconnecting+": DISCONNECTED from RosBridge @ " + RosBridgeServerUrl);
            }
            connected = false;
            GameObject.Find("/Text/CanvasROS/Image").GetComponent<UnityEngine.UI.Image>().color = Color.red;
        }
    }
}