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
    public class PedalPlusSubscriber : UnitySubscriber<MessageTypes.Sensor.Joy>
    {
        public bool pressed;
        public bool listening; 
        float lastT;
        private int flag;
        private bool isMessageReceived;

        protected override void Start()
        {
            base.Start();
            lastT = Time.realtimeSinceStartup;
            pressed = false;
            listening = false;
        }

        protected override void ReceiveMessage(MessageTypes.Sensor.Joy message)
        {
            flag = message.buttons[0];
        }

        private void Update()
        {
            if ((Time.realtimeSinceStartup - lastT) > 1f) {
                listening = true;
            } else listening = false;

            if (flag==1 && listening) {
                pressed = true;
                lastT = Time.realtimeSinceStartup;
            } else pressed = false;
        }
    }
}