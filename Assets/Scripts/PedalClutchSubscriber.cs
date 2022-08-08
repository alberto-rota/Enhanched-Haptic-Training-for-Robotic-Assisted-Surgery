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
    public class PedalClutchSubscriber : UnitySubscriber<MessageTypes.Sensor.Joy>
    {
        public bool pressed;
        private int flag;
        private bool isMessageReceived;

        protected override void Start()
        {
            base.Start();
            pressed = false;
        }

        protected override void ReceiveMessage(MessageTypes.Sensor.Joy message)
        {
            flag = message.buttons[0];
        }

        private void Update()
        {
            if (flag == 1) {
                pressed = true;
            } else pressed = false;
        }
    }
}