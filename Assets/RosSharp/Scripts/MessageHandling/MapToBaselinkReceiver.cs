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

using System;
using UnityEngine;

namespace RosSharp.RosBridgeClient {
    public class MapToBaselinkReceiver : MessageReceiver {
        public override Type MessageType { get { return (typeof(StandardString)); } }

        // Variables for the metadata
        private String posQuatMessage;
        private String[] posAndQuatsList;
        public float robotX;
        public float robotY;
        public float robotZ;
        public float robotQ1;
        public float robotQ2;
        public float robotQ3;
        public float robotQ4;

        private bool isMessageReceived;

        private void Awake() {
            MessageReception += ReceiveMessage;
        }
        private void Start() {
            // Make sure this thing only activates after the robot has been calibrated
        }
        private void Update() {
            if (isMessageReceived)
                ProcessMessage();
        }
        private void ReceiveMessage(object sender, MessageEventArgs e) {
            posQuatMessage = ((StandardString)e.Message).data;
            Debug.Log(posQuatMessage);
            posAndQuatsList = posQuatMessage.Split(',');
            robotX = float.Parse(posAndQuatsList[0]);
            robotY = float.Parse(posAndQuatsList[1]);
            robotZ = float.Parse(posAndQuatsList[2]);
            robotQ1 = float.Parse(posAndQuatsList[0]);
            robotQ2 = float.Parse(posAndQuatsList[1]);
            robotQ3 = float.Parse(posAndQuatsList[2]);
            robotQ4 = float.Parse(posAndQuatsList[3]);
            isMessageReceived = true;
        }

        private void ProcessMessage() {
            //robotMap.transform.localScale = new Vector3(resolution * width, 0.1f, resolution * height);
            isMessageReceived = false;
        }
    }
}
