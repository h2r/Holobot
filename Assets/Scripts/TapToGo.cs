// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License. See LICENSE in the project root for license information.

using System;
using System.IO;
using UnityEngine;
using HoloToolkit.Unity.InputModule;
using HoloToolkit.Unity.InputModule.Utilities.Interactions;
using System.Collections.Generic;

/// <summary>
/// This class implements IInputClickHandler to handle the tap gesture.
/// It increases the scale of the object when tapped.
/// </summary>
/// 
namespace HoloToolkit.Unity {
    public class TapToGo : MonoBehaviour, IInputClickHandler {
        private UniversalWebsocketClient wsc;
        private readonly string unityWaypointPubTopic = "holocontrol/unity_waypoint_pub";
        public GameObject corner1;
        public GameObject corner2;

        public void Start() {
            GameObject wso = GameObject.Find("WebsocketClient");
#if UNITY_EDITOR
            wsc = wso.GetComponent<WebsocketClient>();
#else
        wsc = wso.GetComponent<UWPWebSocketClient>();
#endif
        }

        public void Update() {
            float xpose = (corner1.transform.position.x + corner2.transform.position.x) / 2;
            float zpose = (corner1.transform.position.z + corner2.transform.position.z) / 2;
            this.transform.position = new Vector3(xpose, StateManager.Instance.FloorY, zpose);
        }


        public void OnInputClicked(InputClickedEventData eventData) {
            // Increase the scale of the object just as a response.
            string coord_message = "";

            gameObject.transform.localScale += 0.05f * gameObject.transform.localScale;
            Pose staticROSPose = this.GetComponent<LabelPlace>().ROSpose;
            string coord_str = staticROSPose.X.ToString() + "," + staticROSPose.Y.ToString() + "," + staticROSPose.Theta.ToString();
            coord_message += coord_str + ";";
            wsc.Publish(unityWaypointPubTopic, coord_message.TrimEnd(';'));
            Debug.Log("Published: " + coord_message);
            //currentlyNavigating = true;
           // hasPublishedWaypoints = true;
            //StateManager.Instance.CurrentState = StateManager.State.NavigatingState;
            //frameCountStart = frameCounter;

            eventData.Use(); // Mark the event as used, so it doesn't fall through to other handlers.
        }
    }
}