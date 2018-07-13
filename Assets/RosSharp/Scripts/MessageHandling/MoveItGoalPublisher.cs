using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace RosSharp.RosBridgeClient {

    [RequireComponent(typeof(RosConnector))]
    public class MoveItGoalPublisher : MonoBehaviour {

        public string PlanTopic;
        public string ExecuteTopic;

        public GameObject UrdfModel; // the root gameobject of your robot

        private RosSocket rosSocket;
        private int planPublicationId;
        private int executePublicationId;

        // Use this for initialization
        void Start() {
            rosSocket = GetComponent<RosConnector>().RosSocket;
            Debug.Log("PLANTOPIC: " + PlanTopic);
            planPublicationId = rosSocket.Advertise(PlanTopic, "ros_reality_bridge_msgs/MoveitTarget");
            executePublicationId = rosSocket.Advertise(ExecuteTopic, "std_msgs/String");
            ResetBackend();
            //FadeManager.AssertIsInitialized();
        }

        public void PublishPlan(MoveitTarget moveitTarget) {
            //Debug.Log("Published moveitTarget x:" + moveitTarget.right_arm.pose.position.x.ToString());
            rosSocket.Publish(planPublicationId, moveitTarget);
        }

        public void PublishMove() {
            Debug.Log("Sending execute message");
            rosSocket.Publish(executePublicationId, new StandardString { data = "hoi! (pls move)" });
        }

        void PlanHandler(object args) {
            return;
        }

        public void ResetBackend() {
            rosSocket.Publish(planPublicationId, new MoveitTarget());
        }
    }
}
