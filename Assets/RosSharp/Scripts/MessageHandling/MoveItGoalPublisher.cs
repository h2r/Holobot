using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace RosSharp.RosBridgeClient {

    [RequireComponent(typeof(RosConnector))]
    public class MoveItGoalPublisher : MonoBehaviour {

        public string PlanTopic;
        public string ExecuteTopic;
        private readonly string RequestIdentityPoseTopic = "/holocontrol/identity_pose_request";

        public GameObject UrdfModel; // the root gameobject of your robot

        private RosSocket rosSocket;
        private int planPublicationId;
        private int executePublicationId;
        private int requestIdentityPlanPublicationId;

        // Use this for initialization
        void Start() {
            rosSocket = GetComponent<RosConnector>().RosSocket;
            planPublicationId = rosSocket.Advertise(PlanTopic, "ros_reality_bridge_msgs/MoveitTarget");
            executePublicationId = rosSocket.Advertise(ExecuteTopic, "std_msgs/String");
            requestIdentityPlanPublicationId = rosSocket.Advertise(RequestIdentityPoseTopic, "std_msgs/String");
            ResetBackend();
        }

        public void RequestIdentityPlan() { // ask moveit_movo.py to create a non-moving plan, to stop MovoShadow from moving.
            rosSocket.Publish(requestIdentityPlanPublicationId, new StandardString { data = "identity plan pls" });
        }

        public void PublishPlan(MoveitTarget moveitTarget) {
            //Debug.Log("Published moveitTarget x:" + moveitTarget.right_arm.pose.position.x.ToString());
            rosSocket.Publish(planPublicationId, moveitTarget);
        }

        public void PublishMove() {
            //Debug.Log("Sending execute message");
            rosSocket.Publish(executePublicationId, new StandardString { data = "hoi! (pls move)" });
        }

        void PlanHandler(object args) {
            return;
        }

        public void ResetBackend() {
            RequestIdentityPlan();
            //rosSocket.Publish(planPublicationId, new MoveitTarget());
        }
    }
}
