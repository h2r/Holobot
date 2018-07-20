using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using HoloToolkit.Unity;

namespace RosSharp.RosBridgeClient {

    [RequireComponent(typeof(RosConnector))]
    public class MoveItGoalPublisher : MonoBehaviour {

        public string PlanTopic;
        public string ExecuteTopic;
        private readonly string RequestIdentityPoseTopic = "/holocontrol/identity_pose_request";
        private readonly string PublishRightGripperCommandTopic = "/holocontrol/right_gripper_command";
        private readonly string PublishLeftGripperCommandTopic = "/holocontrol/left_gripper_command";
        private readonly string PublishWaypointPoseStampedTopic = "/holocontrol/waypoint_pose_stamped";

        public GameObject UrdfModel; // the root gameobject of your robot

        private RosSocket rosSocket;
        private int planPublicationId;
        private int executePublicationId;
        private int requestIdentityPlanPublicationId;
        private int rightGripperCommandPublicationId;
        private int leftGripperCommandPublicationId;
        private int waypointPoseStampedPublicationId;

        // Use this for initialization
        void Start() {
            rosSocket = GetComponent<RosConnector>().RosSocket;
            planPublicationId = rosSocket.Advertise(PlanTopic, "ros_reality_bridge_msgs/MoveitTarget");
            executePublicationId = rosSocket.Advertise(ExecuteTopic, "std_msgs/String");
            requestIdentityPlanPublicationId = rosSocket.Advertise(RequestIdentityPoseTopic, "std_msgs/String");
            rightGripperCommandPublicationId = rosSocket.Advertise(PublishRightGripperCommandTopic, "std_msgs/String");
            leftGripperCommandPublicationId = rosSocket.Advertise(PublishLeftGripperCommandTopic, "std_msgs/String");
            waypointPoseStampedPublicationId = rosSocket.Advertise(PublishWaypointPoseStampedTopic, "geometry_msgs/PoseStamped");
            rosSocket.Subscribe("/move_base/GlobalPlanner/plan", "nav_msgs/Path", NavPathHandler);
            ResetBackend();
        }

        private void NavPathHandler(Message message) {
            NavPath path = (NavPath)message;
            if (path.poses.Count > 0) {
                Debug.Log("Num poses in path: " + path.poses.Count);
            }
            WaypointManager.Instance.PathPoses = path.poses;
            WaypointManager.Instance.PathPosesRefreshed = true;
        }

        public void RequestIdentityPlan() { // ask moveit_movo.py to create a non-moving plan, to stop MovoShadow from moving.
            rosSocket.Publish(requestIdentityPlanPublicationId, new StandardString { data = "identity plan pls" });
        }

        public void PublishPoseStamped(GeometryPoseStamped poseStamped) {
            rosSocket.Publish(waypointPoseStampedPublicationId, poseStamped);
        }

        public void PublishPlan(MoveitTarget moveitTarget) {
            Debug.Log("Published moveitTarget x:" + moveitTarget.right_arm.pose.position.x.ToString());
            rosSocket.Publish(planPublicationId, moveitTarget);
        }

        public void PublishMove() {
            Debug.Log("Sending execute message");
            rosSocket.Publish(executePublicationId, new StandardString { data = "hoi! (pls move)" });
        }

        public void PublishGripperCommand(string gripperSide, string command) {
            Debug.Assert(command == "open" || command == "close");
            Debug.Assert(gripperSide == "right" || gripperSide == "left");
            if (gripperSide == "right") {
                rosSocket.Publish(rightGripperCommandPublicationId, new StandardString { data = command });
            }
            else if (gripperSide == "left") {
                rosSocket.Publish(leftGripperCommandPublicationId, new StandardString { data = command });
            }
        }

        public void ResetBackend() {
            RequestIdentityPlan();
        }
    }
}
