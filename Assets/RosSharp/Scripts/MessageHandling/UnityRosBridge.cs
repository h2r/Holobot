using UnityEngine;
using HoloToolkit.Unity;
using System.Collections.Generic;
using System;

namespace RosSharp.RosBridgeClient {

    [RequireComponent(typeof(RosConnector))]
    public class UnityRosBridge : MonoBehaviour {

        private readonly string MoveItPlanTopic = "/ros_reality/goal_pose";
        private readonly string MoveItExecuteTopic = "/ros_reality/move_to_goal";
        private readonly string RequestIdentityPoseTopic = "/holocontrol/identity_pose_request";
        private readonly string PublishRightGripperCommandTopic = "/holocontrol/right_gripper_command";
        private readonly string PublishLeftGripperCommandTopic = "/holocontrol/left_gripper_command";
        //private readonly string PublishWaypointPoseStampedTopic = "/holocontrol/waypoint_pose_stamped";
        //private readonly string InitializeMovocontrolRequestTopic = "/holocontrol/init_movocontrol_request";
        private readonly string WaypointPubTopic = "/holocontrol/unity_waypoint_pub";

        //public GameObject UrdfModel; // the root gameobject of your robot

        private RosSocket rosSocket;
        private int planPublicationId;
        private int executePublicationId;
        private int requestIdentityPlanPublicationId;
        private int rightGripperCommandPublicationId;
        private int leftGripperCommandPublicationId;
        //private int waypointPoseStampedPublicationId;
        private int initializeMovocontrolPublicationId;
        private int waypointPubTopicPublicationId;

        // Use this for initialization
        void Start() {
            rosSocket = GetComponent<RosConnector>().RosSocket;
            planPublicationId = rosSocket.Advertise(MoveItPlanTopic, "ros_reality_bridge_msgs/MoveitTarget");
            executePublicationId = rosSocket.Advertise(MoveItExecuteTopic, "std_msgs/String");
            requestIdentityPlanPublicationId = rosSocket.Advertise(RequestIdentityPoseTopic, "std_msgs/String");
            rightGripperCommandPublicationId = rosSocket.Advertise(PublishRightGripperCommandTopic, "std_msgs/String");
            leftGripperCommandPublicationId = rosSocket.Advertise(PublishLeftGripperCommandTopic, "std_msgs/String");
            //waypointPoseStampedPublicationId = rosSocket.Advertise(PublishWaypointPoseStampedTopic, "geometry_msgs/PoseStamped");
            //initializeMovocontrolPublicationId = rosSocket.Advertise(InitializeMovocontrolRequestTopic, "std_msgs/String");
            waypointPubTopicPublicationId = rosSocket.Advertise(WaypointPubTopic, "nav_msgs/Path");
            //rosSocket.Subscribe("/holocontrol/simulated_nav_path", "nav_msgs/Path", NavPathHandler);
            rosSocket.Subscribe("/move_base/MovocontrolGlobalPlanner/plan", "nav_msgs/Path", NavPathHandler);
            rosSocket.Subscribe("/holocontrol/ros_movo_pose_pub", "std_msgs/String", MovoPoseHandler);
            rosSocket.Subscribe("/holocontrol/nav_finished", "std_msgs/String", NavCompleteHandler);
            //rosSocket.Publish(initializeMovocontrolPublicationId, new StandardString());
            RequestIdentityPlan();
            Debug.Log("UnityRosBridge initialized!");
        }

        private void Update() {

        }

        private void NavPathHandler(Message message) {
            NavPath path = (NavPath)message;
            if (path.poses.Count > 0) {
                Debug.Log("Num poses in path: " + path.poses.Count);
                WaypointManager.Instance.PathPoses = path.poses;
                WaypointManager.Instance.PathPosesRefreshed = true;
            }
        }

        private void MovoPoseHandler(Message message) {
            StandardString movoPose = (StandardString)message;
            List<string> poseStr = new List<string>(movoPose.data.Split(','));
            List<float> pose = new List<float> { Convert.ToSingle(poseStr[0]), Convert.ToSingle(poseStr[1]), Convert.ToSingle(poseStr[2]) };
            Debug.Assert(pose.Count == 3);
            StateManager.Instance.MovoROSPose = new HoloPose(pose[0], pose[1], -pose[2]); // Unity rotation goes clockwise
        }

        private void NavCompleteHandler(Message message) {
            var currState = StateManager.Instance.CurrentState;
            Debug.Assert(currState == StateManager.State.NavigatingState, "NavCompleteHandler called while in state " + currState);
            StateManager.Instance.NavigationComplete = true;
            //StateManager.Instance.TransitionToStandbyState();
        }

        public void PublishWaypoints() {
            Debug.Log("Publishing waypoints!");
            List<Waypoint> waypoints = WaypointManager.Instance.Waypoints;
            if (waypoints.Count == 0) {
                return;
            }
            NavPath waypointPath = new NavPath {
                header = new StandardHeader {
                    frame_id = "/map"
                },
                poses = new List<GeometryPoseStamped>()
            };
            foreach (Waypoint waypoint in waypoints) {
                waypoint.UpdatePose();
                waypointPath.poses.Add(waypoint.ROSPoseStamped);
            }
            rosSocket.Publish(waypointPubTopicPublicationId, waypointPath);
        }

        public void RequestIdentityPlan() { // ask moveit_movo.py to create a non-moving plan, to stop MovoShadow from moving.
            rosSocket.Publish(requestIdentityPlanPublicationId, new StandardString());
        }

        //public void PublishWaypointPoseStamped(GeometryPoseStamped poseStamped) {
        //    rosSocket.Publish(waypointPoseStampedPublicationId, poseStamped);
        //}

        public void PublishPlan(MoveitTarget moveitTarget) {
            //Debug.Log("Published moveitTarget x:" + moveitTarget.right_arm.pose.position.x.ToString());
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
    }
}
