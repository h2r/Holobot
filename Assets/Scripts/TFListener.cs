//using UnityEngine;
//using System;
//using System.Collections.Generic;
//using RosSharp.RosBridgeClient;

////namespace Academy.HoloToolkit.Unity {
//namespace HoloToolkit.Unity {
//    public class TFListener : MonoBehaviour {
//        private UniversalWebsocketClient wsc;
//        private readonly string movoStateTopic = "holocontrol/ros_movo_state_pub";
//        private readonly string movoPoseTopic = "holocontrol/ros_movo_pose_pub";
//        private readonly string unityWaypointPubTopic = "holocontrol/unity_waypoint_pub";
//        private readonly string movoStateRequestTopic = "holocontrol/movo_state_request";
//        private readonly string movoPoseRequestTopic = "holocontrol/movo_pose_request";
//        private bool currentlyNavigating = false;
//        private bool hasPublishedWaypoints = false;
//        private int frameCounter;
//        private int frameCountStart;
//        private UnityRosBridge moveItGoalPublisher;

//        public float scale = 1f;
//        // Use this for initialization
////        void Start() {
////            //if build
////            GameObject wso = GameObject.Find("WebsocketClient");
////#if UNITY_EDITOR
////            wsc = wso.GetComponent<WebsocketClient>();
////#else
////        wsc = wso.GetComponent<UWPWebSocketClient>();
////#endif
////            wsc.Subscribe(movoStateTopic, "std_msgs/String", "none", 0);
////            wsc.Subscribe(movoPoseTopic, "std_msgs/String", "none", 0);
////            wsc.Advertise(unityWaypointPubTopic, "std_msgs/String");
////            wsc.Advertise(movoStateRequestTopic, "std_msgs/String");
////            wsc.Advertise(movoPoseRequestTopic, "std_msgs/String");
////            currentlyNavigating = false;
////            hasPublishedWaypoints = false;
////            frameCounter = 0;
////            moveItGoalPublisher = GameObject.Find("RosConnector").GetComponent<UnityRosBridge>();
////        }

//        //private string ExtractROSMessage(string topic_input) {
//        //    string[] components = topic_input.Split(':');
//        //    foreach (string component in components) {
//        //        if (component.Contains("\"op\"")) {
//        //            var dataPair = component.Split('}');
//        //            var msg = dataPair[0].Trim(new Char[] { ' ', '"' });
//        //            return msg;
//        //        }
//        //    }
//        //    return null;
//        //}

//        //private void PublishWaypoints() {
//        //    Debug.Log("Publishing waypoints!");
//        //    List<Waypoint> waypoints = WaypointManager.Instance.Waypoints;
//        //    int num_waypoints = waypoints.Count;
//        //    Debug.Log(num_waypoints + " waypoints exist.");
//        //    string coord_message = "";
//        //    foreach (Waypoint waypoint in waypoints) {
//        //        Debug.Assert(waypoint != null);
//        //        HoloPose waypoint_pose = waypoint.Pose;
//        //        string coord_str = waypoint_pose.X.ToString() + "," + waypoint_pose.Y.ToString() + "," + waypoint_pose.Theta.ToString();
//        //        coord_message += coord_str + ";";
//        //    }
//        //    wsc.Publish(unityWaypointPubTopic, coord_message.TrimEnd(';'));
//        //    Debug.Log("Published: " + coord_message);
//        //    currentlyNavigating = true;
//        //    hasPublishedWaypoints = true;
//        //    StateManager.Instance.CurrentState = StateManager.State.NavigatingState;
//        //    frameCountStart = frameCounter;
//        //    //if (StateManager.Instance.UnityDebugMode) {
//        //    //    StateManager.Instance.MovoState = "navigating";
//        //    //}
//        //}

//        //public void UpdateMovoROSPose() {
//        //    if (StateManager.Instance.UnityDebugMode) {
//        //        StateManager.Instance.MovoROSPose = new HoloPose(0, 0, 0);
//        //        return;
//        //    }
//        //    wsc.Publish(movoPoseRequestTopic, "");
//        //    //Debug.Log("Published PoseRequestTopic");
//        //    string ros_msg = wsc.messages[movoPoseTopic];
//        //    //Debug.Log("Received Pose response");
//        //    while (ros_msg == null) {
//        //        Debug.Log("Waiting for message...");
//        //        ros_msg = wsc.messages[movoPoseTopic];
//        //    }
//        //    //Debug.Log(ros_msg);
//        //    List<string> poseStr = new List<string>(ExtractROSMessage(ros_msg).Split(','));
//        //    //Debug.Log(poseStr);
//        //    List<float> pose = new List<float> { Convert.ToSingle(poseStr[0]), Convert.ToSingle(poseStr[1]), Convert.ToSingle(poseStr[2]) };
//        //    Debug.Assert(pose.Count == 3);
//        //    StateManager.Instance.MovoROSPose = new HoloPose(pose[0], pose[1], -pose[2]); // Unity rotation goes clockwise
//        //    //Debug.Log("MovoROSPose updated!");
//        //}

//        //private void UpdateMovoState() {
//        //    if (StateManager.Instance.UnityDebugMode) {
//        //        return;
//        //    }
//        //    //Debug.Log("Updating MovoState");
//        //    wsc.Publish(movoStateRequestTopic, "");
//        //    StateManager.Instance.MovoState = ExtractROSMessage(wsc.messages[movoStateTopic]);
//        //    //Debug.Log("MovoROSState updated!");
//        //}

//        //void Update() {
//        //    //Debug.Log("Current state: " + StateManager.Instance.CurrentState);
//        //    frameCounter++;
//        //    try {
//        //        //UpdateMovoROSPose();
//        //        //UpdateMovoState();
//        //    }
//        //    catch {
//        //        //Debug.Log("Bluh");
//        //        return;
//        //    }

//        //    if (StateManager.Instance.CurrentState != StateManager.State.NavigatingState) {
//        //        return;
//        //    }
//        //    if (StateManager.Instance.MovoState == "standby" && hasPublishedWaypoints) {
//        //        if (frameCounter - frameCountStart < 40) { // Give ROS enough time to receive waypoints
//        //            frameCounter++;
//        //            return;
//        //        }
//        //        frameCounter = 0;
//        //        currentlyNavigating = false;
//        //        hasPublishedWaypoints = false;
//        //        StateManager.Instance.TransitionToStandbyState();
//        //    }
//        //    else if (StateManager.Instance.MovoState == "standby" && !currentlyNavigating && !hasPublishedWaypoints) {
//        //        Debug.Log("Check");
//        //        PublishWaypoints();
//        //    }
//        //}

//        //Vector3 RosToUnityPositionAxisConversion(Vector3 rosIn) {
//        //    return new Vector3(-rosIn.x, rosIn.z, -rosIn.y) * scale + StateManager.Instance.MovoOffset;// + robot.transform.position;	
//        //}

//        //Quaternion RosToUnityQuaternionConversion(Quaternion rosIn) {
//        //    return new Quaternion(rosIn.x, -rosIn.z, rosIn.y, rosIn.w);
//        //}


//    }
//}