using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
using Academy.HoloToolkit.Unity;

public class TFListener : MonoBehaviour {

    //private UWPWebSocketClient UWPwsc;
    //private WebsocketClient wsc;
    private UniversalWebsocketClient wsc;
    private string unityTopic = "holocontrol/unity_waypoint_pub";
    private string movoStateTopic = "holocontrol/ros_movo_state_pub";
    private string movoPoseTopic = "holocontrol/ros_movo_pose_pub";
    private string movoStateRequestTopic = "holocontrol/movo_state_request";
    private string movoPoseRequestTopic = "holocontrol/movo_pose_request";
    private bool currentlyNavigating = false;
    private bool hasPublishedWaypoints = false;

    public float scale = 1f;
    // Use this for initialization
    void Start() {
        //if build
        GameObject wso = GameObject.Find("WebsocketClient");
#if UNITY_EDITOR
        wsc = wso.GetComponent<WebsocketClient>();
#else
        wsc = wso.GetComponent<UWPWebSocketClient>();
#endif
        wsc.Subscribe(movoStateTopic, "std_msgs/String", "none", 0);
        wsc.Subscribe(movoPoseTopic, "std_msgs/String", "none", 0);
        wsc.Advertise(unityTopic, "std_msgs/String");
        wsc.Advertise(movoStateRequestTopic, "std_msgs/String");
        wsc.Advertise(movoPoseRequestTopic, "std_msgs/String");
        Debug.Log("Subscribed!");
        currentlyNavigating = false;
        hasPublishedWaypoints = false;
    }

    private string GetROSMessage(string topic_input) {
        string[] components = topic_input.Split(':');
        foreach (string component in components) {
            if (component.Contains("\"op\"")) {
                var dataPair = component.Split('}');
                var msg = dataPair[0].Trim(new Char[] { ' ', '"' });
                return msg;
            }
        }
        return null;
    }

    private void PublishWaypoints() {
        Debug.Log("Publishing waypoints!");
        List<Waypoint> waypoints = WaypointManager.Instance.Waypoints;
        int num_waypoints = waypoints.Count;
        Debug.Assert(num_waypoints == 2);
        string coord_message = "";
        foreach (Waypoint waypoint in waypoints) {
            Vector2 waypoint_coords = waypoint.GetCoords();
            string coord_str = waypoint_coords[0].ToString() + "," + waypoint_coords[1].ToString();
            coord_message += coord_str + ";";
        }
        wsc.Publish(unityTopic, coord_message.TrimEnd(';'));
        Debug.Log("Published: " + coord_message);
        WaypointManager.Instance.ClearWaypoints();
        currentlyNavigating = true;
        hasPublishedWaypoints = true;
        StateManager.Instance.CurrentState = StateManager.State.NavigatingState;
    }

    public string GetMovoPose() {
        wsc.Publish(movoPoseRequestTopic, "True");
        return GetROSMessage(wsc.messages[movoPoseTopic]);
    }

    private string GetMovoState() {
        wsc.Publish(movoStateRequestTopic, "True");
        return GetROSMessage(wsc.messages[movoStateTopic]);
    }

    //private void PrintDict(Dictionary<string,string> d) {
    //    foreach (KeyValuePair<string, string> kvp in d) {
    //        Debug.Log(String.Format("Key: {0}, Value: {1}", kvp.Key, kvp.Value));
    //    }
    //}

    void Update() {
        string movo_state;
        string movo_pose;
        try {
            movo_state = GetMovoState();
            movo_pose = GetMovoPose();
            Debug.Log("Movo pose: " + movo_pose);
        }
        catch {
            return;
        }
        
        if (StateManager.Instance.CurrentState != StateManager.State.NavigatingState) {
            return;
        }
        if (movo_state == "standby" && hasPublishedWaypoints) {
            currentlyNavigating = false;
            hasPublishedWaypoints = false;
            StateManager.Instance.TransitionedToWaypointState = true;
            StateManager.Instance.CurrentState = StateManager.State.WaypointState;
        }
        else if (movo_state == "standby" && !currentlyNavigating && !hasPublishedWaypoints) {
            PublishWaypoints();
        }
    }

    //Vector3 RosToUnityPositionAxisConversion(Vector3 rosIn) {
    //    return new Vector3(-rosIn.x, rosIn.z, -rosIn.y) * scale + StateManager.Instance.RobotOffset;// + robot.transform.position;	
    //}

    //Quaternion RosToUnityQuaternionConversion(Quaternion rosIn) {
    //    return new Quaternion(rosIn.x, -rosIn.z, rosIn.y, rosIn.w);
    //}


}
