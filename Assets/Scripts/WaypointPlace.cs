using UnityEngine;
using UnityEngine.UI;
using UnityEngine.EventSystems;
using System;

namespace HoloToolkit.Unity {
    //namespace Academy.HoloToolkit.Unity {
    public class WaypointPlace : MonoBehaviour {
        GameObject waypointObj;
        GameObject coordTextObj;
        Waypoint thisWaypoint;
        Text coordText;

        private void Start() {
            thisWaypoint = null;
        }

        // Update is called once per frame
        void Update() {
            if (StateManager.Instance.CurrentState != StateManager.State.WaypointState) {
                return;
            }
            if (name == "Waypoint0") {
                return;
            }
            var pos = transform.position;
            transform.position = new Vector3(pos.x, StateManager.Instance.FloorY, pos.z);
            Debug.Assert(StateManager.Instance.CurrentState == StateManager.State.WaypointState);
            //Waypoint curr_waypoint = WaypointManager.Instance.GetLastWaypoint();
            if (thisWaypoint == null) {
                //thisWaypoint = WaypointManager.Instance.GetLastWaypoint();
                thisWaypoint = WaypointManager.Instance.LastWaypoint;
                Debug.Log(name + " waypoint set!");
            }
            waypointObj = thisWaypoint.WaypointObj;
            Debug.Assert(waypointObj != null);

            //coordTextObj = WaypointManager.Instance.GetLastWaypoint().CoordTextObj;
            coordTextObj = thisWaypoint.CoordTextObj;
            Debug.Assert(coordTextObj != null);
            coordText = coordTextObj.GetComponent<Text>();
            thisWaypoint.UpdatePose(calibThetaOffset: -StateManager.Instance.CalibrateThetaOffset);
            Debug.Log(name + " Pose updated!");
            Pose pose = thisWaypoint.Pose;
            string msg = string.Format("{0}\n({1}, {2}, {3})", waypointObj.name, Math.Round(pose.X, 1), Math.Round(pose.Y, 1), Math.Round(pose.Theta, 1));
            //Vector2 predictedUnityPosition = thisWaypoint.GetUnityCoords();
            //Vector3 actualUnityPosition = gameObject.transform.position;
            //coordText.text = "ROS: " + msg + String.Format("\nPredict: ({0},{1})\nReal: ({2},{3})", predictedUnityPosition.x, predictedUnityPosition.y,
            //    actualUnityPosition.x, actualUnityPosition.z);
            coordText.text = msg;
        }
    }
}