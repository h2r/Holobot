using UnityEngine;
using UnityEngine.UI;
using UnityEngine.EventSystems;
using System;

namespace HoloToolkit.Unity {
//namespace Academy.HoloToolkit.Unity {
    public class WaypointPlace : MonoBehaviour {
        //bool placingEnabled;
        bool waypointPlaced;
        GameObject waypointObj;
        GameObject coordTextObj;
        Text coordText;

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
            //// Ensure that all waypoints are at floor level
            //foreach (Waypoint wp in WaypointManager.Instance.Waypoints) {
            //    var pos = wp.WaypointObj.transform.position;
            //    Debug.Assert(StateManager.Instance.FloorY != -99);
            //    wp.WaypointObj.transform.position = new Vector3(pos.x, StateManager.Instance.FloorY, pos.z);
            //}
            Debug.Assert(StateManager.Instance.CurrentState == StateManager.State.WaypointState);
            Waypoint curr_waypoint = WaypointManager.Instance.GetLastWaypoint();
            waypointObj = curr_waypoint.WaypointObj;
            Debug.Assert(waypointObj != null);

            //Utils.RaycastPlace(Camera.main, waypointObj);
            coordTextObj = WaypointManager.Instance.GetLastWaypoint().CoordTextObj;
            Debug.Assert(coordTextObj != null);
            coordText = coordTextObj.GetComponent<Text>();
            Pose pose = curr_waypoint.GetPose();
            string msg = string.Format("{0}\n({1}, {2}, {3})", waypointObj.name, Math.Round(pose.X, 1), Math.Round(pose.Y, 1), Math.Round(pose.Theta, 1));
            coordText.text = msg;
        }
    }
}