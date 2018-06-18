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

        private void Start() {
            Debug.Log("WaypointPlace Start()");
            Reset();
        }

        private void Reset() {
            //placingEnabled = true;
            //waypointPlaced = false;
            Debug.Log("WaypointPlace Reset()");
            WaypointManager.Instance.InitializeWaypoints();
        }

        public void Disable() {
            //WaypointManager.Instance.ClearWaypoints();
            //placingEnabled = false;
        }

        // Called by GazeGestureManager when the user performs a Select gesture
        //void OnSelect() {
        //    if (waypointPlaced) {
        //        waypointPlaced = false;
        //        WaypointManager.Instance.AddWaypoint();
        //        Debug.Assert(placingEnabled = true);
        //    }
        //}

        // Update is called once per frame
        void Update() {
            //Debug.Log("Update");
            //if (StateManager.Instance.TransitionedToWaypointState) {
            //    Debug.Log("Transitioned to WaypointState");
            //    Reset();
            //    StateManager.Instance.TransitionedToWaypointState = false;
            //}
            if (StateManager.Instance.CurrentState != StateManager.State.WaypointState) {
                Disable();
                return;
            }
            // Ensure that all waypoints are at floor level
            foreach (Waypoint wp in WaypointManager.Instance.Waypoints) {
                var pos = wp.WaypointObj.transform.position;
                Debug.Assert(StateManager.Instance.FloorY != -99);
                wp.WaypointObj.transform.position = new Vector3(pos.x, StateManager.Instance.FloorY, pos.z);
            }
            //if (!placingEnabled) {
            //    //Reset();
            //}
            //Debug.Assert(placingEnabled);
            Debug.Assert(StateManager.Instance.CurrentState == StateManager.State.WaypointState);
            Waypoint curr_waypoint = WaypointManager.Instance.GetLastWaypoint();
            waypointObj = curr_waypoint.WaypointObj;
            Debug.Assert(waypointObj != null);

            //m_EventSystem.SetSelectedGameObject(waypointObj);

            //Utils.RaycastPlace(Camera.main, waypointObj);
            coordTextObj = WaypointManager.Instance.GetLastWaypoint().CoordTextObj;
            Debug.Assert(coordTextObj != null);
            coordText = coordTextObj.GetComponent<Text>();
            //Vector2 coords = curr_waypoint.GetCoords();
            //string msg = string.Format("{0}\n({1}, {2})", waypointObj.name, Math.Round(coords[0], 1), Math.Round(coords[1], 1));
            Pose pose = curr_waypoint.GetPose();
            string msg = string.Format("{0}\n({1}, {2}, {3})", waypointObj.name, Math.Round(pose.X, 1), Math.Round(pose.Y, 1), Math.Round(pose.Theta, 1));
            coordText.text = msg;
            //Debug.Log(msg);
            //if (!waypointPlaced) {
            //    waypointPlaced = true;
            //}
        }
    }
}