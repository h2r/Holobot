using UnityEngine;
using UnityEngine.UI;
using System;

namespace Academy.HoloToolkit.Unity {
    public class WaypointPlace : MonoBehaviour {
        bool placingEnabled;
        bool waypointPlaced;
        GameObject waypointObj;
        GameObject coordTextObj;
        Text coordText;

        private void Start() {
            Reset();
        }

        private void Reset() {
            SpatialMapping.Instance.DrawVisualMeshes = true;
            placingEnabled = true;
            waypointPlaced = false;
            WaypointManager.Instance.InitializeWaypoints();
            Debug.Assert(WaypointManager.Instance.Waypoints.Count == 1);
            Debug.Assert(StateManager.Instance.PlacingWaypoints == true);
        }

        public void Disable() {
            WaypointManager.Instance.ClearWaypoints();
            SpatialMapping.Instance.DrawVisualMeshes = false;
            placingEnabled = false;
        }

        // Called by GazeGestureManager when the user performs a Select gesture
        void OnSelect() {
            if (waypointPlaced) {
                waypointPlaced = false;
                if (StateManager.Instance.PlacingWaypoints == true) {
                    WaypointManager.Instance.AddWaypoint();
                    Debug.Assert(placingEnabled == true);
                }
            }
        }

        // Update is called once per frame
        void Update() {
            if (StateManager.Instance.CurrentState != StateManager.State.WaypointState) {
                Disable();
                return;
            }
            if (StateManager.Instance.PlacingWaypoints == false) {
                Disable();
                return;
            }
            if (!placingEnabled) {
                Reset();
            }
            Debug.Assert(placingEnabled);
            Debug.Assert(StateManager.Instance.CurrentState == StateManager.State.WaypointState);
            waypointObj = WaypointManager.Instance.GetLastWaypoint().WaypointObj;
            Utils.RaycastPlace(Camera.main, waypointObj);
            //var waypoints = WaypointManager.Instance.Waypoints;
            //coordTextObj = waypoints[waypoints.Count - 1].CoordTextObj;
            coordTextObj = WaypointManager.Instance.GetLastWaypoint().CoordTextObj;
            Debug.Assert(coordTextObj != null);
            coordText = coordTextObj.GetComponent<Text>();
            string msg = string.Format("{0}\n({1}, {2})", waypointObj.name, Math.Round(waypointObj.transform.position.x, 1), Math.Round(waypointObj.transform.position.z, 1));
            coordText.text = msg;
            if (!waypointPlaced) {
                waypointPlaced = true;
            }
        }
    }
}