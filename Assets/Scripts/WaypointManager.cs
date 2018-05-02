using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Academy.HoloToolkit.Unity {
    public class WaypointManager : Singleton<WaypointManager> {
        public int WaypointInd { get; set; }
        public List<Waypoint> Waypoints { get; private set; }
        private GameObject WaypointTemplate;
        // Use this for initialization

        void Awake() {
            Debug.Log("Initialized WaypointManager");
            WaypointTemplate = GameObject.Find("Waypoint0");
            //WaypointTemplate.GetComponent<Renderer>().enabled = false;
            Waypoints = new List<Waypoint>();
            ClearWaypoints();
        }

        public void ClearWaypoints() {
            foreach (Waypoint wp in Waypoints) {
                Destroy(wp.WaypointObj);
            }
            Waypoints = new List<Waypoint>();
            WaypointInd = 0;
        }

        public void InitializeWaypoints() {
            ClearWaypoints();
            //WaypointTemplate.GetComponent<Renderer>().enabled = true;
            AddWaypoint();
        }

        public GameObject GetCoordTextObj(GameObject waypointObj) {
            return waypointObj.transform.GetChild(0).gameObject.transform.GetChild(0).gameObject;
        }

        public void AddWaypoint() {
            GameObject waypointObj = Instantiate(WaypointTemplate);
            waypointObj.GetComponent<Renderer>().enabled = true; // if doesn't work, enable template then diable immediately after
            if (Waypoints.Count > 0) {
                waypointObj = Instantiate(GetLastWaypoint().WaypointObj);
            }
            Waypoints.Add(new Waypoint(waypointObj, WaypointInd));
            WaypointInd++;
            waypointObj.name = String.Format("Waypoint{0}", WaypointInd);
            GameObject coordTextObj = GetCoordTextObj(waypointObj);
            Debug.Assert(coordTextObj != null);
            coordTextObj.name = String.Format("WaypointCoord{0}", WaypointInd);
        }

        public Waypoint GetLastWaypoint() {
            if (Waypoints.Count == 0) {
                return null;
            }
            return Waypoints[Waypoints.Count - 1];
        }

        private void Update() {
            if (StateManager.Instance.CurrentState != StateManager.State.WaypointState) {
                return;
            }
            if (Waypoints.Count == 4) {
                if (StateManager.Instance.UnityDebugMode) {
                    InitializeWaypoints();
                }
                else {
                    Destroy(GetLastWaypoint().WaypointObj);
                    Waypoints.RemoveAt(Waypoints.Count - 1);
                    StateManager.Instance.CurrentState = StateManager.State.NavigatingState;
                }
            }
        }

    }
}