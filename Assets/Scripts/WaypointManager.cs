using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace HoloToolkit.Unity {
//namespace Academy.HoloToolkit.Unity {
    public class WaypointManager : Singleton<WaypointManager> {
        public int WaypointInd { get; set; }
        public List<Waypoint> Waypoints { get; private set; }
        private GameObject WaypointTemplate;
        private bool DebugStop;
        // Use this for initialization

        void Awake() {
            Debug.Log("Initialized WaypointManager");
            WaypointTemplate = GameObject.Find("Waypoint0");
            Waypoints = new List<Waypoint>();
            //ClearWaypoints();
            Debug.Log("WaypointManager Awake()");
            InitializeWaypoints();
            DebugStop = false;
        }

        public void ClearWaypoints() {
            foreach (Waypoint wp in Waypoints) {
                Destroy(wp.WaypointObj);
            }
            Waypoints = new List<Waypoint>();
            WaypointInd = 0;
        }

        public void InitializeWaypoints() {
            Debug.Log("InitializeWaypoints()");
            ClearWaypoints();
            AddWaypoint();
        }

        public GameObject GetCoordTextObj(GameObject waypointObj) {
            return waypointObj.transform.GetChild(0).gameObject.transform.GetChild(0).gameObject;
        }

        public void AddWaypoint() {
            Debug.Log("AddWaypoint()");
            GameObject waypointObj = Instantiate(WaypointTemplate); // This waypoint will eventually be destroyed, so Instantiate ensures that WaypointTemplate is always there.
            //if (Waypoints.Count == 0) {
            //    waypointObj = Instantiate(WaypointTemplate);
            //}
            ////waypointObj.GetComponent<Renderer>().enabled = true; // if doesn't work, enable template then disable immediately after
            //else {
            //    waypointObj = Instantiate(GetLastWaypoint().WaypointObj);
            //}
            if (StateManager.Instance.CurrentState == StateManager.State.WaypointState) { // If in WaypointState, then place waypoint in front of user.
                UtilFunctions.InitWaypointPos(Camera.main, waypointObj);
            }
            Waypoints.Add(new Waypoint(waypointObj, WaypointInd));
            WaypointInd++;
            waypointObj.name = String.Format("Waypoint{0}", WaypointInd);
            GameObject coordTextObj = GetCoordTextObj(waypointObj);
            Debug.Assert(coordTextObj != null);
            coordTextObj.name = String.Format("WaypointCoord{0}", WaypointInd);
            //Debug.Log(WaypointManager.Instance.Waypoints.Count + " Waypoints exist.");
        }

        public Waypoint GetLastWaypoint() {
            if (Waypoints.Count == 0) {
                return null;
            }
            return Waypoints[Waypoints.Count - 1];
        }

        private void Update() {
            if (StateManager.Instance.CurrentState != StateManager.State.WaypointState) {
                //ClearWaypoints(); // TODO: remove in case of bugs
                return;
            }
            if (Waypoints.Count == 0) {
                InitializeWaypoints();
            }
            if (Waypoints.Count == 3) {
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