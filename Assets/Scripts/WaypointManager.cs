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
        // Use this for initialization

        //void Awake() {
        void Start() {
            Debug.Log("Initialized WaypointManager");
            WaypointTemplate = GameObject.Find("Waypoint0");
            Waypoints = new List<Waypoint>();
            Debug.Log("WaypointManager Awake()");
        }

        public void ClearWaypoints() {
            if (Waypoints == null) { // if there's a bug, change Start() back to Awake().
                return;
            }
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
            if (StateManager.Instance.CurrentState == StateManager.State.WaypointState) { // If in WaypointState, then place waypoint in front of user.
                UtilFunctions.InitWaypointPos(Camera.main, waypointObj);
            }
            WaypointInd++;
            waypointObj.name = String.Format("Waypoint{0}", WaypointInd);
            GameObject coordTextObj = GetCoordTextObj(waypointObj);
            Debug.Assert(coordTextObj != null);
            coordTextObj.name = String.Format("WaypointCoord{0}", WaypointInd);
            Waypoints.Add(new Waypoint(waypointObj, WaypointInd));
            Debug.Log(Waypoints.Count + " waypoints exist.");
            Debug.Assert(GetLastWaypoint().Name == waypointObj.name);
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
            if (Waypoints.Count == 0) {
                InitializeWaypoints();
            }
            Waypoint lastWaypoint = GetLastWaypoint();
            if (!lastWaypoint.Placed) {
                UtilFunctions.FollowGaze(Camera.main, lastWaypoint.WaypointObj);
            }
        }

    }
}