using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace HoloToolkit.Unity {
//namespace Academy.HoloToolkit.Unity {
    public class LabelManager : Singleton<LabelManager> {
        public int WaypointInd { get; set; }
        //public List<Label> Labelpoints { get; private set; }
        public Dictionary<string, GameObject[]> LabelDict = new Dictionary<string, GameObject[]>();
        public GameObject WaypointTemplate;
        // Use this for initialization

        //void Awake() {
        void Start() {
            Debug.Log("Initialized Label Manager");
            //WTemplate = GameObject.Find("Waypoint0");
            //Labelpoints = new List<Label>();
            Debug.Log("LabelManager Awake()");
        }

        //public void ClearWaypoints() {
        //    if (Waypoints == null) { // if there's a bug, change Start() back to Awake().
        //        return;
        //    }
        //    foreach (Waypoint wp in Waypoints) {
        //        Destroy(wp.WaypointObj);
        //    }
        //    Waypoints = new List<Waypoint>();
        //    WaypointInd = 0;
        //}

        public void InitializeWaypoints() {
            Debug.Log("InitializeWaypoints()");
            //ClearWaypoints();
            //AddWaypoint();
        }

       

        public GameObject GetCoordTextObj(GameObject waypointObj) {
            return waypointObj.transform.GetChild(0).gameObject.transform.GetChild(0).gameObject;
        }

        public void AddLabel() {
            Debug.Log("AddLabel()");
            GameObject label1 = Instantiate(WaypointTemplate); // This waypoint will eventually be destroyed, so Instantiate ensures that WaypointTemplate is always there.
            GameObject label2 = Instantiate(WaypointTemplate); // This waypoint will eventually be destroyed, so Instantiate ensures that WaypointTemplate is always there.


            if (StateManager.Instance.CurrentState == StateManager.State.LabelState) { // If in WaypointState, then place waypoint in front of user.
                UtilFunctions.InitLabelPos(Camera.main, label1, label2);
                LabelDict.Add(LabelDict.Count.ToString(), new[] { label1, label2 });
            }
            Debug.Log("Finished adding");
            Debug.Log(LabelDict.Count.ToString());
            //WaypointInd++;
            //waypointObj.name = String.Format("Waypoint{0}", WaypointInd);
            //GameObject coordTextObj = GetCoordTextObj(waypointObj);
            //Debug.Assert(coordTextObj != null);
            //coordTextObj.name = String.Format("WaypointCoord{0}", WaypointInd);
            //Waypoints.Add(new Waypoint(waypointObj, WaypointInd));
            //Debug.Log(Waypoints.Count + " waypoints exist.");
            //Debug.Assert(GetLastWaypoint().Name == waypointObj.name);
        }

        //public Waypoint GetLastWaypoint() {
        //    if (Waypoints.Count == 0) {
        //        return null;
        //    }
        //    return Waypoints[Waypoints.Count - 1];
        //}

        private void Update() {
            if (StateManager.Instance.CurrentState != StateManager.State.WaypointState) {
                return;
            }
            //if (Waypoints.Count == 0) {
            //    InitializeWaypoints();
            //}
            //Waypoint lastWaypoint = GetLastWaypoint();
            //if (!lastWaypoint.Placed) {
                //UtilFunctions.FollowGaze(Camera.main, lastWaypoint.WaypointObj);
            //}
        }

    }
}