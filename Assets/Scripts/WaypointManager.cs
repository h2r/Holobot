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
        public List<GeometryPoseStamped> PathPoses; // used to project the Movo's planned navigation path
        public List<GameObject> MovoGhosts;
        // Use this for initialization

        //void Awake() {
        void Start() {
            Debug.Log("Initialized WaypointManager");
            WaypointTemplate = GameObject.Find("Waypoint0");
            Waypoints = new List<Waypoint>();
            PathPoses = new List<GeometryPoseStamped>();
            MovoGhosts = new List<GameObject>();
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

        private void VisualizePlannedPath() {
            if (PathPoses.Count == 0) {
                return;
            }
            foreach (GameObject movoGhost in MovoGhosts) {
                Destroy(movoGhost);
            }
            MovoGhosts.Clear();
            foreach (GeometryPoseStamped stampedPose in PathPoses) {
                GeometryPose pose = stampedPose.pose;
                Vector2 unityCoords = UtilFunctions.RosToUnityCoords(new Vector2(pose.position.x, pose.position.y));
                Vector3 unityPosition = new Vector3(unityCoords.x, StateManager.Instance.FloorY, unityCoords.y);
                Quaternion ROSQuaternion = new Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
                //Quaternion unityQuaternion = UtilFunctions.RosToUnityRotationAxisConversion(ROSquaternion); // TODO: add back in
                GameObject movoGhost = Instantiate(GameObject.Find("base_link"));
                movoGhost.transform.position = unityPosition;
                movoGhost.transform.rotation = ROSQuaternion;
                MovoGhosts.Add(movoGhost);
                Debug.Log("Num ghosts: " + MovoGhosts.Count);
            }
        }

        private void Update() {
            // TODO: first get ROS to Unity conversions working, then visualize path(s) upon placing waypoint.
            if (StateManager.Instance.CurrentState == StateManager.State.NavigatingState) {
                VisualizePlannedPath();
            }
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
            //Debug.Log("AHHHHH");
            //VisualizePlannedPath();
        }

    }
}