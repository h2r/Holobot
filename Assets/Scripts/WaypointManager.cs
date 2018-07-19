using System;
using System.Collections.Generic;
using UnityEngine;
using HoloToolkit.Unity;


public class WaypointManager : Singleton<WaypointManager> {
    public int WaypointInd { get; set; }
    public List<Waypoint> Waypoints { get; private set; }
    private GameObject WaypointTemplate;
    public List<GeometryPoseStamped> PathPoses; // used to project the Movo's planned navigation path
    public List<GameObject> MovoGhosts;
    public bool PathPosesRefreshed;

    void Start() {
        Debug.Log("Initialized WaypointManager");
        WaypointTemplate = GameObject.Find("Waypoint0");
        Waypoints = new List<Waypoint>();
        PathPoses = new List<GeometryPoseStamped>();
        MovoGhosts = new List<GameObject>();
        Debug.Log("WaypointManager Awake()");
        PathPosesRefreshed = false;
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

    private void DestroyGhosts() {
        for (int i = 0; i < MovoGhosts.Count; i++) {
            GameObject ghost = MovoGhosts[i];
            MovoGhosts.Remove(ghost);
            Destroy(ghost);
        }
    }

    private void VisualizePlannedPath() {
        if (PathPoses.Count == 0) {
            return;
        }
        DestroyGhosts();
        int numPoses = PathPoses.Count;
        for (int i = 0; i < numPoses; i += 5) {
            if (PathPosesRefreshed) {
                DestroyGhosts();
                PathPosesRefreshed = false;
                return;
            }
            GeometryPoseStamped stampedPose = PathPoses[i];
            GeometryPose pose = stampedPose.pose;
            GameObject movoGhost = Instantiate(GameObject.Find("base_link"));
            Vector3 eulerAngles = new Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w).eulerAngles;
            float theta = -eulerAngles.z; // ROS orientation goes in opposite direction
            HoloToolkit.Unity.Pose ROSPose = new HoloToolkit.Unity.Pose(pose.position.x, pose.position.y, theta);
            MovoPlace.PlaceAtROSPose(movoGhost, ROSPose);
            MovoGhosts.Add(movoGhost);
            Debug.Log("Num ghosts: " + MovoGhosts.Count);
        }
    }

    private void Update() {
        // TODO: first get ROS to Unity conversions working, then visualize path(s) upon placing waypoint.
        if (StateManager.Instance.CurrentState == StateManager.State.NavigatingState) {
            VisualizePlannedPath();
        }
        else {
            DestroyGhosts();
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
    }

}

public class Waypoint {
    public GameObject WaypointObj { get; private set; }
    public GameObject CoordTextObj { get; private set; }
    public String Name { get; private set; }
    public Boolean Placed { get; set; }
    public HoloToolkit.Unity.Pose Pose { get; private set; }
    private double Deg2rad(float angle) {
        return (Math.PI / 180) * angle;
    }
    public Vector2 GetCoords() {
        Transform robotObjTransform = GameObject.Find("Movo").transform;
        Vector3 relativePos = robotObjTransform.InverseTransformPoint(WaypointObj.transform.position);
        var x_coord = relativePos.z; // + StateManager.Instance.MovoROSStartPose.X;
        var y_coord = -relativePos.x; // + StateManager.Instance.MovoROSStartPose.Y;
        x_coord += StateManager.Instance.MovoROSStartPose.X;
        y_coord += StateManager.Instance.MovoROSStartPose.Y;
        return new Vector2(x_coord, y_coord);
    }
    public Vector2 GetUnityCoords() {
        Vector2 ROSCoords = GetCoords();
        Vector2 UnityCoords = ROSCoords;
        UnityCoords.x -= StateManager.Instance.MovoROSStartPose.X;
        UnityCoords.y -= StateManager.Instance.MovoROSStartPose.Y;
        Transform robotObjTransform = GameObject.Find("Movo").transform;
        Vector3 UnityPosition = robotObjTransform.TransformPoint(-UnityCoords.y, StateManager.Instance.FloorY, UnityCoords.x);
        return new Vector2(UnityPosition.x, UnityPosition.z);
    }
    public void UpdatePose(float calibThetaOffset = 0) {
        Vector2 coords = GetCoords();
        Debug.Assert(StateManager.Instance.MovoUnityToROSOffset != null);
        float theta = WaypointObj.transform.eulerAngles.y + StateManager.Instance.MovoUnityToROSOffset.Theta + calibThetaOffset;
        //Debug.Log("theta: " + theta);
        Pose = new HoloToolkit.Unity.Pose(coords.x, coords.y, -theta); // ROS theta goes counterclockwise
    }
    public Waypoint(GameObject waypointObj, int waypointInd) {
        Name = String.Format("Waypoint{0}", waypointInd);
        waypointObj.name = Name;
        WaypointObj = waypointObj;
        CoordTextObj = WaypointManager.Instance.GetCoordTextObj(waypointObj);
        Placed = false;
        UpdatePose();
    }
}