using System;
using UnityEngine;

namespace HoloToolkit.Unity {
    //namespace Academy.HoloToolkit.Unity {
    public class UtilFunctions {
        public static void InitWaypointPos(Camera cam, GameObject obj) {
            var headPosition = cam.transform.position;
            var gazeDirection = cam.transform.forward;
            obj.transform.position = headPosition + gazeDirection * 2.0f;
            var pos = obj.transform.position;
            Debug.Assert(StateManager.Instance.FloorY != -99);
            obj.transform.position = new Vector3(pos.x, StateManager.Instance.FloorY, pos.z);
        }

        //public static void ReportState() {
        //    Debug.Log("Current state: " + StateManager.Instance.CurrentState);
        //}

        public static void FollowGaze(Camera cam, GameObject obj, float dist = 2.0f) {
            var headPosition = cam.transform.position;
            var gazeDirection = cam.transform.forward;
            obj.transform.position = headPosition + gazeDirection * dist;
        }

        public static void SetGrippersActive(bool state) {
            StateManager.Instance.RightGripper.SetActive(state);
            StateManager.Instance.LeftGripper.SetActive(state);
        }

        public static float Deg2Rad(float deg) {
            return (float)(Math.PI / 180) * deg;
        }

        public static Vector3 UnityToRosPositionAxisConversion(Vector3 rosIn) {
            return new Vector3(rosIn.z, -rosIn.x, rosIn.y);
        }

        //Convert 4D Unity quaternion to ROS quaternion
        public static Quaternion UnityToRosRotationAxisConversion(Quaternion qIn) {
            return new Quaternion(qIn.z, -qIn.x, qIn.y, -qIn.w);
        }

        public static GeometryQuaternion QuaternionToGeometryQuaternion(Quaternion q) {
            return new GeometryQuaternion {
                x = q.x,
                y = q.y,
                z = q.z,
                w = q.w
            };
        }

        public static GeometryPoint Vector3ToGeometryPoint(Vector3 v) {
            return new GeometryPoint {
                x = v.x,
                y = v.y,
                z = v.z
            };
        }
    }

    public class Waypoint {
        public GameObject WaypointObj { get; private set; }
        public GameObject CoordTextObj { get; private set; }
        public String Name { get; private set; }
        public Boolean Placed { get; set; }
        public Pose Pose { get; private set; }
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
        public void UpdatePose(float calibThetaOffset = 0) {
            Vector2 coords = GetCoords();
            Debug.Assert(StateManager.Instance.MovoUnityToROSOffset != null);
            float theta = WaypointObj.transform.eulerAngles.y + StateManager.Instance.MovoUnityToROSOffset.Theta + calibThetaOffset;
            Debug.Log("theta: " + theta);
            Pose = new Pose(coords.x, coords.y, -theta); // ROS theta goes counterclockwise
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

    public class Label {
        public GameObject WaypointObj { get; private set; }
        public GameObject CoordTextObj { get; private set; }
        public String Name { get; private set; }
        public Boolean Placed { get; set; }
        public Pose Pose { get; private set; }
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
        public void UpdatePose(float calibThetaOffset = 0) {
            Vector2 coords = GetCoords();
            Debug.Assert(StateManager.Instance.MovoUnityToROSOffset != null);
            float theta = WaypointObj.transform.eulerAngles.y + StateManager.Instance.MovoUnityToROSOffset.Theta + calibThetaOffset;
            Debug.Log("theta: " + theta);
            Pose = new Pose(coords.x, coords.y, -theta); // ROS theta goes counterclockwise
        }
        public Label(GameObject waypointObj, int waypointInd) {
            Name = String.Format("Waypoint{0}", waypointInd);
            waypointObj.name = Name;
            WaypointObj = waypointObj;
            CoordTextObj = LabelManager.Instance.GetCoordTextObj(waypointObj);
            Placed = false;
            UpdatePose();
        }
    }

    public class Pose {
        public float X { get; private set; }
        public float Y { get; private set; }
        public float Theta { get; set; }
        public Pose(float x, float y, float theta) {
            X = x;
            Y = y;
            Theta = theta;
        }
        public Vector3 ToUnityCoordsMovo() {
            return new Vector3(-Y, 0.0f, X);
        }
        //public Vector3 ToUnityCoords(float unityY, bool flipX = true) {
        //    if (flipX) {
        //        return new Vector3(Y, unityY, -X);
        //    }
        //    return new Vector3(Y, unityY, X);
        //}
        public static Pose operator +(Pose pose1, Pose pose2) {
            return new Pose(pose1.X + pose2.X, pose1.Y + pose2.Y, pose1.Theta + pose2.Theta);
        }
        public static Pose operator -(Pose pose1, Pose pose2) {
            return new Pose(pose1.X - pose2.X, pose1.Y - pose2.Y, pose1.Theta - pose2.Theta);
        }
        public static Pose operator -(Pose pose) {
            return new Pose(-pose.X, -pose.Y, -pose.Theta);
        }
    }
}