using System;
using UnityEngine;

namespace HoloToolkit.Unity {
//namespace Academy.HoloToolkit.Unity {
    public class Utils {
        public static void InitWaypointPos(Camera cam, GameObject obj) {
            var headPosition = cam.transform.position;
            var gazeDirection = cam.transform.forward;
            obj.transform.position = headPosition + gazeDirection * 2.0f;
            var pos = obj.transform.position;
            Debug.Assert(StateManager.Instance.FloorY != -99);
            obj.transform.position = new Vector3(pos.x, StateManager.Instance.FloorY, pos.z);
        }

        //public static void RaycastPlace(Camera cam, GameObject obj, bool isMovo = false) {
        //    Debug.Assert(SpatialMapping.Instance.DrawVisualMeshes == true);
        //    // Do a raycast into the world that will only hit the Spatial Mapping mesh.
        //    var headPosition = cam.transform.position;
        //    var gazeDirection = cam.transform.forward;

        //    RaycastHit hitInfo;
        //    if (Physics.Raycast(headPosition, gazeDirection, out hitInfo,
        //        30.0f, SpatialMapping.PhysicsRaycastMask)) {
        //        // Move this object's parent object to
        //        // where the raycast hit the Spatial Mapping mesh.
        //        //this.transform.position = hitInfo.point;
        //        obj.transform.position = hitInfo.point;

        //        // Rotate this object's parent object to face the user.
        //        Quaternion toQuat = cam.transform.localRotation;
        //        toQuat.x = 0;
        //        toQuat.z = 0;
        //        //this.transform.rotation = toQuat;
        //        obj.transform.rotation = toQuat;
        //        if (isMovo) {
        //            Debug.Assert(obj.name == "Movo");
        //            obj.transform.rotation *= Quaternion.Euler(0, 180, 0);
        //            if (MovoPlace.MovoYSet) {
        //                obj.transform.position = new Vector3(hitInfo.point.x, MovoPlace.MovoY, hitInfo.point.z);
        //            }
        //        }
        //    }
        //}

        //public static void SetSpatialMapping(bool state) {
        //    //Debug.Log("Set spatial mapping: " + state);
        //    SpatialMapping.Instance.DrawVisualMeshes = state;
        //}

    }

    public class Waypoint {
        public GameObject WaypointObj { get; private set; }
        public GameObject CoordTextObj { get; private set; }
        public String Name { get; private set; }
        private double Deg2rad(float angle) {
            return (Math.PI / 180) * angle;
        }
        public Vector2 GetCoords() {
            Transform robotObjTransform = GameObject.Find("Movo").transform;
            Vector3 relativePos = robotObjTransform.InverseTransformPoint(WaypointObj.transform.position);
            //float delTheta = StateManager.Instance.MovoUnityToROSOffset.Theta;
            //Debug.Log(StateManager.Instance.MovoUnityStartPose.Theta);
            //double delTheta = StateManager.Instance.MovoROSStartPose.Theta - StateManager.Instance.MovoUnityStartPose.Theta;
            double delTheta = -StateManager.Instance.MovoROSStartPose.Theta;
            //Debug.Log(delTheta);
            delTheta = (Math.PI / 180) * delTheta; // deg to rad
            var x_coord = relativePos.z; // + StateManager.Instance.MovoROSStartPose.X;
            var y_coord = -relativePos.x; // + StateManager.Instance.MovoROSStartPose.Y;
            // Rotating the coordinates to match ROS coordinate system
            // ----------------------------------------
            x_coord = (float)(x_coord * Math.Cos(delTheta) - y_coord * Math.Sin(delTheta));
            y_coord = (float)(x_coord * Math.Sin(delTheta) + y_coord * Math.Cos(delTheta));
            // ----------------------------------------
            x_coord += StateManager.Instance.MovoROSStartPose.X;
            y_coord += StateManager.Instance.MovoROSStartPose.Y;
            return new Vector2(x_coord, y_coord);
        }
        public Pose GetPose() {
            Vector2 coords = GetCoords();
            Debug.Assert(StateManager.Instance.MovoUnityToROSOffset != null);
            float theta = WaypointObj.transform.eulerAngles.y + StateManager.Instance.MovoUnityToROSOffset.Theta;
            return new Pose(coords.x, coords.y, -theta); // ROS theta goes counterclockwise
        }
        public Waypoint(GameObject waypointObj, int waypointInd) {
            Name = String.Format("Waypoint{0}", waypointInd);
            waypointObj.name = Name;
            WaypointObj = waypointObj;
            CoordTextObj = WaypointManager.Instance.GetCoordTextObj(waypointObj);
        }
    }

    public class Pose {
        public float X { get; private set; }
        public float Y { get; private set; }
        public float Theta { get; private set; }
        public Pose(float x, float y, float theta) {
            X = x;
            Y = y;
            Theta = theta;
        }
        public Vector3 ToUnityCoordsMovo(float unityY) {
            return new Vector3(-Y, unityY, X);
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