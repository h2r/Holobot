using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Academy.HoloToolkit.Unity {
    public class Utils : MonoBehaviour {
        public static void RaycastPlace(Camera cam, GameObject obj) {
            Debug.Assert(SpatialMapping.Instance.DrawVisualMeshes == true);
            // Do a raycast into the world that will only hit the Spatial Mapping mesh.
            var headPosition = cam.transform.position;
            var gazeDirection = cam.transform.forward;

            RaycastHit hitInfo;
            if (Physics.Raycast(headPosition, gazeDirection, out hitInfo,
                30.0f, SpatialMapping.PhysicsRaycastMask)) {
                // Move this object's parent object to
                // where the raycast hit the Spatial Mapping mesh.
                //this.transform.position = hitInfo.point;
                obj.transform.position = hitInfo.point;

                // Rotate this object's parent object to face the user.
                Quaternion toQuat = cam.transform.localRotation;
                toQuat.x = 0;
                toQuat.z = 0;
                //this.transform.rotation = toQuat;
                obj.transform.rotation = toQuat;
            }
        }

        public static void SetSpatialMapping(bool state) {
            SpatialMapping.Instance.DrawVisualMeshes = state;
        }

        public static Pose ROSToUnityPose(Pose rosPose) {
            if (StateManager.Instance.MovoROSToUnityOffset == null) {
                return null;
            }
            return rosPose + StateManager.Instance.MovoROSToUnityOffset;
        }

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
            var x_coord = -relativePos.z + StateManager.Instance.MovoUnityToROSOffset.X;
            var y_coord = relativePos.x + StateManager.Instance.MovoUnityToROSOffset.Y;
            return new Vector2(x_coord, y_coord);
        }
        public Waypoint(GameObject waypointObj, int WaypointInd) {
            waypointObj.transform.parent = GameObject.Find("Movo").transform;
            Name = String.Format("Waypoint{0}", WaypointInd);
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
        public Vector3 ToUnityCoords(float unityY) {
            return new Vector3(Y, unityY, -X);
        }
        public static Pose operator +(Pose pose1, Pose pose2) {
            return new Pose(pose1.X + pose2.X, pose1.Y + pose2.Y, pose1.Theta + pose2.Theta);
        }
        public static Pose operator -(Pose pose1, Pose pose2) {
            return new Pose(pose1.X - pose2.X, pose1.Y - pose2.Y, pose1.Theta - pose2.Theta);
        }
    }
}