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

        public static Quaternion RosToUnityRotationAxisConversion(Quaternion qIn) {
            //return new Quaternion(qIn.y, -qIn.z, -qIn.x, qIn.w);
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

        public static Vector2 RosToUnityCoords(Vector2 ROSCoords) {
            Vector2 UnityCoords = ROSCoords;
            UnityCoords.x -= StateManager.Instance.MovoROSStartPose.X;
            UnityCoords.y -= StateManager.Instance.MovoROSStartPose.Y;
            Transform robotObjTransform = GameObject.Find("Movo").transform;
            Vector3 UnityPosition = robotObjTransform.TransformPoint(-UnityCoords.y, StateManager.Instance.FloorY, UnityCoords.x);
            return new Vector2(UnityPosition.x, UnityPosition.z);
        }

        public static GeometryPoseStamped PoseToPoseStamped(GeometryPose pose, string frame_id) {
            return new GeometryPoseStamped {
                pose = pose,
                header = new StandardHeader {
                    frame_id = frame_id
                }
            };
        }

        public static GeometryPoseStamped PoseToPoseStamped(HoloPose pose, string frame_id) {
            Quaternion quat = Quaternion.Euler(0, 0, pose.Theta);
            return new GeometryPoseStamped {
                header = new StandardHeader {
                    frame_id = frame_id
                },
                pose = new GeometryPose {
                    position = new GeometryPoint {
                        x = pose.X,
                        y = pose.Y,
                        z = 0
                    },
                    orientation = new GeometryQuaternion {
                        x = quat.x,
                        y = quat.y,
                        z = quat.z,
                        w = quat.w
                    }
                }
            };
        }
    }

    public class HoloPose {
        public float X { get; private set; }
        public float Y { get; private set; }
        public float Theta { get; set; }
        public HoloPose(float x, float y, float theta) {
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
        public static HoloPose operator +(HoloPose pose1, HoloPose pose2) {
            return new HoloPose(pose1.X + pose2.X, pose1.Y + pose2.Y, pose1.Theta + pose2.Theta);
        }
        public static HoloPose operator -(HoloPose pose1, HoloPose pose2) {
            return new HoloPose(pose1.X - pose2.X, pose1.Y - pose2.Y, pose1.Theta - pose2.Theta);
        }
        public static HoloPose operator -(HoloPose pose) {
            return new HoloPose(-pose.X, -pose.Y, -pose.Theta);
        }
    }
}