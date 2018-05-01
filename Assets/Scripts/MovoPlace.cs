using UnityEngine;
using UnityEngine.UI;
using System;

namespace Academy.HoloToolkit.Unity {
    public class MovoPlace : MonoBehaviour {
        bool robotPlaced;
        private float movoY;
        GameObject movoObj;
        private Pose movoUnityStartPose;
        private Pose movoROSToUnityOffset;
        private Pose movoUnityToROSOffset;

        private void Start() {
            Reset();
        }

        private void Reset() {
            robotPlaced = false;
            movoObj = GameObject.Find("Movo");
        }

        // GameObject pose + movoUnityToROSOffset = ROS pose
        // ROS pose + movoROSToUnityOffset = Unity pose
        private void CalibrateMovo() {
            Debug.Assert(movoObj != null);
            Vector3 movoPos = movoObj.transform.position;
            movoUnityStartPose = new Pose(-movoPos.z, movoPos.x, movoObj.transform.eulerAngles.y);
            StateManager.Instance.MovoStartPose = movoUnityStartPose;
            Pose movoROSStartPose = StateManager.Instance.MovoROSPose;
            Debug.Assert(movoROSStartPose != null);
            movoROSToUnityOffset = movoUnityStartPose - movoROSStartPose; // rosPose + movoOffset = unityPose
            movoUnityToROSOffset = movoROSStartPose - movoUnityStartPose;
            StateManager.Instance.MovoROSToUnityOffset = movoROSToUnityOffset;
            StateManager.Instance.MovoUnityToROSOffset = movoUnityToROSOffset;
            Debug.Log(String.Format("ROS to Unity offset: ({0}, {1}, {2})", movoROSToUnityOffset.X, movoROSToUnityOffset.Y, movoROSToUnityOffset.Theta));
        }

        // Called by GazeGestureManager when the user performs a Select gesture
        void OnSelect() {
            if (!robotPlaced) {
                robotPlaced = true;
                movoY = movoObj.transform.position.y;
                CalibrateMovo();
                StateManager.Instance.RobotCalibrated = true;
                StateManager.Instance.CurrentState = StateManager.State.WaypointState;
            }
        }

        // Update is called once per frame
        void Update() {
            if (StateManager.Instance.CurrentState != StateManager.State.CalibratingState) {
                Debug.Assert(StateManager.Instance.RobotCalibrated == true);
                movoObj.transform.position = (StateManager.Instance.MovoROSPose + movoROSToUnityOffset).ToUnityCoords(movoY);
                movoObj.transform.rotation = Quaternion.Euler(0, StateManager.Instance.MovoROSPose.Theta, 0);
                return;
            }
            if (StateManager.Instance.RobotCalibrated != true) {
                Debug.Assert(movoObj != null);
                if (!robotPlaced) {
                    Utils.RaycastPlace(Camera.main, movoObj);
                }
            }
        }
    }
}