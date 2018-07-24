﻿using UnityEngine;

namespace HoloToolkit.Unity {
    public class MovoPlace : MonoBehaviour {
        public static void CalibrateMovo() {
            if (StateManager.Instance.CurrentState == StateManager.State.CalibratingState) {
                Debug.Log("Calibrating!");
                GameObject movoObj = GameObject.Find("Movo");
                StateManager.Instance.FloorY = movoObj.transform.position.y;
                Vector3 movoUnityPos = movoObj.transform.position;
                StateManager.Instance.MovoUnityStartPose = new HoloPose(-movoUnityPos.z, movoUnityPos.x, movoObj.transform.eulerAngles.y);
                Debug.Assert(StateManager.Instance.MovoROSPose != null);
                StateManager.Instance.MovoROSStartPose = StateManager.Instance.MovoROSPose;
                Debug.Assert(StateManager.Instance.MovoROSStartPose != null);
                StateManager.Instance.MovoUnityToROSOffset = StateManager.Instance.MovoROSStartPose - StateManager.Instance.MovoUnityStartPose;
                Debug.Assert(StateManager.Instance.MovoUnityToROSOffset != null);
                StateManager.Instance.RobotCalibrated = true;
                StateManager.Instance.ROSDelTheta = 0;
                StateManager.Instance.TransitionToStandbyState();
            }
        }

        public static void PlaceAtROSPose(GameObject gameObject, HoloPose desiredROSPose, bool keepMovoAsParent=true) {
            gameObject.transform.SetParent(GameObject.Find("Movo").transform);
            HoloPose movoROSStartPose = StateManager.Instance.MovoROSStartPose;
            gameObject.transform.localPosition = (desiredROSPose - movoROSStartPose).ToUnityCoordsMovo();
            StateManager.Instance.ROSDelTheta = desiredROSPose.Theta - movoROSStartPose.Theta;
            gameObject.transform.localRotation = Quaternion.Euler(0, StateManager.Instance.ROSDelTheta + StateManager.Instance.CalibrateThetaOffset, 0);
            if (!keepMovoAsParent) {
                gameObject.transform.parent = null;
            }
        }

        void Update() {
            GameObject baseLink = StateManager.Instance.MovoBaseLink;
            Debug.Assert(baseLink != null);
            HoloPose movoROSPose = StateManager.Instance.MovoROSPose;
            if (movoROSPose == null) {
                return;
            }
            if (!StateManager.Instance.RobotCalibrated) {
                StateManager.Instance.CalibrateThetaOffset = movoROSPose.Theta;
                baseLink.transform.localRotation = Quaternion.Euler(0, StateManager.Instance.CalibrateThetaOffset, 0);
            }
            else if (StateManager.Instance.CurrentState != StateManager.State.CalibratingState) {
                PlaceAtROSPose(baseLink, movoROSPose);
                //    Pose movoROSStartPose = StateManager.Instance.MovoROSStartPose;
                //    baseLink.transform.localPosition = (movoROSPose - movoROSStartPose).ToUnityCoordsMovo();
                //    StateManager.Instance.ROSDelTheta = movoROSPose.Theta - movoROSStartPose.Theta;
                //    baseLink.transform.localRotation = Quaternion.Euler(0, StateManager.Instance.ROSDelTheta + StateManager.Instance.CalibrateThetaOffset, 0);
            }
        }
    }
}