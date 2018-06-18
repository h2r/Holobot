using UnityEngine;

//namespace Academy.HoloToolkit.Unity {
namespace HoloToolkit.Unity {
    public class MovoPlace : MonoBehaviour {
        void Update() {
            if (StateManager.Instance.CurrentState != StateManager.State.CalibratingState) {
                if (StateManager.Instance.UnityDebugMode) {
                    return;
                }
                Debug.Assert(StateManager.Instance.RobotCalibrated == true);
                Pose movoROSStartPose = StateManager.Instance.MovoROSStartPose;
                Pose movoROSPose = StateManager.Instance.MovoROSPose;
                GameObject baseLink = GameObject.Find("base_link");
                baseLink.transform.localPosition = (movoROSPose - movoROSStartPose).ToUnityCoordsMovo(0);
                float ROSDelTheta = movoROSPose.Theta - movoROSStartPose.Theta;
                baseLink.transform.localRotation = Quaternion.Euler(0, ROSDelTheta, 0);
            }
        }
    }
}