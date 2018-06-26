using UnityEngine;

//namespace Academy.HoloToolkit.Unity {
namespace HoloToolkit.Unity {
    public class MovoPlace : MonoBehaviour {

        public static void CalibrateMovo() {
            if (StateManager.Instance.CurrentState == StateManager.State.CalibratingState) {
                Debug.Log("Calibrating!");
                GameObject movoObj = GameObject.Find("Movo");
                StateManager.Instance.FloorY = movoObj.transform.position.y;
                Vector3 movoUnityPos = movoObj.transform.position;
                StateManager.Instance.MovoUnityStartPose = new Pose(-movoUnityPos.z, movoUnityPos.x, movoObj.transform.eulerAngles.y);
                Debug.Assert(StateManager.Instance.MovoROSPose != null);
                StateManager.Instance.MovoROSStartPose = StateManager.Instance.MovoROSPose;
                Debug.Assert(StateManager.Instance.MovoROSStartPose != null);
                StateManager.Instance.RobotCalibrated = true;
                //Debug.Assert(WaypointManager.Instance.Waypoints.Count > 0);
                //UtilFunctions.InitWaypointPos(Camera.main, WaypointManager.Instance.Waypoints[0].WaypointObj);
                StateManager.Instance.MovoUnityToROSOffset = StateManager.Instance.MovoROSStartPose - StateManager.Instance.MovoUnityStartPose;
                Debug.Assert(StateManager.Instance.MovoUnityToROSOffset != null);
                //StateManager.Instance.CurrentState = StateManager.State.WaypointState;
                //StateManager.Instance.CurrentState = StateManager.State.StandbyState;
                StateManager.Instance.TransitionToStandbyState();
            }
        }

        void Update() {
            if (StateManager.Instance.CurrentState != StateManager.State.CalibratingState) {
                if (StateManager.Instance.UnityDebugMode) {
                    return;
                }
                Debug.Assert(StateManager.Instance.RobotCalibrated == true);
                Pose movoROSStartPose = StateManager.Instance.MovoROSStartPose;
                Pose movoROSPose = StateManager.Instance.MovoROSPose;
                //GameObject baseLink = GameObject.Find("base_link");
                GameObject baseLink = StateManager.Instance.MovoBaseLink;
                baseLink.transform.localPosition = (movoROSPose - movoROSStartPose).ToUnityCoordsMovo(0);
                float ROSDelTheta = movoROSPose.Theta - movoROSStartPose.Theta;
                baseLink.transform.localRotation = Quaternion.Euler(0, ROSDelTheta, 0);
            }
        }
    }
}