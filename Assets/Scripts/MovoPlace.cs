using UnityEngine;

namespace Academy.HoloToolkit.Unity {
    public class MovoPlace : MonoBehaviour {
        public static float MovoY;
        public static bool MovoYSet;
        private GameObject movoObj;
        private bool movoPlaced;
        private Pose movoLocalPose;

        private void Start() {
            movoPlaced = false;
            MovoYSet = false;
            movoObj = GameObject.Find("Movo");
            movoLocalPose = new Pose(0, 0, 0);
        }

        // GameObject pose + movoUnityToROSOffset = ROS pose
        // ROS pose + movoROSToUnityOffset = Unity pose
        private void CalibrateMovo() {
            Debug.Assert(movoObj != null);
            Vector3 movoUnityPos = movoObj.transform.position;
            Pose movoUnityStartPose = new Pose(-movoUnityPos.z, movoUnityPos.x, movoObj.transform.eulerAngles.y);
            StateManager.Instance.MovoUnityStartPose = movoUnityStartPose;
            StateManager.Instance.MovoROSStartPose = StateManager.Instance.MovoROSPose;
            Debug.Assert(StateManager.Instance.MovoROSStartPose != null);
            //StateManager.Instance.MovoROSToUnityOffset = movoUnityStartPose - StateManager.Instance.MovoROSStartPose;
            //StateManager.Instance.MovoUnityToROSOffset = -StateManager.Instance.MovoROSToUnityOffset;
        }

        // Called by GazeGestureManager when the user performs a Select gesture
        void OnSelect() {
            if (StateManager.Instance.CurrentState != StateManager.State.CalibratingState) {
                return;
            }
            if (!MovoYSet) {
                MovoY = movoObj.transform.position.y;
                MovoYSet = true;
            }
            else if (!movoPlaced) {
                movoPlaced = true;
                CalibrateMovo();
                StateManager.Instance.RobotCalibrated = true;
                StateManager.Instance.CurrentState = StateManager.State.WaypointState;
            }
        }

        // Update is called once per frame
        void Update() {
            if (StateManager.Instance.CurrentState != StateManager.State.CalibratingState) {
                if (StateManager.Instance.UnityDebugMode) {
                    return;
                }
                //Debug.Log("Updating base_link coords");
                Debug.Assert(StateManager.Instance.RobotCalibrated == true);
                //Pose movoROSToUnityOffset = StateManager.Instance.MovoROSToUnityOffset;
                //movoObj.transform.position = (StateManager.Instance.MovoROSPose + movoROSToUnityOffset).ToUnityCoords(MovoY);
                //float theta = StateManager.Instance.MovoROSPose.Theta + movoROSToUnityOffset.Theta;
                //movoObj.transform.rotation = Quaternion.Euler(0, theta, 0);
                Pose movoROSStartPose = StateManager.Instance.MovoROSStartPose;
                Pose movoROSPose = StateManager.Instance.MovoROSPose;
                GameObject baseLink = GameObject.Find("base_link");
                baseLink.transform.localPosition = (movoROSPose - movoROSStartPose).ToUnityCoordsMovo(0);
                float ROSDelTheta = movoROSPose.Theta - movoROSStartPose.Theta;
                baseLink.transform.localRotation = Quaternion.Euler(0, ROSDelTheta, 0);
            }
            else if (StateManager.Instance.RobotCalibrated != true) {
                Debug.Assert(movoObj != null);
                if (!movoPlaced) {
                    //Utils.RaycastPlace(Camera.main, movoObj, true);
                }
            }
        }
    }
}