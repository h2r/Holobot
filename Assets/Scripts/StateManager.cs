using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

namespace HoloToolkit.Unity {
    //namespace Academy.HoloToolkit.Unity {
    public class StateManager : Singleton<StateManager> {
        //public class StateManager : MonoBehaviour {
        public bool RobotCalibrated { get; set; }
        public float CalibrateThetaOffset; // offset baseLink by this amount to account for intial ROS theta
        //public bool TransitionedToWaypointState { get; set; }
        public enum State { CalibratingState, StandbyState, WaypointState, NavigatingState, PuppetState, MotionIntentState };
        public State CurrentState { get; set; }
        //public Pose MovoROSToUnityOffset { get; set; }
        //public Pose MovoUnityToROSOffset { get; set; }
        public Pose MovoROSPose { get; set; }
        public Pose MovoROSStartPose { get; set; }
        public Pose MovoUnityStartPose { get; set; }
        public Pose MovoUnityToROSOffset { get; set; }
        public string MovoState { get; set; }
        public bool UnityDebugMode = false;
        [HideInInspector]
        public float FloorY = -99; // The y-coordinate of the floor (initialized to impossible value)
        [HideInInspector]
        public GameObject RightGripper, LeftGripper, MovoBaseLink, MovoShadow;
        [HideInInspector]
        public List<string> EinCommandsToExecute;
        [HideInInspector]
        public bool LookAtUser;
        [HideInInspector]
        public float ROSDelTheta;
        [HideInInspector]
        public bool MoveitPlanIdentityPose = false; // if true, make moveit plan movement to current arm poses (e.g. to initialize model joint states)

        private void Start() {
            Debug.Log("Initialized StateManager");
            RightGripper = GameObject.Find("RightGripper");
            LeftGripper = GameObject.Find("LeftGripper");
            MovoShadow = GameObject.Find("movo_moveit_shadow");
            MovoState = "standby";
            EinCommandsToExecute = new List<string>();
            LookAtUser = false;
            MovoBaseLink = GameObject.Find("base_link");
            TransitionToCalibrateState();
            ROSDelTheta = 0;
        }

        public void TransitionToWaypointState() {
            WaypointManager.Instance.InitializeWaypoints();
            UtilFunctions.SetGrippersActive(false);
            CurrentState = State.WaypointState;
            MovoShadow.SetActive(false);
            Debug.Log("Transitioned to waypoint state");
        }

        public void TransitionToNavigatingState() {
            if (CurrentState != State.WaypointState) {
                return;
            }
            if (UnityDebugMode) {
                WaypointManager.Instance.InitializeWaypoints();
            }
            else {
                CurrentState = State.NavigatingState;
            }
            MovoShadow.SetActive(false);
            Debug.Log("Transitioned to navigation state");
        }

        public void TransitionToCalibrateState() {
            RobotCalibrated = false;
            WaypointManager.Instance.ClearWaypoints();
            UtilFunctions.SetGrippersActive(false);
            CurrentState = State.CalibratingState;
            GameObject movoObj = GameObject.Find("Movo");
            movoObj.transform.position = MovoBaseLink.transform.position;
            MovoBaseLink.transform.localPosition = new Vector3(0, 0, 0);
            Debug.Log("Transitioned to calibrate state");
        }

        public void TransitionToStandbyState() {
            WaypointManager.Instance.ClearWaypoints();
            UtilFunctions.SetGrippersActive(false);
            MovoShadow.SetActive(false);
            CurrentState = State.StandbyState;
            Debug.Log("Transitioned to standby state");
        }

        public void TransitionToPuppetState() {
            WaypointManager.Instance.ClearWaypoints();
            UtilFunctions.SetGrippersActive(true);
            MovoShadow.SetActive(false);
            CurrentState = State.PuppetState;
            Debug.Log("Transitioned to puppet state");
        }

        public void TransitionToMotionIntentState() {
            WaypointManager.Instance.ClearWaypoints();
            UtilFunctions.SetGrippersActive(true);
            MovoShadow.SetActive(true);
            CurrentState = State.MotionIntentState;
            //UpdateRightArm = false;
            //UpdateLeftArm = false;
            Debug.Log("Transitioned to motion intent state");
        }

        public void DisplayState() {
            Text stateMsg = GameObject.Find("StateReporter").GetComponent<Text>();
            string msg = CurrentState.ToString() + "\n" + SpeechHandler.CurrentCommand + "\n";
            stateMsg.text = msg;
        }

        private void Update() {
            Debug.Assert(MovoState == "standby" || MovoState == "navigating");
            DisplayState();
        }
    }
}