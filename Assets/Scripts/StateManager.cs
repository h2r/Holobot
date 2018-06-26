using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

namespace HoloToolkit.Unity {
    //namespace Academy.HoloToolkit.Unity {
    public class StateManager : Singleton<StateManager> {
        //public class StateManager : MonoBehaviour {
        public bool RobotCalibrated { get; set; }
        //public bool TransitionedToWaypointState { get; set; }
        public enum State { CalibratingState, StandbyState, WaypointState, NavigatingState, PuppetState };
        public State CurrentState { get; set; }
        //public Pose MovoROSToUnityOffset { get; set; }
        //public Pose MovoUnityToROSOffset { get; set; }
        public Pose MovoROSPose { get; set; }
        public Pose MovoROSStartPose { get; set; }
        public Pose MovoUnityStartPose { get; set; }
        public Pose MovoUnityToROSOffset { get; set; }
        public string MovoState { get; set; }
        public bool UnityDebugMode = false;
        public float FloorY = -99; // The y-coordinate of the floor (initialized to impossible value)
        public GameObject RightGripper;
        public GameObject LeftGripper;
        public GameObject MovoBaseLink;
        public List<string> EinCommandsToExecute;
        public bool LookAtUser;
        [HideInInspector]
        public bool UpdateRightArm = false;
        [HideInInspector]
        public bool UpdateLeftArm = false;

        private void Start() {
            Debug.Log("Initialized StateManager");
            //MovoROSPose = null;
            RightGripper = GameObject.Find("RightGripper");
            LeftGripper = GameObject.Find("LeftGripper");
            TransitionToCalibrateState();
            MovoState = "standby";
            EinCommandsToExecute = new List<string>();
            LookAtUser = false;
            MovoBaseLink = GameObject.Find("base_link");
        }

        public void TransitionToWaypointState() {
            WaypointManager.Instance.InitializeWaypoints();
            UtilFunctions.SetGrippersActive(false);
            Instance.CurrentState = State.WaypointState;
            UpdateRightArm = false;
            UpdateLeftArm = false;
            Debug.Log("Transitioned to waypoint state");
        }

        public void TransitionToCalibrateState() {
            RobotCalibrated = false;
            WaypointManager.Instance.ClearWaypoints();
            UtilFunctions.SetGrippersActive(false);
            CurrentState = State.CalibratingState;
            UpdateRightArm = false;
            UpdateLeftArm = false;
            Debug.Log("Transitioned to calibrate state");
        }

        public void TransitionToStandbyState() {
            WaypointManager.Instance.ClearWaypoints();
            UtilFunctions.SetGrippersActive(false);
            CurrentState = State.StandbyState;
            UpdateRightArm = false;
            UpdateLeftArm = false;
            Debug.Log("Transitioned to standby state");
        }

        public void TransitionToPuppetState() {
            WaypointManager.Instance.ClearWaypoints();
            UtilFunctions.SetGrippersActive(true);
            CurrentState = State.PuppetState;
            UpdateRightArm = false;
            UpdateLeftArm = false;
            Debug.Log("Transitioned to puppet state");
        }

        public void DisplayState() {
            Text stateMsg = GameObject.Find("StateReporter").GetComponent<Text>();
            string msg = CurrentState.ToString() + "\n" + SpeechHandler.CurrentCommand + "\n";
            if (UpdateRightArm) {
                msg += "updating right arm...";
            }
            else if (UpdateLeftArm) {
                msg += "updating left arm...";
            }
            stateMsg.text = msg;
        }

        private void Update() {
            //Debug.Log("Current state: " + CurrentState);
            Debug.Assert(MovoState == "standby" || MovoState == "navigating");
            DisplayState();
        }
    }
}