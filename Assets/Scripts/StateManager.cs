using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

namespace HoloToolkit.Unity {
    //namespace Academy.HoloToolkit.Unity {
    public class StateManager : Singleton<StateManager> {
        //public class StateManager : MonoBehaviour {
        public bool RobotCalibrated { get; set; }
        //public bool TransitionedToWaypointState { get; set; }
        public enum State { CalibratingState, StandbyState, WaypointState, NavigatingState };
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

        private void Start() {
            Debug.Log("Initialized StateManager");
            //MovoROSPose = null;
            RightGripper = GameObject.Find("RightGripper");
            LeftGripper = GameObject.Find("LeftGripper");
            TransitionToCalibrateState();
            MovoState = "standby";
        }

        public void TransitionToWaypointState() {
            if (CurrentState == State.WaypointState) {
                return;
            }
            WaypointManager.Instance.InitializeWaypoints();
            UtilFunctions.SetGrippersActive(true);
            Instance.CurrentState = State.WaypointState;
            Debug.Log("Transitioned to waypoint state");
        }

        public void TransitionToCalibrateState() {
            RobotCalibrated = false;
            WaypointManager.Instance.ClearWaypoints();
            UtilFunctions.SetGrippersActive(false);
            CurrentState = State.CalibratingState;
            Debug.Log("Transitioned to calibrate state");
        }

        public void TransitionToStandbyState() {
            WaypointManager.Instance.ClearWaypoints();
            UtilFunctions.SetGrippersActive(true);
            CurrentState = State.StandbyState;
            Debug.Log("Transitioned to standby state");
        }

        public void DisplayState() {
            Text stateMsg = GameObject.Find("StateReporter").GetComponent<Text>();
            stateMsg.text = CurrentState.ToString() + "\n" + SpeechHandler.CurrentCommand;
        }

        private void Update() {
            //Debug.Log("Current state: " + CurrentState);
            Debug.Assert(MovoState == "standby" || MovoState == "navigating");
            DisplayState();
        }
    }
}