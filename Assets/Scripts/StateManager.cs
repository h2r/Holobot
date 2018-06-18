using System.Collections.Generic;
using UnityEngine;

namespace HoloToolkit.Unity {
//namespace Academy.HoloToolkit.Unity {
    public class StateManager : Singleton<StateManager> {
    //public class StateManager : MonoBehaviour {
        public bool RobotCalibrated { get; set; }
        //public bool TransitionedToWaypointState { get; set; }
        public enum State { CalibratingState, StandbyState, WaypointState, NavigatingState, ArmManipulationState };
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

        //void Awake() {
        private void Start() {
            Debug.Log("Initialized StateManager");
            //MovoROSToUnityOffset = null;
            MovoROSPose = null;
            RobotCalibrated = false;
            CurrentState = State.CalibratingState;
            //TransitionedToWaypointState = false;
            MovoState = "standby";
        }

        private void ParseStates() {
            //if (CurrentState == State.CalibratingState || CurrentState == State.WaypointState) {
            //    Utils.SetSpatialMapping(true);
            //}
            //else {
            //    Utils.SetSpatialMapping(false);
            //}
            if (CurrentState == State.StandbyState) {
                Debug.Log("Standing by...");
                return;
            }
            if (!RobotCalibrated && CurrentState != State.NavigatingState) {
                CurrentState = State.CalibratingState;
            }
            if (RobotCalibrated && CurrentState == State.CalibratingState) {
                //Debug.Assert(MovoROSToUnityOffset != null);
                CurrentState = State.WaypointState;
            }
        }

        private void Update() {
            ParseStates();
            Debug.Assert(MovoState == "standby" || MovoState == "navigating");
        }
    }
}