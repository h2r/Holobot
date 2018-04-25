using UnityEngine;
using UnityEngine.XR.WSA.Input;
using System.Collections.Generic;
using System.IO;

namespace Academy.HoloToolkit.Unity {
    public class StateManager : Singleton<StateManager> {
        public Vector3 RobotOffset { get; set; }
        public bool RobotCalibrated { get; set; }
        public Vector3 RobotStartPos { get; set; }
        public enum State { CalibratingState, WaypointState };
        public State CurrentState;

        void Awake() {
            Debug.Log("Initialized StateManager");
            RobotOffset = Vector3.zero;
            RobotStartPos = Vector3.zero;
            RobotCalibrated = false;
            CurrentState = State.CalibratingState;
        }

        private void ParseStates() {
            if (CurrentState == State.CalibratingState || CurrentState == State.WaypointState) {
                Utils.SetSpatialMapping(true);
            }
            else {
                Utils.SetSpatialMapping(false);
            }
            if (!RobotCalibrated) {
                CurrentState = State.CalibratingState;
            }
            if (RobotCalibrated) {
                CurrentState = State.WaypointState;
            }
        }

        private void Update() {
            ParseStates();
        }
    }
}