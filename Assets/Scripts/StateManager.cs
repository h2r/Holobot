using UnityEngine;
using UnityEngine.XR.WSA.Input;
using System.Collections.Generic;
using System.IO;

namespace Academy.HoloToolkit.Unity {
    public class StateManager : Singleton<StateManager> {
        public Vector3 RobotOffset { get; set; }
        public bool RobotCalibrated { get; set; }
        public bool PlacingWaypoints { get; set; }
        public Vector3 RobotStartPos { get; set; }
        public enum State { CalibratingState, WaypointState };
        public State CurrentState;

        void Awake() {
            Debug.Log("Initialized StateManager");
            RobotOffset = Vector3.zero;
            RobotStartPos = Vector3.zero;
            RobotCalibrated = true;
            PlacingWaypoints = true;
            CurrentState = State.CalibratingState;
        }

        private void Update() {
            if (!RobotCalibrated) {
                CurrentState = State.CalibratingState;
            }
            if (RobotCalibrated) {
                PlacingWaypoints = true;
                CurrentState = State.WaypointState;
                if (!PlacingWaypoints) {
                    return;
                }
            }
        }
    }
}