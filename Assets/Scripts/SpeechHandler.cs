using UnityEngine;
using HoloToolkit.Unity.InputModule;
using RosSharp.RosBridgeClient;

namespace HoloToolkit.Unity {
    public class SpeechHandler : MonoBehaviour, ISpeechHandler {

        //public MoveItGoalPublisher MoveItGoalPublisher;
        //public DisplayTrajectoryReceiver DisplayTrajectoryReceiver;

        void ISpeechHandler.OnSpeechKeywordRecognized(SpeechEventData eventData) {
            string command = eventData.RecognizedText.ToLower();
            Debug.Log("Command: " + command);
            if (command == "state") {
                Debug.Log(StateManager.Instance.CurrentState);
                return;
            }
            if (command.Contains("transition")) {
                switch (command) {
                    case "transition standby":
                        if (!StateManager.Instance.RobotCalibrated) {
                            return;
                        }
                        StateManager.Instance.TransitionToStandbyState();
                        break;
                    case "transition calibrate":
                        StateManager.Instance.TransitionToCalibrateState();
                        break;
                    case "transition waypoints":
                        if (!StateManager.Instance.RobotCalibrated) {
                            return;
                        }
                        StateManager.Instance.TransitionToWaypointState();
                        break;
                }
                return;
            }
            switch (StateManager.Instance.CurrentState) {
                case StateManager.State.CalibratingState:
                    ParseCalibrateCommands(command);
                    break;
                case StateManager.State.WaypointState:
                    ParseWaypointCommands(command);
                    break;
            }
        }

        private void ParseCalibrateCommands(string command) {
            Debug.Log("ParseCalibrateCommands()");
            if (StateManager.Instance.CurrentState != StateManager.State.CalibratingState) {
                return;
            }
            switch (command) {
                case "calibrate":
                    MovoPlace.CalibrateMovo();
                    break;
            }
            Debug.Log("done!");
        }

        private void ParseWaypointCommands(string command) {
            if (StateManager.Instance.CurrentState != StateManager.State.WaypointState) {
                return;
            }
            switch (command) {
                case "another":
                    WaypointManager.Instance.AddWaypoint();
                    break;
                case "move":
                    WaypointManager.Instance.TransitionToNavigatingState();
                    break;
                case "restart":
                    WaypointManager.Instance.InitializeWaypoints();
                    break;
            }
        }



        // Sends the goal position to MoveIt
        //public void Plan() {
        //    Debug.Log("Plan");
        //    MoveItGoalPublisher.PublishPlan();
        //}

        //// Tells MoveIt to execute the plan
        //public void Move() {
        //    Debug.Log("Execute");
        //    MoveItGoalPublisher.PublishMove(); // move the arm
        //    DisplayTrajectoryReceiver.loop = false; // stop the visualization
        //}
    }
}