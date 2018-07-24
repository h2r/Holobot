using UnityEngine;
using HoloToolkit.Unity.InputModule;
using RosSharp.RosBridgeClient;

namespace HoloToolkit.Unity {
    public class SpeechHandler : MonoBehaviour, ISpeechHandler {

        //public MoveItGoalPublisher MoveItGoalPublisher;
        //public DisplayTrajectoryReceiver DisplayTrajectoryReceiver;

        [HideInInspector]
        public static string CurrentCommand;
        private int frameCounter = 0;
        [HideInInspector]
        private bool commandDetected = false;
        private UnityRosBridge moveitGoalPublisher;

        private void Start() {
            moveitGoalPublisher = GameObject.Find("RosConnector").GetComponent<UnityRosBridge>();
        }

        void ISpeechHandler.OnSpeechKeywordRecognized(SpeechEventData eventData) {
            CurrentCommand = eventData.RecognizedText.ToLower();
            commandDetected = true;
            Debug.Log("Command: " + CurrentCommand);
            if (CurrentCommand == "state") {
                Debug.Log(StateManager.Instance.CurrentState);
                return;
            }
            switch (CurrentCommand) {
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
                case "transition puppet":
                    if (!StateManager.Instance.RobotCalibrated) {
                        return;
                    }
                    StateManager.Instance.TransitionToPuppetState();
                    break;
                case "transition motion intent":
                    if (!StateManager.Instance.RobotCalibrated) {
                        return;
                    }
                    StateManager.Instance.TransitionToMotionIntentState();
                    break;
                case "look straight":
                    StateManager.Instance.EinCommandsToExecute.Add("lookStraight");
                    break;
                case "look smug":
                    StateManager.Instance.EinCommandsToExecute.Add("lookSmug");
                    break;
                case "look at me":
                    StateManager.Instance.LookAtUser = true;
                    break;
                case "stop looking":
                    StateManager.Instance.LookAtUser = false;
                    break;
                case "open left":
                    moveitGoalPublisher.PublishGripperCommand("left", "open");
                    break;
                case "close left":
                    moveitGoalPublisher.PublishGripperCommand("left", "close");
                    break;
                case "open right":
                    moveitGoalPublisher.PublishGripperCommand("right", "open");
                    break;
                case "close right":
                    moveitGoalPublisher.PublishGripperCommand("right", "close");
                    break;
            }
            switch (StateManager.Instance.CurrentState) {
                case StateManager.State.CalibratingState:
                    ParseCalibrateCommands(CurrentCommand);
                    break;
                case StateManager.State.WaypointState:
                    ParseWaypointCommands(CurrentCommand);
                    break;
                case StateManager.State.PuppetState:
                    ParsePuppetCommands(CurrentCommand);
                    break;
                case StateManager.State.MotionIntentState:
                    ParseMotionIntentCommands(CurrentCommand);
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
                    StateManager.Instance.TransitionToNavigatingState();
                    break;
                case "restart":
                    WaypointManager.Instance.InitializeWaypoints();
                    break;
            }
        }

        private void ParsePuppetCommands(string command) {
            if (StateManager.Instance.CurrentState != StateManager.State.PuppetState) {
                return;
            }
            switch (command) {
                //case "open right":
                //    StateManager.Instance.EinCommandsToExecute.Add("switchToRightArm openGripper");
                //    break;
                //case "open left":
                //    StateManager.Instance.EinCommandsToExecute.Add("switchToLeftArm openGripper");
                //    break;
                case "close right":
                    StateManager.Instance.EinCommandsToExecute.Add("switchToRightArm closeGripper");
                    break;
                case "close left":
                    StateManager.Instance.EinCommandsToExecute.Add("switchToLeftArm closeGripper");
                    break;
            }
        }

        private void ParseMotionIntentCommands(string command) {
            if (StateManager.Instance.CurrentState != StateManager.State.MotionIntentState) {
                return;
            }
            UnityRosBridge moveitGoalPublisher = GameObject.Find("RosConnector").GetComponent<UnityRosBridge>();
            switch (command) {
                case "move":
                    Debug.Log("Sending move command...");
                    moveitGoalPublisher.PublishMove();
                    break;
                case "stop":
                    moveitGoalPublisher.RequestIdentityPlan();
                    break;
            }
        }

        private void Update() {
            if (commandDetected) {
                frameCounter++;
            }
            if (frameCounter == 40) {
                commandDetected = false;
                frameCounter = 0;
                CurrentCommand = "";
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