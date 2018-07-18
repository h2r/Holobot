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
                case "transition label":
                    if (!StateManager.Instance.RobotCalibrated) {
                        return;
                    }
                    Debug.Log("Transitioned to label state");
                    StateManager.Instance.TransitionToLabelState();
                    break;
                case "transition puppet":
                    if (!StateManager.Instance.RobotCalibrated) {
                        return;
                    }
                    StateManager.Instance.TransitionToPuppetState();
                    break;
                case "transition arm trail":
                    if (!StateManager.Instance.RobotCalibrated) {
                        return;
                    }
                    StateManager.Instance.TransitionToArmTrailState();
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
                case StateManager.State.LabelState:
                    ParseLabelCommands(CurrentCommand);
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
                    MapManager.Instance.CalibrateMap();
                    break;
            }
        }

        private void ParseLabelCommands(string command) {
            Debug.Log("ParseLabelCommands()");
            if (StateManager.Instance.CurrentState != StateManager.State.LabelState) {
                return;
            }
            switch (command) {
                case "save labels":
                    Debug.Log("Imma save the labels");
                    LabelManager.Instance.SaveLabels();
                    break;
            }
            switch (command) {
                case "add label":
                    Debug.Log("Imma add the labels");
                    LabelManager.Instance.AddLabel();
                    break;
            }
            switch (command) {
                case "move first":
                    Debug.Log("Imma move to first!");
                    GameObject.Find("TFListener").GetComponent<TFListener>().MoveFirst();
                    break;
            }
            switch (command) {
                case "load labels":
                    Debug.Log("Imma load the labels!");
                    LabelManager.Instance.LoadLabels();
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
                    //WaypointManager.Instance.TransitionToNavigatingState();
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
                case "open right":
                    StateManager.Instance.EinCommandsToExecute.Add("switchToRightArm openGripper");
                    break;
                case "open left":
                    StateManager.Instance.EinCommandsToExecute.Add("switchToLeftArm openGripper");
                    break;
                case "close right":
                    StateManager.Instance.EinCommandsToExecute.Add("switchToRightArm closeGripper");
                    break;
                case "close left":
                    StateManager.Instance.EinCommandsToExecute.Add("switchToLeftArm closeGripper");
                    break;
                case "move right arm":
                    StateManager.Instance.UpdateRightArm = true;
                    StateManager.Instance.UpdateLeftArm = false;
                    break;
                case "move left arm":
                    StateManager.Instance.UpdateRightArm = false;
                    StateManager.Instance.UpdateLeftArm = true;
                    break;
                case "stop":
                    StateManager.Instance.UpdateRightArm = false;
                    StateManager.Instance.UpdateLeftArm = false;
                    break;
            }
        }

        private void ParseArmTrailCommands(string command) {
            if (StateManager.Instance.CurrentState != StateManager.State.ArmTrailState) {
                return;
            }
            switch (command) {
                case "switch right arm":
                    break;
                case "switch left arm":
                    break;
                case "plan":
                    break;
                case "move":
                    break;
                case "stop":
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