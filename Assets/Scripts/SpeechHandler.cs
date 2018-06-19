using UnityEngine;
using HoloToolkit.Unity.InputModule;
using RosSharp.RosBridgeClient;

namespace HoloToolkit.Unity {
    public class SpeechHandler : MonoBehaviour, ISpeechHandler {

        //public MoveItGoalPublisher MoveItGoalPublisher;
        //public DisplayTrajectoryReceiver DisplayTrajectoryReceiver;

        void ISpeechHandler.OnSpeechKeywordRecognized(SpeechEventData eventData) {
            //Debug.Log(eventData.RecognizedText);
            switch (eventData.RecognizedText.ToLower()) {
                case "calibrate":
                    MovoPlace.CalibrateMovo();
                    break;
                case "place":
                    Place();
                    break;
                case "move":
                    if (StateManager.Instance.CurrentState == StateManager.State.WaypointState) {
                        WaypointManager.Instance.TransitionToNavigatingState();
                    }
                    break;
                case "puppet":
                    ManipulateArms();
                    break;
            }
        }

        public void TransitionToWaypointState() {
            if (StateManager.Instance.CurrentState == StateManager.State.WaypointState) {
                return;
            }
            WaypointManager.Instance.InitializeWaypoints();
            StateManager.Instance.CurrentState = StateManager.State.WaypointState;
        }

        // Place waypoint
        public void Place() {
            if (StateManager.Instance.CurrentState != StateManager.State.WaypointState) {
                Debug.Log("\"PLACE\" ERROR: Must be in WaypointState!");
                return;
            }
            Debug.Log("Generating waypoint!");
            WaypointManager.Instance.AddWaypoint();
        }

        public void ManipulateArms() {

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