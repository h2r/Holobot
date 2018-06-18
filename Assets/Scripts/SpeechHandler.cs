using UnityEngine;
using HoloToolkit.Unity.InputModule;
using RosSharp.RosBridgeClient;

namespace Academy.HoloToolkit.Unity {
    public class SpeechHandler : MonoBehaviour, ISpeechHandler {

        //public MoveItGoalPublisher MoveItGoalPublisher;
        //public DisplayTrajectoryReceiver DisplayTrajectoryReceiver;

        void ISpeechHandler.OnSpeechKeywordRecognized(SpeechEventData eventData) {
            //Debug.Log(eventData.RecognizedText);
            switch (eventData.RecognizedText.ToLower()) {
                case "calibrate":
                    Calibrate();
                    break;
                case "place":
                    Place();
                    break;
            }
        }

        // Calibrate Movo
        public void Calibrate() {
            if (StateManager.Instance.CurrentState == StateManager.State.CalibratingState) {
                Debug.Log("Calibrating!");
                GameObject movoObj = GameObject.Find("Movo");
                StateManager.Instance.FloorY = movoObj.transform.position.y;
                Vector3 movoUnityPos = movoObj.transform.position;
                StateManager.Instance.MovoUnityStartPose = new Pose(-movoUnityPos.z, movoUnityPos.x, movoObj.transform.eulerAngles.y);
                StateManager.Instance.MovoROSStartPose = StateManager.Instance.MovoROSPose;
                Debug.Assert(StateManager.Instance.MovoROSStartPose != null);
                StateManager.Instance.RobotCalibrated = true;
                StateManager.Instance.CurrentState = StateManager.State.WaypointState;
                Utils.InitWaypointPos(Camera.main, WaypointManager.Instance.Waypoints[0].WaypointObj);
                StateManager.Instance.MovoUnityToROSOffset = StateManager.Instance.MovoROSStartPose - StateManager.Instance.MovoUnityStartPose;
            }
        }

        // Place waypoint
        public void Place() {
            if (StateManager.Instance.CurrentState == StateManager.State.WaypointState) {
                Debug.Log("Placing waypoint!");
                WaypointManager.Instance.AddWaypoint();
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