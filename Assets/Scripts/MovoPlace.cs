using UnityEngine;
using UnityEngine.UI;
using System;

namespace Academy.HoloToolkit.Unity {
    public class MovoPlace : MonoBehaviour {
        bool robotPlaced;
        GameObject robotObj;

        private void Start() {
            Reset();
        }

        private void Reset() {
            //SpatialMapping.Instance.DrawVisualMeshes = true;
            robotPlaced = false;
            robotObj = GameObject.Find("Movo");
        }

        public void Disable() {
            //SpatialMapping.Instance.DrawVisualMeshes = false;
        }

        // Called by GazeGestureManager when the user performs a Select gesture
        void OnSelect() {
            if (!robotPlaced) {
                robotPlaced = true;
                StateManager.Instance.RobotCalibrated = true;
                StateManager.Instance.CurrentState = StateManager.State.WaypointState;
                Disable();
            }
        }

        // Update is called once per frame
        void Update() {
            if (StateManager.Instance.CurrentState != StateManager.State.CalibratingState) {
                Disable();
                return;
            }
            if (StateManager.Instance.RobotCalibrated == true) {
                Disable();
                return;
            }
            Debug.Assert(StateManager.Instance.RobotCalibrated == false);
            Debug.Assert(robotObj != null);
            if (!robotPlaced) {
                Utils.RaycastPlace(Camera.main, robotObj);
            }
        }
    }
}