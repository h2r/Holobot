﻿using UnityEngine;
using UnityEngine.UI;
using UnityEngine.EventSystems;
using System;

namespace Academy.HoloToolkit.Unity {
    public class WaypointPlace : MonoBehaviour {
        bool placingEnabled;
        bool waypointPlaced;
        GameObject waypointObj;
        GameObject coordTextObj;
        Text coordText;

        private void Start() {
            Reset();
        }

        private void Reset() {
            placingEnabled = true;
            waypointPlaced = false;
            //WaypointManager.Instance.InitializeWaypoints();
        }

        public void Disable() {
            //WaypointManager.Instance.ClearWaypoints();
            placingEnabled = false;
        }

        // Called by GazeGestureManager when the user performs a Select gesture
        void OnSelect() {
            if (waypointPlaced) {
                waypointPlaced = false;
                WaypointManager.Instance.AddWaypoint();
                Debug.Assert(placingEnabled = true);
            }
        }

        // Update is called once per frame
        void Update() {
            if (StateManager.Instance.TransitionedToWaypointState) {
                Reset();
                StateManager.Instance.TransitionedToWaypointState = false;
            }
            if (StateManager.Instance.CurrentState != StateManager.State.WaypointState) {
                Disable();
                return;
            }
            //if (!placingEnabled) {
            //    //Reset();
            //}
            //Debug.Assert(placingEnabled);
            Debug.Assert(StateManager.Instance.CurrentState == StateManager.State.WaypointState);
            Waypoint curr_waypoint = WaypointManager.Instance.GetLastWaypoint();
            waypointObj = curr_waypoint.WaypointObj;

            //m_EventSystem.SetSelectedGameObject(waypointObj);

            Utils.RaycastPlace(Camera.main, waypointObj);
            coordTextObj = WaypointManager.Instance.GetLastWaypoint().CoordTextObj;
            Debug.Assert(coordTextObj != null);
            coordText = coordTextObj.GetComponent<Text>();
            Vector2 coords = curr_waypoint.GetCoords();
            string msg = string.Format("{0}\n({1}, {2})", waypointObj.name, Math.Round(coords[0], 1), Math.Round(coords[1], 1));
            coordText.text = msg;
            if (!waypointPlaced) {
                waypointPlaced = true;
            }
        }
    }
}