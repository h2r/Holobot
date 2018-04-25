using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Academy.HoloToolkit.Unity {
    public class WaypointManager : Singleton<WaypointManager> {
        public int WaypointInd { get; set; }
        public List<Waypoint> Waypoints { get; private set; }
        private GameObject WaypointTemplate;
        // Use this for initialization

        void Awake() {
            Debug.Log("Initialized WaypointManager");
            WaypointTemplate = GameObject.Find("Waypoint0");
            WaypointTemplate.GetComponent<Renderer>().enabled = false;
            ClearWaypoints();
            //InitializeWaypoints();
        }

        public void ClearWaypoints() {
            WaypointTemplate.GetComponent<Renderer>().enabled = false;
            Waypoints = new List<Waypoint>();
            WaypointInd = 0;
            StateManager.Instance.PlacingWaypoints = false;
        }

        public void InitializeWaypoints() {
            ClearWaypoints();
            WaypointTemplate.GetComponent<Renderer>().enabled = true;
            AddWaypoint();
            StateManager.Instance.PlacingWaypoints = true;
        }

        public GameObject GetCoordTextObj(GameObject waypointObj) {
            return waypointObj.transform.GetChild(0).gameObject.transform.GetChild(0).gameObject;
        }

        public void AddWaypoint() {
            GameObject waypointObj = WaypointTemplate;
            if (Waypoints.Count > 0) {
                waypointObj = Instantiate(GetLastWaypoint().WaypointObj);
            }
            Waypoints.Add(new Waypoint(waypointObj, WaypointInd));
            WaypointInd++;
            waypointObj.name = String.Format("Waypoint{0}", WaypointInd);
            GameObject coordTextObj = GetCoordTextObj(waypointObj);
            Debug.Assert(coordTextObj != null);
            coordTextObj.name = String.Format("WaypointCoord{0}", WaypointInd);
        }

        public Waypoint GetLastWaypoint() {
            return Waypoints[Waypoints.Count - 1];
        }

        private void Update() {
            
        }

    }
    public class Waypoint {
        public GameObject WaypointObj { get; private set; }
        public GameObject CoordTextObj { get; private set; }
        public String Name { get; private set; }
        private double Deg2rad(float angle) {
            return (Math.PI / 180) * angle;
        }
        public Vector2 GetCoords() {
            Transform robotObj = GameObject.Find("Movo").transform;
            Vector3 relativePos = robotObj.InverseTransformDirection(WaypointObj.transform.position);
            double theta = Deg2rad(robotObj.transform.eulerAngles.y);
            Debug.Assert((theta - 45) < 0.1);
            var x_coord = relativePos.x * Math.Cos(theta) + relativePos.y * Math.Sin(theta);
            var y_coord = -relativePos.x * Math.Sin(theta) + relativePos.z * Math.Cos(theta);
            return new Vector2((float)x_coord, (float)y_coord);
        }
        public Waypoint(GameObject waypointObj, int WaypointInd) {
            waypointObj.transform.parent = GameObject.Find("Movo").transform;
            Name = String.Format("Waypoint{0}", WaypointInd);
            waypointObj.name = Name;
            WaypointObj = waypointObj;
            CoordTextObj = WaypointManager.Instance.GetCoordTextObj(waypointObj);
        }
    }
}