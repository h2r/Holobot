using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Academy.HoloToolkit.Unity
{
    public class WaypointManager : Singleton<WaypointManager>
    {
        public int WaypointInd { get; set; }
        public List<Waypoint> Waypoints { get; private set; }
        // Use this for initialization

        void Awake()
        {
            ResetWaypoints();
        }

        public void ResetWaypoints()
        {
            Waypoints = new List<Waypoint>();
            WaypointInd = 0;
        }

        public void AddWaypoint(GameObject waypointObj)
        {
            Debug.Assert(waypointObj != null);
            Debug.Assert(Waypoints != null);
            Waypoints.Add(new Waypoint(waypointObj, WaypointInd));
            WaypointInd++;
        }
        
    }
    public class Waypoint
    {
        public GameObject WaypointObj { get; set; }
        public Vector2 Coords { get; private set; }
        public String Name { get; private set; }
        public Waypoint(GameObject waypointObj, int WaypointInd)
        {
            waypointObj.transform.parent = GameObject.Find("Movo").transform;
            Name = String.Format("Waypoint{0}", WaypointInd);
            waypointObj.name = Name;
            WaypointObj = waypointObj;
            Coords = new Vector2(WaypointObj.transform.position.x, WaypointObj.transform.position.z);
        }
    }
}