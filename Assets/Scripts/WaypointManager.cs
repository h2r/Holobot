using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Academy.HoloToolkit.Unity
{
    public class WaypointManager : Singleton<WaypointManager>
    {
        public int waypointInd;
        public List<GameObject> Waypoints { get; private set; }
        // Use this for initialization

        void Awake()
        {
            waypointInd = 0;
            Waypoints = new List<GameObject>();
        }

        public void AddWaypoint(GameObject waypointObj)
        {
            Debug.Assert(waypointObj != null);
            Debug.Assert(Waypoints != null);
            Waypoints.Add(waypointObj);
        }

    }
}