using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Academy.HoloToolkit.Unity
{
    public class WaypointManager : Singleton<WaypointManager>
    {
        public int waypointInd;
        // Use this for initialization

        void Start()
        {
            waypointInd = 0;
        }

        public void IncrementInd()
        {
            waypointInd++;
        }

        // Update is called once per frame
        void Update()
        {

        }
    }
}