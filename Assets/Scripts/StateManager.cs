using UnityEngine;
using UnityEngine.XR.WSA.Input;
using System.Collections.Generic;
using System.IO;

namespace Academy.HoloToolkit.Unity
{
    public class StateManager : Singleton<GestureManager>
    {
        public Vector3 RobotOffset { get; set; }
        public bool RobotCalibrated { get; set; }
        public Vector3 RobotStartPos { get; set; }

        void Awake()
        {
            Debug.Log("Ahhhhhh");
            GameObject.Find("Waypoint0").SetActive(true);
            RobotOffset = Vector3.zero;
            RobotStartPos = Vector3.zero;
            RobotCalibrated = false;
        }
    }

    public class Record
    {
        public Vector3 Position { get; private set; }
        // public Quaternion Rotation { get; private set; }
        public float DelTime;
        public Record(Vector3 pos, float dt)
        {
            Position = pos;
            DelTime = dt;
        }
    }
}