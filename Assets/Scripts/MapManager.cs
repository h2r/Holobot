using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace HoloToolkit.Unity {
    //namespace Academy.HoloToolkit.Unity {
    public class MapManager : Singleton<MapManager> {

        public GameObject mapMetaData;
        public GameObject mapOrigin;
        public GameObject map;

        private float height;
        private float width;
        private float robotXPos;
        private float robotYPos;
        private float robotZPos;
        private Vector3 robotRosPose;

        // Use this for initialization
        void Start() {
            map.SetActive(false);
            //mapMetaData = GameObject.Find("map_metadata");
            //mapOrigin = GameObject.Find("map_to_baselink");
            //map = GameObject.Find("map");

        }

        public void CalibrateMap() {
            if (StateManager.Instance.RobotCalibrated) {
                height = mapMetaData.GetComponent<RosSharp.RosBridgeClient.MapReceiver>().mapMetreHeight;
                width = mapMetaData.GetComponent<RosSharp.RosBridgeClient.MapReceiver>().mapMetreWidth;
                robotXPos = mapOrigin.GetComponent<RosSharp.RosBridgeClient.MapToBaselinkReceiver>().robotX;
                robotYPos = mapOrigin.GetComponent<RosSharp.RosBridgeClient.MapToBaselinkReceiver>().robotY;
                robotZPos = mapOrigin.GetComponent<RosSharp.RosBridgeClient.MapToBaselinkReceiver>().robotZ;

                robotRosPose = GetUnityCoords(new Vector2(robotXPos, robotYPos));

                map.transform.localScale = new Vector3(width, 0.1f, height);
                map.transform.position = new Vector3((StateManager.Instance.MovoUnityStartPose.Y + (0.5f * width)) - robotRosPose.z, 
                    StateManager.Instance.FloorY, (StateManager.Instance.MovoUnityStartPose.X + (0.5f * height)) - robotRosPose.x);
                map.SetActive(true);

            }
        }

        public Vector3 GetUnityCoords(Vector2 RosPose) {
            Vector2 ROSCoords = RosPose;
            Vector2 UnityCoords = ROSCoords;
            UnityCoords.x -= StateManager.Instance.MovoROSStartPose.X;
            UnityCoords.y -= StateManager.Instance.MovoROSStartPose.Y;
            Transform robotObjTransform = GameObject.Find("Movo").transform;
            Vector3 UnityPosition = robotObjTransform.TransformPoint(-UnityCoords.y, StateManager.Instance.FloorY, UnityCoords.x);
            return new Vector3(UnityPosition.x, StateManager.Instance.FloorY, UnityPosition.z);
        }
    }
}
