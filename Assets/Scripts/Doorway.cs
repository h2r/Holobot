using UnityEngine;
using UnityEngine.UI;
using UnityEngine.EventSystems;
using System;
using System.Collections.Generic;

namespace HoloToolkit.Unity {
    //namespace Academy.HoloToolkit.Unity {
    public class Doorway : MonoBehaviour {
        GameObject coordTextObj;
        public Pose ROSpose;
        public String roomLocation;
        //Waypoint thisWaypoint;
        //Text coordText;

        private void Start() {
            //thisWaypoint = null;
            roomLocation = "NO ROOM FOR THIS DOOR";
            foreach (Transform t in transform) {
                if (t.name == "New Text")// Do something to child one
                    {
                    coordTextObj = t.gameObject;//this.gameObject.transform.GetChild(0).gameObject;
                    //Debug.Log("new text found!");
                }
            }

        }

        // Update is called once per frame
        void Update() {
            if (StateManager.Instance.CurrentState != StateManager.State.LabelState) {
                return;
            }

            var pos = transform.position;
            transform.position = new Vector3(pos.x, StateManager.Instance.FloorY, pos.z);
            Debug.Assert(StateManager.Instance.CurrentState == StateManager.State.LabelState);
            ////Waypoint curr_waypoint = WaypointManager.Instance.GetLastWaypoint();
            //if (thisWaypoint == null) {
            //    thisWaypoint = WaypointManager.Instance.GetLastWaypoint();
            //    Debug.Log(name + " waypoint set!");
            //}
            //waypointObj = thisWaypoint.WaypointObj;
            //Debug.Assert(waypointObj != null);

            ////coordTextObj = WaypointManager.Instance.GetLastWaypoint().CoordTextObj;
            //coordTextObj = thisWaypoint.CoordTextObj;
            //Debug.Assert(coordTextObj != null);
            //coordText = coordTextObj.GetComponent<Text>();
            ROSpose = UpdatePose(calibThetaOffset: -StateManager.Instance.CalibrateThetaOffset);
            //Debug.Log(name + " Pose updated!");
            //Pose pose = thisWaypoint.Pose;
            string msg = string.Format("{0}\n({1}, {2}, {3})", this.name, Math.Round(ROSpose.X, 1), Math.Round(ROSpose.Y, 1), Math.Round(ROSpose.Theta, 1));
            //Debug.Log(msg);
            coordTextObj.GetComponent<TextMesh>().text = msg;
            updateRoomLocation();
        }

        public void updateRoomLocation() {
            foreach (KeyValuePair<string, GameObject[]> places in LabelManager.Instance.LabelDict) 
                {
                Vector3 corner1 = places.Value[0].transform.position;
                Vector3 corner2 = places.Value[1].transform.position;
                if (Math.Min(corner1.x, corner2.x) < this.transform.position.x && Math.Max(corner1.x, corner2.x) > this.transform.position.x) 
                    {
                    if (Math.Min(corner1.z, corner2.z) < this.transform.position.z && Math.Max(corner1.z, corner2.z) > this.transform.position.z) {
                        roomLocation = places.Key;
                        this.name = roomLocation;
                    }
                }
            }
        }

        public Pose UpdatePose(float calibThetaOffset = 0) {
            Vector2 coords = GetCoords();
            Debug.Assert(StateManager.Instance.MovoUnityToROSOffset != null);
            float theta = transform.eulerAngles.y + StateManager.Instance.MovoUnityToROSOffset.Theta + calibThetaOffset;
            //Debug.Log("theta: " + theta);
            Pose ret = new Pose(coords.x, coords.y, -theta); // ROS theta goes counterclockwise
            return ret;
        }

        public Vector2 GetCoords() {
            Transform robotObjTransform = GameObject.Find("Movo").transform;
            Vector3 relativePos = robotObjTransform.InverseTransformPoint(transform.position);
            var x_coord = relativePos.z; // + StateManager.Instance.MovoROSStartPose.X;
            var y_coord = -relativePos.x; // + StateManager.Instance.MovoROSStartPose.Y;
            x_coord += StateManager.Instance.MovoROSStartPose.X;
            y_coord += StateManager.Instance.MovoROSStartPose.Y;
            return new Vector2(x_coord, y_coord);
        }

    }
}