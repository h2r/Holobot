using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using System;
using System.IO;
using UnityEngine;
using HoloToolkit.Unity.InputModule;
using HoloToolkit.Unity.InputModule.Utilities.Interactions;
using System.Collections.Generic;

namespace HoloToolkit.Unity {
//namespace Academy.HoloToolkit.Unity {
    public class LabelManager : Singleton<LabelManager> {
        public int WaypointInd { get; set; }
        //public List<Label> Labelpoints { get; private set; }
        public Dictionary<string, GameObject[]> LabelDict = new Dictionary<string, GameObject[]>();
        public GameObject WaypointTemplate;
        protected string text = " "; // assigned to allow first line to be read below
        private UniversalWebsocketClient wsc;
        private readonly string labelSaveTopic = "holocontrol/save_labels";
        private readonly string labelLoadITopic = "holocontrol/load_labels_indicator";

        // Use this for initialization

        //void Awake() {
        void Start() {
            Debug.Log("Initialized Label Manager");
            //WTemplate = GameObject.Find("Waypoint0");
            //Labelpoints = new List<Label>();
            Debug.Log("LabelManager Awake()");
            GameObject wso = GameObject.Find("WebsocketClient");
            #if UNITY_EDITOR
                        wsc = wso.GetComponent<WebsocketClient>();
#else
                    wsc = wso.GetComponent<UWPWebSocketClient>();
#endif
            wsc.Advertise(labelSaveTopic, "std_msgs/String");
        }

        //public void ClearWaypoints() {
        //    if (Waypoints == null) { // if there's a bug, change Start() back to Awake().
        //        return;
        //    }
        //    foreach (Waypoint wp in Waypoints) {
        //        Destroy(wp.WaypointObj);
        //    }
        //    Waypoints = new List<Waypoint>();
        //    WaypointInd = 0;
        //}

        public void InitializeWaypoints() {
            Debug.Log("InitializeWaypoints()");
            //ClearWaypoints();
            //AddWaypoint();
        }

       

        public GameObject GetCoordTextObj(GameObject waypointObj) {
            return waypointObj.transform.GetChild(0).gameObject.transform.GetChild(0).gameObject;
        }

        public void SaveLabels() 
        {
            string msg = "empty message here";
            string c = "";
            string label_name;
            foreach (KeyValuePair <string, GameObject[]> label_pair in LabelDict) {
                //Now you can access the key and value both separately from this attachStat as:
                GameObject l1 = label_pair.Value[0];
                GameObject r1 = label_pair.Value[1];
                label_name = l1.name.Split()[0];
                float lx = l1.GetComponent<LabelPlace>().ROSpose.X; //ROS poses
                float ly = l1.GetComponent<LabelPlace>().ROSpose.Y; //ROS poses
                float rx = r1.GetComponent<LabelPlace>().ROSpose.X; //ROS poses
                float ry = r1.GetComponent<LabelPlace>().ROSpose.Y; //ROS poses
                msg = label_name + " " + lx.ToString() + " " + ly.ToString() + " " + rx.ToString() + " " + ry.ToString();
                Debug.Log(msg);
                //wsc.Publish(labelSaveTopic, msg);
                c = c + msg + ";";
            }
            //wsc.Publish(labelSaveTopic, msg);
            Debug.Log(c);
            wsc.Publish(labelSaveTopic, c);
            //wsc.Publish(labelSaveTopic, "heyo!");

        }

        public void LoadLabels() {
            //string path = Path.Combine(Application.persistentDataPath, "MyFile3.txt");
            //theSourceFile = new FileInfo(path);
            //reader = theSourceFile.OpenText();
            //Debug.Log("trying to read file");
            //while (text != null) {
            //    text = reader.ReadLine();
            //    Console.WriteLine(text);
            //}
            //Debug.Log("done reading file!");
            //reader.Close();
            wsc.Publish(labelLoadITopic, "loadem");
        }

        public void AddLabel() {
            Debug.Log("AddLabel()");
            GameObject label1 = Instantiate(WaypointTemplate); // This waypoint will eventually be destroyed, so Instantiate ensures that WaypointTemplate is always there.
            GameObject label2 = Instantiate(WaypointTemplate); // This waypoint will eventually be destroyed, so Instantiate ensures that WaypointTemplate is always there.

            if (StateManager.Instance.CurrentState == StateManager.State.LabelState) { // If in WaypointState, then place waypoint in front of user.
                UtilFunctions.InitLabelPos(Camera.main, label1, label2);
                LabelDict.Add(LabelDict.Count.ToString(), new[] { label1, label2 });
                label1.name = LabelDict.Count.ToString() + " L";
                label2.name = LabelDict.Count.ToString() + " R";
            }

            Debug.Log("Finished adding");
            Debug.Log(LabelDict.Count.ToString());
            //WaypointInd++;
            //waypointObj.name = String.Format("Waypoint{0}", WaypointInd);
            //GameObject coordTextObj = GetCoordTextObj(waypointObj);
            //Debug.Assert(coordTextObj != null);
            //coordTextObj.name = String.Format("WaypointCoord{0}", WaypointInd);
            //Waypoints.Add(new Waypoint(waypointObj, WaypointInd));
            //Debug.Log(Waypoints.Count + " waypoints exist.");
            //Debug.Assert(GetLastWaypoint().Name == waypointObj.name);
        }

        //public Waypoint GetLastWaypoint() {
        //    if (Waypoints.Count == 0) {
        //        return null;
        //    }
        //    return Waypoints[Waypoints.Count - 1];
        //}

        private void Update() {
            if (StateManager.Instance.CurrentState != StateManager.State.WaypointState) {
                return;
            }
            //if (Waypoints.Count == 0) {
            //    InitializeWaypoints();
            //}
            //Waypoint lastWaypoint = GetLastWaypoint();
            //if (!lastWaypoint.Placed) {
                //UtilFunctions.FollowGaze(Camera.main, lastWaypoint.WaypointObj);
            //}
        }

    }
}