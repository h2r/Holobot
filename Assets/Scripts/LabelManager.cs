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
        public GameObject StaticLocTemplate;
        protected string text = " "; // assigned to allow first line to be read below
        private UniversalWebsocketClient wsc;
        private readonly string labelSaveTopic = "holocontrol/save_labels";
        private readonly string labelLoadITopic = "holocontrol/load_labels_indicator";
        private readonly string labelLoadTopic = "holocontrol/loaded_labels";
        private string lastLoadLabels;

        // Use this for initialization

        //void Awake() {
        void Start() {
            lastLoadLabels = "nothing";
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
            wsc.Advertise(labelLoadITopic, "std_msgs/String");
            wsc.Subscribe(labelLoadTopic, "std_msgs/String", "none", 0);
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
            GameObject midpoint = Instantiate(StaticLocTemplate); // This waypoint will eventually be destroyed, so Instantiate ensures that WaypointTemplate is always there.
            midpoint.GetComponent<TapToGo>().corner1 = label1;
            midpoint.GetComponent<TapToGo>().corner2 = label2;

            if (StateManager.Instance.CurrentState == StateManager.State.LabelState) { // If in WaypointState, then place waypoint in front of user.
                UtilFunctions.InitLabelPos(Camera.main, label1, label2);

                LabelDict.Add(LabelDict.Count.ToString(), new[] { label1, label2 });
                label1.name = LabelDict.Count.ToString() + " L";
                label2.name = LabelDict.Count.ToString() + " R";
                midpoint.name = LabelDict.Count.ToString() + "M";
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
        private string GetROSMessage(string topic_input) {
            string[] components = topic_input.Split(':');
            foreach (string component in components) {
                if (component.Contains("\"op\"")) {
                    var dataPair = component.Split('}');
                    var msg = dataPair[0].Trim(new Char[] { ' ', '"' });
                    return msg;
                }
            }
            return null;
        }

        private Vector3 GetUnityPos(Vector3 pos) 
        {
            var x_coord = pos.x - StateManager.Instance.MovoROSStartPose.X;
            var y_coord = pos.z - StateManager.Instance.MovoROSStartPose.Y;

            y_coord = -y_coord;

            Vector3 temp = new Vector3(x_coord, StateManager.Instance.FloorY, y_coord);

            Transform robotObjTransform = GameObject.Find("Movo").transform;
            Vector3 relativePos = robotObjTransform.TransformPoint(temp);
            return relativePos;
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

        private Vector3 GetUnityPos2(Vector3 pos) {
            var x_coord = pos.x - StateManager.Instance.MovoUnityToROSOffset.X;
            var y_coord = pos.z - StateManager.Instance.MovoUnityToROSOffset.Y;

            y_coord = -y_coord;

            Vector3 temp = new Vector3(x_coord, StateManager.Instance.FloorY, y_coord);

            Transform robotObjTransform = GameObject.Find("Movo").transform;
            Vector3 relativePos = robotObjTransform.TransformPoint(temp);
            return relativePos;
        }



        private void Update() {
            //Debug.Log(lastLoadLabels);
            try {
                //Debug.Log(wsc.messages[labelLoadTopic]);
                string msg = GetROSMessage(wsc.messages[labelLoadTopic]);

                if (msg == lastLoadLabels)  //these labels have already been loaded
                    {
                    return;
                }

                Debug.Log(msg);
                string[] lines = msg.Split('$');
                //Debug.Log(lines);
                //Debug.Log(lines[0]);
                //Debug.Log(lines[1]);
                //Debug.Log(lines.Length);
                for (int i = 0; i < lines.Length-2; i++) {
                    string line = lines[i];
                    string[] split_line = line.Split(' ');
                    Debug.Log(lines[i]);
                    Debug.Log(split_line[0]);
                    string l_name = split_line[0];
                    float xl = float.Parse(split_line[1]);// - StateManager.Instance.MovoUnityToROSOffset.Y;
                    float yl = float.Parse(split_line[2]);// - StateManager.Instance.MovoUnityToROSOffset.X;
                    float xr = float.Parse(split_line[3]);// - StateManager.Instance.MovoUnityToROSOffset.Y; ;
                    float yr = float.Parse(split_line[4]);// - StateManager.Instance.MovoUnityToROSOffset.X;



                    Debug.Log("Loading in a label");
                    GameObject label1 = Instantiate(WaypointTemplate); // This waypoint will eventually be destroyed, so Instantiate ensures that WaypointTemplate is always there.
                    GameObject label2 = Instantiate(WaypointTemplate); // This waypoint will eventually be destroyed, so Instantiate ensures that WaypointTemplate is always there.

                    GameObject midpoint = Instantiate(StaticLocTemplate); // This waypoint will eventually be destroyed, so Instantiate ensures that WaypointTemplate is always there.
                    midpoint.GetComponent<TapToGo>().corner1 = label1;
                    midpoint.GetComponent<TapToGo>().corner2 = label2;


                    if (StateManager.Instance.CurrentState == StateManager.State.LabelState) { // If in WaypointState, then place waypoint in front of user.
                        UtilFunctions.InitLabelPos(Camera.main, label1, label2);
                        LabelDict.Add(l_name, new[] { label1, label2 });
                        label1.name = l_name + " L";
                        label2.name = l_name + " R";
                        midpoint.name = l_name + "M";

                        label1.transform.position = GetUnityCoords(new Vector2(xl, yl));//GetUnityPos(new Vector3(xl, label1.transform.position.y, yl)); //new Vector3(xl - StateManager.Instance.MovoUnityToROSOffset.X, label1.transform.position.y, yl - StateManager.Instance.MovoUnityToROSOffset.Y);
                        label2.transform.position = GetUnityCoords(new Vector2(xr, yr));//GetUnityPos(new Vector3(xr, label1.transform.position.y, yr)); //new Vector3(xr - StateManager.Instance.MovoUnityToROSOffset.X, label1.transform.position.y, yr - StateManager.Instance.MovoUnityToROSOffset.Y);
                    }

                    Debug.Log("Finished adding");
                    Debug.Log(LabelDict.Count.ToString());
                }

                lastLoadLabels = msg;
            }
            catch (Exception e) {
                Debug.Log("Waiting to load labels...");
            }
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