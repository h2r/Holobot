using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Text;
using System.IO;
using System.Linq;
using System;

public class TrajectoryVisualizer : MonoBehaviour {

    private UniversalWebsocketClient wsc;
    string topic = "trajectory_path";
    string oldPath = "";

    List<string> waypoints;
    public GameObject robot;
    public GameObject armPrefab;
    GameObject[] arms = new GameObject[0];
    int cur = 0;
    public bool predictable = true;
    public int traj_num = 1;

    

    void Start() {
        GameObject wso = GameObject.Find("WebsocketClient");
#if NETFX_CORE
        wsc = wso.GetComponent<UWPWebSocketClient> ();
#else
        wsc = wso.GetComponent<WebsocketClient>();
#endif

        wsc.Subscribe(topic, "std_msgs/String", "none", 0);
    }

    void Update() {
        string path;
        try {
            path = wsc.messages[topic];
            //path = "Trajectories/All/t" + traj_num;
            Debug.Log(path);
        }
        catch {
            path = "";
        }
        if (path != oldPath) {
            DestroyTrajectory();
            CreateTrajectory(path);
            oldPath = path;
        }
    }

    void CreateTrajectory (string path) {
        Load(path);
        CreateTrail();
        //InvokeRepeating("Animate", 0.1f, 0.1f);
	}

    void DestroyTrajectory() {
        foreach (GameObject a in arms) {
            Destroy(a);
            waypoints.Clear();
        }
    } 

    //string GeneratePath() {
    //    string path;
    //    if (predictable) {
    //        path = "Trajectories/Predictable/t" + traj_num + "_pred";
    //    } else {
    //        path = "Trajectories/Unpredictable/t" + traj_num + "_unpred";
    //    }
    //    return path;
    //}

    //void Update() {
    //    Animate();
    //}

    void Animate() {
        arms[cur].SetActive(false);
        if (++cur == arms.Length) {
            cur = 0;
        }
        arms[cur].SetActive(true);
    }

    void CreateTrail() {
        int c = 0;
        int n = waypoints.Count;
        Color trail = Color.blue;
        arms = new GameObject[waypoints.Count];
        foreach (string w in waypoints) {
            if (c % 50 == 0 && c > 2) {
                GameObject clone = Instantiate(armPrefab);
                clone.name = "clone" + c.ToString();
                armPrefab.transform.position = robot.transform.position;
                SetPosition(w, clone);
                arms[c] = clone;
                MeshRenderer[] renderers = clone.GetComponentsInChildren<MeshRenderer>();
                //trail.a = 0.5f - c / (2f*n) + 0.5f;
                trail.r = 0.5f - c / (2f * n);
                trail.g = 0.5f - c / (2f * n);
                trail.b = 0.95f - c / (2f * n);
                foreach (MeshRenderer r in renderers) {
                    foreach (Material m in r.materials) {
                        m.color = trail;
                        //m.SetOverrideTag("RenderType", "Transparent");
                        //m.SetInt("_SrcBlend", 1);
                        //m.SetInt("_DstBlend", 10);
                        //m.SetInt("_ZWrite", 0);
                        //m.DisableKeyword("_ALPHATEST_ON");
                        //m.DisableKeyword("_ALPHABLEND_ON");
                        //m.EnableKeyword("_ALPHAPREMULTIPLY_ON");
                        //m.renderQueue = 3000;
                    }
                }
                //clone.SetActive(false);
            }
            c++;
        }
    }


    void Load(string filename) {
        string[] separatingChars = {"---"};
        waypoints = new List<string>();

        TextAsset dataAsset = Resources.Load(filename) as TextAsset;
        waypoints = dataAsset.text.Split(separatingChars, StringSplitOptions.RemoveEmptyEntries).ToList();
        waypoints.RemoveAt(waypoints.Count - 1);
    }


    void SetPosition(string data, GameObject clone) {
        string[] dataPairs = data.Split(';');

        if (dataPairs.Length > 0) {
            for (int i = 0; i < dataPairs.Length; i++) {
                string[] dataPair = dataPairs[i].Split(':');
                string linkName = dataPair[0] + "Pivot";
                Transform cur = clone.transform.Find(linkName);
                if (cur != null) {

                    string[] tmp = dataPair[1].Split(')');
                    string pos = tmp[0];
                    string rot = tmp[1];
                    pos = pos.Substring(1, pos.Length - 1);
                    rot = rot.Substring(1, rot.Length - 1);

                    string[] poses = pos.Split(',');
                    float pos_x = float.Parse(poses[0]);
                    float pos_y = float.Parse(poses[1]);
                    float pos_z = float.Parse(poses[2]);

                    Vector3 curPos = new Vector3(pos_x, pos_y, pos_z);

                    string[] rots = rot.Split(',');
                    float rot_x = float.Parse(rots[0]);
                    float rot_y = float.Parse(rots[1]);
                    float rot_z = float.Parse(rots[2]);
                    float rot_w = float.Parse(rots[3]);

                    Quaternion curRot = new Quaternion(rot_x, rot_y, rot_z, rot_w);
                    cur.localPosition = RosToUnityPositionAxisConversion(curPos);
                    cur.localRotation = RosToUnityQuaternionConversion(curRot);
                    //cur.localScale = new Vector3(1.05f, 1.05f, 1.05f);
                }
            }
        }
    }

    Vector3 RosToUnityPositionAxisConversion(Vector3 rosIn) {
        return new Vector3(-rosIn.x, rosIn.z, -rosIn.y);// + robot.transform.position;	
    }

    Quaternion RosToUnityQuaternionConversion(Quaternion rosIn) {
        return new Quaternion(rosIn.x, -rosIn.z, rosIn.y, rosIn.w);
    }
}
