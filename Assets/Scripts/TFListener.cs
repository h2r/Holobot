using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
using Academy.HoloToolkit.Unity;

public class TFListener : MonoBehaviour
{

    //private UWPWebSocketClient UWPwsc;
    //private WebsocketClient wsc;
    private UniversalWebsocketClient wsc;
	string topic = "ros_unity";
    public GameObject root;
    public float rotation = 0f;
    Hashtable links = new Hashtable();

    public float scale = 1f;
	// Use this for initialization
	void Start ()
	{
        //if build
        GameObject wso = GameObject.Find("WebsocketClient");
#if UNITY_EDITOR
        wsc = wso.GetComponent<WebsocketClient>();
#else
        wsc = wso.GetComponent<UWPWebSocketClient>();
#endif
        wsc.Subscribe (topic, "std_msgs/String", "none", 0);

	}

    void Update() {
        //foreach (KeyValuePair<string, string> kvp in wsc.messages)
        //{
        //    Debug.Log("hifdhjf");
        //    //textBox3.Text += ("Key = {0}, Value = {1}", kvp.Key, kvp.Value);
        //    Debug.LogFormat("Key = {0}, Value = {1}", kvp.Key, kvp.Value);
        //}
        //Debug.Log("check");
        // Issue: wsc.messages is empty when it shouldn't be
        // Fix: use try/catch - only run code if wsc is not empty
        if (GestureManager.Instance.RobotCalibrating)
        {
            return;
        }
        string message;
        try
        {
            message = wsc.messages[topic]; //get newest robot state data (from transform)
        }
        catch
        {
            return;
        }
        //Debug.Log("check2");
        string[] tfElements = message.Split(';'); //split the message into each joint/link data pair
        //Debug.Log(string.Join(", ", tfElements));
        foreach (string tfElement in tfElements) {
            //Debug.Log(tfElement);
            //continue;
            string[] dataPair = tfElement.Split(':');
            GameObject cur = GameObject.Find(dataPair[0] + "Pivot"); // replace with hashmap
            if (cur != null) {

                string[] tmp = dataPair[1].Split('^'); //seperate position from rotation data
                string pos = tmp[0]; //position data
                string rot = tmp[1]; //rotation data
                pos = pos.Substring(1, pos.Length - 2);
                rot = rot.Substring(1, rot.Length - 2);
                string[] poses = pos.Split(',');
                float pos_x = float.Parse(poses[0]); //x position
                float pos_y = float.Parse(poses[1]); //y position
                float pos_z = float.Parse(poses[2]); //z position

                //if (dataPair[0] == "right_wrist")
                //{
                //    Debug.Log(String.Format("Wrist pos: ({0}, {1}, {2})", pos_x, pos_y, pos_z));
                //}


                Vector3 curPos = new Vector3(pos_x, pos_y, pos_z); //save current position
                string[] rots = rot.Split(',');
                //save rotation as quaternions
                float rot_x = float.Parse(rots[0]);
                float rot_y = float.Parse(rots[1]);
                float rot_z = float.Parse(rots[2]);
                float rot_w = float.Parse(rots[3]);


                Quaternion curRot = new Quaternion(rot_x, rot_y, rot_z, rot_w);

                cur.transform.localPosition = RosToUnityPositionAxisConversion(curPos); //convert ROS coordinates to Unity coordinates and scale for position vector
                cur.transform.localRotation = RosToUnityQuaternionConversion(curRot); //convert ROS quaternions to Unity quarternions
                if (!cur.name.Contains("kinect")) { //rescaling direction of kinect point cloud
                    cur.transform.localScale = new Vector3(scale, scale, scale);
                }
                else {
                    cur.transform.localScale = new Vector3(-scale, scale, -scale);
                }
                cur.transform.position = RosToUnityPositionAxisConversion (curPos);
                cur.transform.rotation = RosToUnityQuaternionConversion (curRot);
            }
        }
    }

    Vector3 RosToUnityPositionAxisConversion (Vector3 rosIn)
	{
        // Debug.Log(GestureManager.Instance.RobotOffset);
        return new Vector3(-rosIn.x, rosIn.z, -rosIn.y) * scale + GestureManager.Instance.RobotOffset;// + robot.transform.position;	
	}

	Quaternion RosToUnityQuaternionConversion (Quaternion rosIn)
	{
        return new Quaternion(rosIn.x, -rosIn.z, rosIn.y, rosIn.w);
    }


		

	//Vector3 RosToUnityRotationAxisConversion(Quaternion rosIn) OLD, I don't think this ever worked
	//{
	//	float roll = Mathf.Atan2 (2 * rosIn.y * rosIn.w + 2 * rosIn.x * rosIn.z, 1 - 2 * rosIn.y * rosIn.y - 2 * rosIn.z * rosIn.z);
	//	float pitch = Mathf.Atan2 (2 * rosIn.x * rosIn.w + 2 * rosIn.y * rosIn.z, 1 - 2 * rosIn.x * rosIn.x - 2 * rosIn.z * rosIn.z);
	//	float yaw = Mathf.Asin (2 * rosIn.x * rosIn.y + 2 * rosIn.z * rosIn.w);
	//	return new Vector3 (-rosIn.x, rosIn.z, -rosIn.y);
	//}


}
