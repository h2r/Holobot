using UnityEngine;
using System;
using System.Collections;

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
#if NETFX_CORE
        wsc = wso.GetComponent<UWPWebSocketClient> ();
#else
        wsc = wso.GetComponent<WebsocketClient>();
#endif

        wsc.Subscribe (topic, "std_msgs/String", "none", 0);

	}

	void Update () // maybe should be fixed update
	{
        try {
        string message = wsc.messages[topic];
			//Debug.Log (message);
			string[] dataPairs = message.Split (';');

			if (dataPairs.Length > 0) {
				for (int i = 0; i < dataPairs.Length; i++) {
					string[] dataPair = dataPairs [i].Split (':');
                    string linkName = dataPair[0] + "Pivot";

                    if(!links.ContainsKey(linkName)) {
                        links[linkName] = root.transform.Find(linkName);
                    }
                    Transform cur = (Transform) links[linkName];
					if (cur != null) {

						string[] tmp = dataPair [1].Split (')');
						string pos = tmp [0];
						string rot = tmp [1];
						pos = pos.Substring (1, pos.Length - 1);
						rot = rot.Substring (1, rot.Length - 1);

						string[] poses = pos.Split (',');
						float pos_x = float.Parse (poses [0]);
						float pos_y = float.Parse (poses [1]);
						float pos_z = float.Parse (poses [2]);

						Vector3 curPos = new Vector3 (pos_x, pos_y, pos_z);


						string[] rots = rot.Split (',');
						float rot_x = float.Parse (rots [0]);
						float rot_y = float.Parse (rots [1]);
						float rot_z = float.Parse (rots [2]);
						float rot_w = float.Parse (rots [3]);


						Quaternion curRot = new Quaternion (rot_x, rot_y, rot_z, rot_w);
                        cur.localPosition = RosToUnityPositionAxisConversion(curPos);
                        cur.localRotation = RosToUnityQuaternionConversion(curRot);

                        if (!cur.name.Contains("kinect")) {
							cur.localScale = new Vector3(scale, scale, scale);
						} else {
							cur.localScale = new Vector3(-scale, scale, -scale);
						}
					}
				}
			}
		} catch (Exception e) {
            Debug.Log(e.ToString());
		}
    }

    Vector3 RosToUnityPositionAxisConversion (Vector3 rosIn)
	{
        return new Vector3(-rosIn.x, rosIn.z, -rosIn.y) * scale;// + robot.transform.position;	
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
