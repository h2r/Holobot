using Academy.HoloToolkit.Unity;
using UnityEngine;

/// <summary>
/// The Interactible class flags a Game Object as being "Interactible".
/// Determines what happens when an Interactible is being gazed at.
/// </summary>
public class Interactible : MonoBehaviour {
    [Tooltip("Audio clip to play when interacting with this hologram.")]
    public AudioClip TargetFeedbackSound;
    private AudioSource audioSource;
    private string topic = "ros_unity";
    private Material[] defaultMaterials;
    private UniversalWebsocketClient wsc;

    void Start() {
        // defaultMaterials = GetComponent<Renderer>().materials;
        // Add a BoxCollider if the interactible does not contain one.
        Collider collider = GetComponentInChildren<Collider>();
        if (collider == null) {
            gameObject.AddComponent<BoxCollider>();
        }

        EnableAudioHapticFeedback();
        GameObject wso = GameObject.Find("WebsocketClient");
#if UNITY_EDITOR
        wsc = wso.GetComponent<WebsocketClient>();
#else
        wsc = wso.GetComponent<UWPWebSocketClient>();
#endif
        wsc.Subscribe(topic, "std_msgs/String", "none", 0);
    }

    private void Update()
    {

        // Note: based on TFListener.cs, look at this for reference for updating the whole robot
      
        string message;
        try
        {
            message = wsc.messages[topic];         
        }
        catch
        {
            return;
        }
        string[] tfElements = message.Split(';'); //split the message into each joint/link data pair
                                                    //Debug.Log(string.Join(", ", tfElements));
        foreach (string tfElement in tfElements)
        {
            //Debug.Log(tfElement);
            //continue;
            string[] dataPair = tfElement.Split(':');
            if (dataPair[0] == "right_gripper_base" || dataPair[0] == "torso")
            {
                string[] tmp = dataPair[1].Split('^'); //seperate position from rotation data
                string pos = tmp[0]; //position data
                                        //string rot = tmp[1]; //rotation data
                pos = pos.Substring(1, pos.Length - 2);
                //rot = rot.Substring(1, rot.Length - 2);
                string[] poses = pos.Split(',');
                float ros_x = float.Parse(poses[0]); //x position
                float ros_y = float.Parse(poses[1]); //y position
                float ros_z = float.Parse(poses[2]); //z position
                if (dataPair[0] == "right_gripper_base")
                {
                    GestureManager.Instance.rosInitGripperPos = new Vector3(ros_x, ros_y, ros_z);
                }
                else if (dataPair[0] == "torso")
                {
                    GestureManager.Instance.rosInitTorsoPos = new Vector3(ros_x, ros_y, ros_z);
                }
            }
        }
    }

    private void EnableAudioHapticFeedback() {
        // If this hologram has an audio clip, add an AudioSource with this clip.
        if (TargetFeedbackSound != null) {
            audioSource = GetComponent<AudioSource>();
            if (audioSource == null) {
                audioSource = gameObject.AddComponent<AudioSource>();
            }

            audioSource.clip = TargetFeedbackSound;
            audioSource.playOnAwake = false;
            audioSource.spatialBlend = 1;
            audioSource.dopplerLevel = 0;
        }
    }

    /* TODO: DEVELOPER CODING EXERCISE 2.d */

    void GazeEntered() {
        //for (int i = 0; i < defaultMaterials.Length; i++) {
            // 2.d: Uncomment the below line to highlight the material when gaze enters.
        //    defaultMaterials[i].SetFloat("_Highlight", .25f);
       // }
    }

    void GazeExited() {
        //for (int i = 0; i < defaultMaterials.Length; i++) {
            // 2.d: Uncomment the below line to remove highlight on material when gaze exits.
         //   defaultMaterials[i].SetFloat("_Highlight", 0f);
        //}
    }

    void OnSelect(GameObject selected) {
        //for (int i = 0; i < defaultMaterials.Length; i++) {
        //    defaultMaterials[i].SetFloat("_Highlight", .5f);
        //}
        if (selected == null)
        {
            return;
        }
        // Play the audioSource feedback when we gaze and select a hologram.
        if (audioSource != null && !audioSource.isPlaying) {
            audioSource.Play();
        }
        //if (GestureManager.Instance.IsRecordingData)
        //{
        //    // create a new object at the place where the moving object was selected
        //    // these will also serve as undo points
        //    GestureManager.Instance.RecordUndoPoint();
        //}

        if (GestureManager.Instance.HasCalibratedSphere)
        {
            // clicking on sphere
            // saves a shadow
            GameObject shadow = Instantiate(selected, selected.transform.position, selected.transform.rotation);
            shadow.layer = 1;

        } else // case where we are calibrating, want to set the ofset and switch things to 
        {
            wsc.Unsubscribe(topic); // unsubscibe and grab latest values from the globals
            Debug.Log("init gripper: " + GestureManager.Instance.rosInitGripperPos);
            Debug.Log("init torso: " + GestureManager.Instance.rosInitTorsoPos);

            GestureManager.Instance.CalibrationOffset = selected.transform.position 
                - RosToUnityPositionAxisConversion(GestureManager.Instance.rosInitGripperPos);
            GestureManager.Instance.HasCalibratedSphere = true;
            selected.GetComponent<Renderer>().material.color = new Color(1, 1, 1, 0.2f);
        }
    }

    Vector3 RosToUnityPositionAxisConversion(Vector3 rosIn)
    {
        // Debug.Log(GestureManager.Instance.RobotOffset);
        return new Vector3(-rosIn.x, rosIn.z, -rosIn.y);// * scale;// + GestureManager.Instance.RobotOffset;
    }
}