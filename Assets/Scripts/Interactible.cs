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
    private UniversalWebsocketClient wsc;
    private Material[] defaultMaterials;
    private bool firstTime;

    void Start() {
        // defaultMaterials = GetComponent<Renderer>().materials;
        GameObject wso = GameObject.Find("WebsocketClient");
#if !UNITY_EDITOR
        wsc = wso.GetComponent<UWPWebSocketClient>();
#else
        wsc = wso.GetComponent<WebsocketClient>();
#endif
        // GameObject.Find("ControlSphere").transform.position = GameObject.Find("right_wrist").transform.position; // moves the control sphere
        wsc.Advertise("dmp_train_data", "std_msgs/String");
        firstTime = true;
        // Add a BoxCollider if the interactible does not contain one.
        Collider collider = GetComponentInChildren<Collider>();
        if (collider == null) {
            gameObject.AddComponent<BoxCollider>();
        }

        EnableAudioHapticFeedback();
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

        // Play the audioSource feedback when we gaze and select a hologram.
        if (audioSource != null && !audioSource.isPlaying) {
            audioSource.Play();
        }

        if (GestureManager.Instance.HasCalibratedSphere && selected == GameObject.Find("ControlSphere")
            && GestureManager.Instance.IsRecordingData)
        {
            // clicking on sphere
            // should save a support
            // saves a shadow
            GameObject rob = GameObject.Find("Robot_very_low_poly");
            GameObject shadow = Instantiate(rob, rob.transform.position, rob.transform.rotation);
            shadow.layer = 1;
            Vector3 rosPos = UnityToRosPositionAxisConversion((selected.transform.position
            - GestureManager.Instance.RobotOffset)
            - (rob.transform.position
            - GestureManager.Instance.RobotOffset));

            string baseMessage = rosPos.x + " " + rosPos.y + " " + rosPos.z;
            // the added 1 bit is to tell the ros server that we are sending a critical point
            wsc.SendLfdMessage("PT", baseMessage + " 1");
        }
        if (selected == GameObject.Find("base_link"))
        {
            if (GestureManager.Instance.RobotCalibrating) {
                GestureManager.Instance.RobotOffset = 
                    GameObject.Find("Robot_very_low_poly").transform.position 
                    - GestureManager.Instance.RobotStart;
                GestureManager.Instance.HasCalibratedSphere = true;
                GameObject sphere = GameObject.Find("ControlSphere");
                Color color = Color.cyan;
                color.a = 0.3f;
                GameObject.Find("ControlSphere").GetComponent<Renderer>().material.color = color;
                Vector3 init_pos = sphere.transform.position;
                sphere.transform.position = GameObject.Find("r_gripper_l_fingerPivot").transform.position;
            } else if (firstTime)
            {
                firstTime = false;
                GestureManager.Instance.RobotStart = GameObject.Find("Robot_very_low_poly").transform.position;
            }
            GestureManager.Instance.RobotCalibrating = !GestureManager.Instance.RobotCalibrating; // toggle
        }

    }

    Vector3 UnityToRosPositionAxisConversion(Vector3 rosIn)
    {
        return new Vector3(-rosIn.x, -rosIn.z, rosIn.y);
    }
}