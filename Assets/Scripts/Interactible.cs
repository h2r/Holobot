using Academy.HoloToolkit.Unity;
using UnityEngine;

/// <summary>
/// The Interactible class flags a Game Object as being "Interactible".
/// Determines what happens when an Interactible is being gazed at.
/// </summary>
public class Interactible : MonoBehaviour {
    [Tooltip("Audio clip to play when interacting with this hologram.")]
    public AudioClip TargetFeedbackSound;
    private bool hasCalibratedSphere, hasCalibratedRobot;
    private AudioSource audioSource;

    private Material[] defaultMaterials;
    private bool firstTime;

    void Start() {
        // defaultMaterials = GetComponent<Renderer>().materials;
        firstTime = true;
        // Add a BoxCollider if the interactible does not contain one.
        Collider collider = GetComponentInChildren<Collider>();
        if (collider == null) {
            gameObject.AddComponent<BoxCollider>();
        }

        EnableAudioHapticFeedback();
        hasCalibratedSphere = false;
        hasCalibratedRobot = false;
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

    //void OnSelect(GameObject selected) {
    //    //for (int i = 0; i < defaultMaterials.Length; i++) {
    //    //    defaultMaterials[i].SetFloat("_Highlight", .5f);
    //    //}

    //    // Play the audioSource feedback when we gaze and select a hologram.
    //    if (audioSource != null && !audioSource.isPlaying) {
    //        audioSource.Play();
    //    }
    //    //if (GestureManager.Instance.IsRecordingData)
    //    //{
    //    //    // create a new object at the place where the moving object was selected
    //    //    // these will also serve as undo points
    //    //    GestureManager.Instance.RecordUndoPoint();
    //    //}
    //    Debug.Log(selected);
    //    if (hasCalibratedSphere && selected == GameObject.Find("ControlSphere"))
    //    {
    //        // clicking on sphere
    //        // toggle gripper

    //    }
    //    if (selected == GameObject.Find("base_link"))
    //    {
    //        if (GestureManager.Instance.CalibratingRobot) {
    //            GestureManager.Instance.RobotOffset = 
    //                GameObject.Find("Robot_very_low_poly").transform.position 
    //                - GestureManager.Instance.RobotStartPos;
    //            Debug.Log("Robot offset is:");
    //            Debug.Log(GestureManager.Instance.RobotOffset);

    //            hasCalibratedSphere = true;
    //            GameObject sphere = GameObject.Find("ControlSphere");
    //            Vector3 init_pos = sphere.transform.position;
    //            sphere.transform.position = GameObject.Find("right_gripper_base").transform.position;
    //            GestureManager.Instance.SphereOffset = sphere.transform.position - init_pos;

    //        } else if (firstTime)
    //        {
    //            firstTime = false;
    //            GestureManager.Instance.RobotStartPos = GameObject.Find("Robot_very_low_poly").transform.position;
    //        }
    //        GestureManager.Instance.CalibratingRobot = !GestureManager.Instance.CalibratingRobot; // toggle
    //    }

    //}
}