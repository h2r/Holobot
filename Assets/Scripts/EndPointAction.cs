using Academy.HoloToolkit.Unity;
using UnityEngine;

public class EndPointAction : MonoBehaviour
{
    [Tooltip("Audio clip to play when interacting with this hologram.")]
    private Vector3 manipulationPreviousPosition;
    Vector3 currPos;
    public AudioClip TargetFeedbackSound;
    public GameObject root;
    private AudioSource audioSource;

    // Use this for initialization
    void Start()
    {
        Collider collider = GetComponentInChildren<Collider>();
        if (collider == null)
        {
            gameObject.AddComponent<BoxCollider>();
        }

        EnableAudioHapticFeedback();
    }

    private void PerformRotation(){}

    void PerformManipulationStart(Vector3 position)
    {
        manipulationPreviousPosition = position;
    }

    void PerformManipulationUpdate(Vector3 position)
    {
        if (GestureManager.Instance.IsManipulating)
        {
            if (audioSource != null && !audioSource.isPlaying)
            {
                audioSource.Play();
            }
            Vector3 moveVector = Vector3.zero;
            moveVector = position - manipulationPreviousPosition;
            manipulationPreviousPosition = position;
            transform.position += moveVector;
            currPos = transform.position;
        }
    }

    void OnSelect(GameObject selected)
    {
        Vector3 rosPos = UnityToRosPositionAxisConversion(
            (selected.transform.position - GestureManager.Instance.RobotOffset)
            - (root.transform.position - GestureManager.Instance.RobotOffset)
        );
        if (audioSource != null && !audioSource.isPlaying)
        {
            audioSource.Play();
        }
        if (selected == GameObject.Find("StartSphere"))
        {
            GestureManager.Instance.MotionPlanStart = rosPos;
            Debug.Log("StartSphere Purple at: " + rosPos);
        }
        else if (selected == GameObject.Find("StopSphere"))
        {
            GestureManager.Instance.MotionPlanStop = rosPos;
            Debug.Log("StopSphere cyan at: " + rosPos);
        }
    }

    private void EnableAudioHapticFeedback()
    {
        // If this hologram has an audio clip, add an AudioSource with this clip.
        if (TargetFeedbackSound != null)
        {
            audioSource = GetComponent<AudioSource>();
            if (audioSource == null)
            {
                audioSource = gameObject.AddComponent<AudioSource>();
            }

            audioSource.clip = TargetFeedbackSound;
            audioSource.playOnAwake = false;
            audioSource.spatialBlend = 1;
            audioSource.dopplerLevel = 0;
        }
    }

    Vector3 UnityToRosPositionAxisConversion(Vector3 rosIn)
    {
        return new Vector3(-rosIn.x, -rosIn.z, rosIn.y);
    }
}
