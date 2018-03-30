using Academy.HoloToolkit.Unity;
using UnityEngine;

public class EndPointAction : MonoBehaviour
{
    [Tooltip("Audio clip to play when interacting with this hologram.")]
    private Vector3 manipulationPreviousPosition;
    //Vector3 currPos;
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
        if (audioSource != null && !audioSource.isPlaying)
        {
            audioSource.Play();
        }
        manipulationPreviousPosition = position;
    }

    void PerformManipulationUpdate(Vector3 position)
    {
        if (GestureManager.Instance.IsManipulating)
        {            
            Vector3 moveVector = Vector3.zero;
            moveVector = position - manipulationPreviousPosition;
            manipulationPreviousPosition = position;
            transform.position += moveVector;
        }
    }

    void OnSelect(GameObject selected)
    {
        if (audioSource != null && !audioSource.isPlaying)
        {
            audioSource.Play();
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
}
