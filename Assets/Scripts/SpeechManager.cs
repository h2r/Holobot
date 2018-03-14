using Academy.HoloToolkit.Unity;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.Windows.Speech;

public class SpeechManager : Singleton<SpeechManager>
{
    float expandAnimationCompletionTime;

    // KeywordRecognizer object.
    KeywordRecognizer keywordRecognizer;

    // Defines which function to call when a keyword is recognized.
    delegate void KeywordAction(PhraseRecognizedEventArgs args);
    Dictionary<string, KeywordAction> keywordCollection;

    void Start()
    {
        keywordCollection = new Dictionary<string, KeywordAction>();
        // Add keywords
        keywordCollection.Add("Manipulate", MoveCommand);
        keywordCollection.Add("Rotate", RotateCommand);
        keywordCollection.Add("Start", StartCommand);
        keywordCollection.Add("Stop", StopCommand);
        keywordCollection.Add("Go back", UndoCommand);
        // Initialize KeywordRecognizer with the previously added keywords.
        keywordRecognizer = new KeywordRecognizer(keywordCollection.Keys.ToArray());
        keywordRecognizer.OnPhraseRecognized += KeywordRecognizer_OnPhraseRecognized;
        keywordRecognizer.Start();
    }

    void OnDestroy()
    {
        keywordRecognizer.Dispose();

    }

    private void KeywordRecognizer_OnPhraseRecognized(PhraseRecognizedEventArgs args)
    {
        KeywordAction keywordAction;

        if (keywordCollection.TryGetValue(args.text, out keywordAction))
        {
            keywordAction.Invoke(args);
        }
    }

    private void MoveCommand(PhraseRecognizedEventArgs args)
    {
        Debug.Log("GOT MANIPULATE");
        //Transform cal = GameObject.Find("ControlSphere").transform;
        //Transform screen = GameObject.Find("screen").transform;
        GestureManager.Instance.Transition(GestureManager.Instance.ManipulationRecognizer);
    }

    private void RotateCommand(PhraseRecognizedEventArgs args)
    {
        Debug.Log("GOT ROTATE");
        //Transform cal = GameObject.Find("calibrater").transform;
        //Transform screen = GameObject.Find("screen").transform;
        GestureManager.Instance.Transition(GestureManager.Instance.NavigationRecognizer);
    }

    private void StartCommand(PhraseRecognizedEventArgs args)
    {
        Debug.Log("GOT START");
        //GameObject.  Find("TrajectoryVisualizer").SetActive(true);
        if (!GestureManager.Instance.IsRecordingData)
        {
            GestureManager.Instance.IsRecordingData = true;
        }
    }

    private void StopCommand(PhraseRecognizedEventArgs args)
    {
        Debug.Log("GOT STOP");
        if (GestureManager.Instance.IsRecordingData)
        {
            GestureManager.Instance.IsRecordingData = false;
            GestureManager.Instance.WritePathData();
        }
    }

    private void UndoCommand(PhraseRecognizedEventArgs args)
    {
        // TODO go back to last undo point
        Debug.Log("GOT UNDO");
        if (GestureManager.Instance.IsRecordingData)
        {
            GestureManager.Instance.UndoAction();
        }
    }

    public void Update()
    {
        //if (isModelExpanding && Time.realtimeSinceStartup >= expandAnimationCompletionTime)
        //{
        //    isModelExpanding = false;

        //    Animator[] expandedAnimators = ExpandModel.Instance.ExpandedModel.GetComponentsInChildren<Animator>();

        //    foreach (Animator animator in expandedAnimators)
        //    {
        //        animator.enabled = false;
        //    }
        //}
    }
}

// above is the new code for motion control
// below is the original holobot code

//using System.Collections.Generic;
//using System.Linq;
//using UnityEngine;
//using UnityEngine.Windows.Speech;

//public class SpeechManager : MonoBehaviour {
//    KeywordRecognizer keywordRecognizer = null;
//    Dictionary<string, System.Action> keywords = new Dictionary<string, System.Action>();
//    public GameObject tflistener;
//    //GameObject basePivot;
//    // Use this for initialization
//    void Start() {
//        //basePivot = GameObject.Find("basePivot");
//        keywords.Add("world", () => {
//            Transform cal = GameObject.Find("calibrater").transform;
//            Transform screen = GameObject.Find("screen").transform;
//            // Call the OnReset method on every descendant object.

//            Debug.Log("rotation");
//        });
//        keywords.Add("position", () => {
//            Transform cal = GameObject.Find("ControlSphere").transform;
//            Transform screen = GameObject.Find("screen").transform;
//            // Call the OnReset method on every descendant object.

//            Debug.Log("position");
//        });
//        keywords.Add("ball", () => {
//            Debug.Log("ball");
//            GameObject.Find("ControlSphere").transform.position = GameObject.Find("right_wrist").transform.position;
//            GameObject.  Find("TrajectoryVisualizer").SetActive(true);

//        });

//        // Tell the KeywordRecognizer about our keywords.
//        keywordRecognizer = new KeywordRecognizer(keywords.Keys.ToArray());

//        // Register a callback for the KeywordRecognizer and start recognizing!
//        keywordRecognizer.OnPhraseRecognized += KeywordRecognizer_OnPhraseRecognized;
//        keywordRecognizer.Start();
//    }

//    private void KeywordRecognizer_OnPhraseRecognized(PhraseRecognizedEventArgs args) {
//        System.Action keywordAction;
//        if (keywords.TryGetValue(args.text, out keywordAction)) {
//            keywordAction.Invoke();
//        }
//    }
//}