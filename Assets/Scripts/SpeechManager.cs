﻿using Academy.HoloToolkit.Unity;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.Windows.Speech;

public class SpeechManager : Singleton<SpeechManager>
{
    float expandAnimationCompletionTime;
    private UniversalWebsocketClient wsc;
    private Vector3 startUnityPos;
    public GameObject root;

    // KeywordRecognizer object.
    KeywordRecognizer keywordRecognizer;

    // Defines which function to call when a keyword is recognized.
    delegate void KeywordAction(PhraseRecognizedEventArgs args);
    Dictionary<string, KeywordAction> keywordCollection;

    void Start()
    {
        keywordCollection = new Dictionary<string, KeywordAction>();
        startUnityPos = Vector3.zero;
        // Add keywords
        keywordCollection.Add("Manipulate", MoveCommand);
        keywordCollection.Add("Rotate", RotateCommand);
        keywordCollection.Add("Start", StartCommand);
        keywordCollection.Add("Stop", StopCommand); 
        keywordCollection.Add("Plan", ExecuteCommand);
        keywordCollection.Add("Go back", UndoCommand); 
        // Initialize KeywordRecognizer with the previously added keywords.
        keywordRecognizer = new KeywordRecognizer(keywordCollection.Keys.ToArray());
        keywordRecognizer.OnPhraseRecognized += KeywordRecognizer_OnPhraseRecognized;
        keywordRecognizer.Start();
        GameObject wso = GameObject.Find("WebsocketClient");
        #if !UNITY_EDITOR
        wsc = wso.GetComponent<UWPWebSocketClient>();
        #else
        wsc = wso.GetComponent<WebsocketClient>();
        #endif
        wsc.Advertise("dmp_train_data", "std_msgs/String");
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
            GameObject.Find("ControlSphere").GetComponent<Renderer>().material.color = Color.green;
            startUnityPos = GameObject.Find("ControlSphere").transform.position;
            GestureManager.Instance.MotionPlanStart = UnityToRosPositionAxisConversion(
                (startUnityPos - GestureManager.Instance.CalibrationOffset)
                - GestureManager.Instance.rosInitTorsoPos
            );
        }
    }

    private void StopCommand(PhraseRecognizedEventArgs args)
    {
        Debug.Log("GOT STOP");
        if (GestureManager.Instance.IsRecordingData)
        {
            GestureManager.Instance.IsRecordingData = false;
            
            #if UNITY_EDITOR
            GestureManager.Instance.WritePathData();
            #endif

            GameObject.Find("ControlSphere").GetComponent<Renderer>().material.color = Color.red;
            wsc.SendDemonstrationData("EOF");

            GameObject startSphere = GameObject.Find("StartSphere");
            startSphere.transform.position = startUnityPos;
            startSphere.GetComponent<Renderer>().material.color = Color.magenta;
            startSphere.layer = 0; // can now interact with this sphere
            
            Vector3 tempOffset = new Vector3(0.03f, 0.03f, 0.03f);
            GameObject stopSphere = GameObject.Find("StopSphere");
            stopSphere.transform.position = GameObject.Find("ControlSphere").transform.position + tempOffset;
            stopSphere.GetComponent<Renderer>().material.color = Color.cyan;
            stopSphere.layer = 0;
            GestureManager.Instance.MotionPlanStop = UnityToRosPositionAxisConversion(
                (stopSphere.transform.position - GestureManager.Instance.CalibrationOffset)
                - GestureManager.Instance.rosInitTorsoPos
            );
        }
    }

    public void ExecuteCommand(PhraseRecognizedEventArgs args)
    {
        Debug.Log("GOT EXECUTE");
        if (!GestureManager.Instance.IsRecordingData && 
            GameObject.Find("ControlSphere").GetComponent<Renderer>().material.color == Color.red)
        {
            GameObject.Find("ControlSphere").GetComponent<Renderer>().material.color = Color.yellow;
            wsc.SendExecuteMotionPlan(GestureManager.Instance.MotionPlanStart, GestureManager.Instance.MotionPlanStop);
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

    Vector3 UnityToRosPositionAxisConversion(Vector3 rosIn)
    {
        return new Vector3(-rosIn.x, -rosIn.z, rosIn.y);
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