using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.Windows.Speech;

public class SpeechManager : MonoBehaviour {
    KeywordRecognizer keywordRecognizer = null;
    Dictionary<string, System.Action> keywords = new Dictionary<string, System.Action>();
    public GameObject tflistener;
    //GameObject basePivot;
    // Use this for initialization
    void Start() {
        //basePivot = GameObject.Find("basePivot");
        keywords.Add("world", () => {
            Transform cal = GameObject.Find("calibrater").transform;
            Transform screen = GameObject.Find("screen").transform;
            // Call the OnReset method on every descendant object.

            Debug.Log("rotation");
        });
        keywords.Add("position", () => {
            Transform cal = GameObject.Find("ControlSphere").transform;
            Transform screen = GameObject.Find("screen").transform;
            // Call the OnReset method on every descendant object.

            Debug.Log("position");
        });
        keywords.Add("ball", () => {
            Debug.Log("ball");
            GameObject.Find("ControlSphere").transform.position = GameObject.Find("right_wrist").transform.position;
            GameObject.  Find("TrajectoryVisualizer").SetActive(true);

        });

        // Tell the KeywordRecognizer about our keywords.
        keywordRecognizer = new KeywordRecognizer(keywords.Keys.ToArray());

        // Register a callback for the KeywordRecognizer and start recognizing!
        keywordRecognizer.OnPhraseRecognized += KeywordRecognizer_OnPhraseRecognized;
        keywordRecognizer.Start();
    }

    private void KeywordRecognizer_OnPhraseRecognized(PhraseRecognizedEventArgs args) {
        System.Action keywordAction;
        if (keywords.TryGetValue(args.text, out keywordAction)) {
            keywordAction.Invoke();
        }
    }
}