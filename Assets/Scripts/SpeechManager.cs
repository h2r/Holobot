using Academy.HoloToolkit.Unity;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.Windows.Speech;

public class SpeechManager : Singleton<SpeechManager>
{
    float expandAnimationCompletionTime;
    private UniversalWebsocketClient wsc;
    public GameObject root;
    private GameObject[] endPointSpheres;
    private bool sentStop;
    private bool gripperOpen;
    // KeywordRecognizer object.
    KeywordRecognizer keywordRecognizer;

    // Defines which function to call when a keyword is recognized.
    delegate void KeywordAction(PhraseRecognizedEventArgs args);
    Dictionary<string, KeywordAction> keywordCollection;

    private enum GripperCommand
    {
        CLOSE = 0,
        OPEN = 1,
        NONE = 2,
    }

    void Start()
    {
        sentStop = false;

        keywordCollection = new Dictionary<string, KeywordAction>();
        // Add keywords
        keywordCollection.Add("Manipulate", MoveCommand);
        keywordCollection.Add("Rotate", RotateCommand);
        keywordCollection.Add("Start", StartCommand);
        keywordCollection.Add("New", NewSkillCommand);
        keywordCollection.Add("Stop", StopCommand); 
        keywordCollection.Add("Plan", ExecuteCommand);
        keywordCollection.Add("Open", OpenCommand);
        keywordCollection.Add("Close", CloseCommand);
        //keywordCollection.Add("Repeat", RepeatCommand);

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
        wsc.Advertise("ein/" + "right" + "/forth_commands", "std_msgs/String");
        wsc.SendEinMessage("openGripper", "right"); // gripper always starts open
        gripperOpen = true;
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
        GestureManager.Instance.Transition(GestureManager.Instance.ManipulationRecognizer);
    }

    private void RotateCommand(PhraseRecognizedEventArgs args)
    {
        Debug.Log("GOT ROTATE");
        GestureManager.Instance.Transition(GestureManager.Instance.NavigationRecognizer);
    }

    private void StartCommand(PhraseRecognizedEventArgs args)
    {
        Debug.Log("GOT START");
        //GameObject.  Find("TrajectoryVisualizer").SetActive(true);
        if (!GestureManager.Instance.IsRecordingData && GestureManager.Instance.HasCalibratedSphere)
        {
            GestureManager.Instance.IsRecordingData = true;
            GameObject.Find("ControlSphere").GetComponent<Renderer>().material.color = Color.green;
            Vector3 controlSpherePos = GameObject.Find("ControlSphere").transform.position;
            GestureManager.Instance.UnityMotionPlanEndpoints.Add(controlSpherePos);
            if (gripperOpen)
            {
                GestureManager.Instance.GripperCommands.Add((int)GripperCommand.OPEN);
            } else
            {
                GestureManager.Instance.GripperCommands.Add((int)GripperCommand.CLOSE);
            }
        }
    }

    private void NewSkillCommand(PhraseRecognizedEventArgs args)
    {
        Debug.Log("GOT NEW");
        if (GestureManager.Instance.IsRecordingData)
        {
            this.SendEOS(GripperCommand.NONE);
        }
    }

    private void StopCommand(PhraseRecognizedEventArgs args)
    {
        Debug.Log("GOT STOP");
        if (GestureManager.Instance.IsRecordingData)
        {
            GestureManager.Instance.IsRecordingData = false;
            this.SendEOS(GripperCommand.NONE);
            sentStop = true;
            // can no longer touch the control sphere
            GameObject.Find("ControlSphere").GetComponent<Renderer>().material.color = Color.clear;
            GameObject.Find("ControlSphere").layer = 1;
            int endPointCount = GestureManager.Instance.UnityMotionPlanEndpoints.Count();
            endPointSpheres = new GameObject[endPointCount];
            // iterate over endpoints and have them show up, this is where a user can edit for autonomous plan
            for(int i = 0; i < endPointCount; i++ )
            {
                GameObject go = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                go.AddComponent<MeshCollider>();
                go.AddComponent<EndPointAction>();
                go.transform.position = GestureManager.Instance.UnityMotionPlanEndpoints[i];
                Debug.Log("THIS IS SUPPOSED TO BE THE UNITY LOC OF EP: " + GestureManager.Instance.UnityMotionPlanEndpoints[i]);
                go.transform.localScale = new Vector3(0.1f, 0.1f, 0.1f);
                go.layer = 0; // default layer
                endPointSpheres[i] = go;
            }
        }
    }

    private void ExecuteCommand(PhraseRecognizedEventArgs args)
    {
        Debug.Log("GOT EXECUTE");
        if (sentStop)
        {
            string data = "";
            GameObject.Find("ControlSphere").GetComponent<Renderer>().material.color = Color.yellow;
            for(int i=0; i < endPointSpheres.Count(); i++)
            {
                endPointSpheres[i].layer = 1;
                GameObject.Find("ControlSphere").layer = 8;
                Vector3 rosPos = UnityToRosPositionAxisConversion(
                    (endPointSpheres[i].transform.position - GestureManager.Instance.RobotOffset)
                    - (root.transform.position - GestureManager.Instance.RobotOffset)
                    );
                int gripperCommand = GestureManager.Instance.GripperCommands[i];
                data = data + " " + rosPos.x + " " + rosPos.y + " " + rosPos.z + " " + gripperCommand;
                Debug.Log("CURR UNITY LOC: " + endPointSpheres[i].transform.position);
            }
            Debug.Log("EXE " + data);
            wsc.SendLfdMessage("EXE", data);
        }
    }

    private void OpenCommand(PhraseRecognizedEventArgs args)
    {
        if(!gripperOpen)
        {
            wsc.SendEinMessage("openGripper", "right");
            gripperOpen = true;
        }
        if (GestureManager.Instance.IsRecordingData)
        {
            this.SendSupport();
            this.SendEOS(GripperCommand.OPEN);
        }
        
    }

    private void CloseCommand(PhraseRecognizedEventArgs args)
    {
        if(gripperOpen)
        {
            wsc.SendEinMessage("closeGripper", "right");
            gripperOpen = false;
        }
        if (GestureManager.Instance.IsRecordingData)
        {
            this.SendSupport();
            this.SendEOS(GripperCommand.CLOSE);
        }
    }

    //private void RepeatCommand(PhraseRecognizedEventArgs args)
    //{

    //}

    private void SendSupport()
    {
        // clicking on sphere
        // should save a support
        // saves a shadow
   
        Vector3 rosPos = UnityToRosPositionAxisConversion(
            (GameObject.Find("ControlSphere").transform.position - GestureManager.Instance.RobotOffset)
            - (root.transform.position - GestureManager.Instance.RobotOffset)
        );

        string baseMessage = rosPos.x + " " + rosPos.y + " " + rosPos.z;
        // the added 1 bit is to tell the ros server that we are sending a critical point
        wsc.SendLfdMessage("PT", baseMessage + " 1");
    }

    private void SendEOS(GripperCommand cmd)
    {
        Vector3 controlSpherePos = GameObject.Find("ControlSphere").transform.position;
        // add last endpoint
        GestureManager.Instance.UnityMotionPlanEndpoints.Add(controlSpherePos);
        GestureManager.Instance.GripperCommands.Add((int)cmd);
        wsc.SendLfdMessage("EOS", "");
    }

    public void Update()
    {

    }

    Vector3 UnityToRosPositionAxisConversion(Vector3 rosIn)
    {
        return new Vector3(-rosIn.x, -rosIn.z, rosIn.y);
    }
}
