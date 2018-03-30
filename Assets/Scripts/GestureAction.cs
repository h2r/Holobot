using Academy.HoloToolkit.Unity;
using System.Collections;
using System;
using UnityEngine;

/// <summary>
/// GestureAction performs custom actions based on
/// which gesture is being performed.
/// </summary>
public class GestureAction : MonoBehaviour {
    [Tooltip("Rotation max speed controls amount of rotation.")]
    public float RotationSensitivity = 10.0f;
    private Vector3 manipulationPreviousPosition;
    private float rotationFactor;
    private UniversalWebsocketClient wsc;
    //Vector3 startPos;
    //Vector3 currPos;
    public GameObject manipulator;
    public GameObject root;

    void Start() {
        GameObject wso = GameObject.Find("WebsocketClient");
#if !UNITY_EDITOR
        wsc = wso.GetComponent<UWPWebSocketClient>();
#else
        wsc = wso.GetComponent<WebsocketClient>();
#endif
        wsc.Advertise("ein/" + "right" + "/forth_commands", "std_msgs/String");
        wsc.Advertise("dmp_train_data", "std_msgs/String");
    }

    void Update() {

    } 

    private void PerformRotation() {
        if (GestureManager.Instance.IsNavigating)
        {
            rotationFactor = GestureManager.Instance.NavigationPosition.x * RotationSensitivity;

            // 2.c: transform.Rotate along the Y axis using rotationFactor.
            transform.Rotate(new Vector3(0, -1 * rotationFactor, 0));
        }
        rotateArm();
      
    }

    void PerformManipulationStart(Vector3 position) {
        Debug.Log("GestureAction PerformManipulationStart");
        manipulationPreviousPosition = position;
        //startPos = position;
    }

    void PerformManipulationUpdate(Vector3 position) {
        if (GestureManager.Instance.IsManipulating) {
            Vector3 moveVector = Vector3.zero;
            moveVector = position - manipulationPreviousPosition;
            manipulationPreviousPosition = position;
            transform.position += moveVector; // * 3
            translateArm(); // note this method also records data if we are in recording mode
        }
    }

    void translateArm() {
        Vector3 rosPos = UnityToRosPositionAxisConversion((manipulator.transform.position 
            - GestureManager.Instance.RobotOffset) 
            - (root.transform.position 
            - GestureManager.Instance.RobotOffset)
        );

        string baseMessage = rosPos.x + " " + rosPos.y + " " + rosPos.z;
        string einMessage = baseMessage + " 0 1 0 0 moveToEEPose ";
        if (GestureManager.Instance.IsRecordingData) // for local recording, take out eventually
        {
            // the added 0 bit is to tell the ros server that we are not sending a critical point
            wsc.SendLfdMessage("PT", baseMessage + " 0"); // sends ros position data
        }
        // Debug.Log(String.Format("Cmd: ({0}, {1}, {2})", rosPos.x, rosPos.y, rosPos.z));
        wsc.SendEinMessage(einMessage, "right");
    }

    void rotateArm()
    {
        // TODO: allow the user to rotate the EE
    }

    Vector3 UnityToRosPositionAxisConversion(Vector3 rosIn) {
        return new Vector3(-rosIn.x, -rosIn.z, rosIn.y);
    }
}