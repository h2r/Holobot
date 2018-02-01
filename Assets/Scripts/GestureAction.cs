using Academy.HoloToolkit.Unity;
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
    Vector3 startPos;
    Vector3 currPos;
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
    }

    void Update() {
        PerformRotation();
    }

    private void PerformRotation() {
        //if (GestureManager.Instance.IsNavigating &&
        //    (!ExpandModel.Instance.IsModelExpanded ||
        //    (ExpandModel.Instance.IsModelExpanded && HandsManager.Instance.FocusedGameObject == gameObject))) {
        //    /* TODO: DEVELOPER CODING EXERCISE 2.c */

        //    // 2.c: Calculate rotationFactor based on GestureManager's NavigationPosition.X and multiply by RotationSensitivity.
        //    // This will help control the amount of rotation.
        //    rotationFactor = GestureManager.Instance.NavigationPosition.x * RotationSensitivity;

        //    // 2.c: transform.Rotate along the Y axis using rotationFactor.
        //    transform.Rotate(new Vector3(0, -1 * rotationFactor, 0));
        //}
        //moveArm();
        moveArm2();
    }

    void PerformManipulationStart(Vector3 position) {
        Debug.Log("GestureAction PerformManipulationStart");
        manipulationPreviousPosition = position;
        startPos = position;
    }

    void PerformManipulationUpdate(Vector3 position) {
        if (GestureManager.Instance.IsManipulating) {
            /* TODO: DEVELOPER CODING EXERCISE 4.a */

            Vector3 moveVector = Vector3.zero;

            // 4.a: Calculate the moveVector as position - manipulationPreviousPosition.
            moveVector = position - manipulationPreviousPosition;

            // 4.a: Update the manipulationPreviousPosition with the current position.
            manipulationPreviousPosition = position;

            // 4.a: Increment this transform's position by the moveVector.
            //startPos = transform.position;
            transform.position += moveVector * 3;
            currPos = transform.position;
            //Debug.Log(transform.position);
            moveArm2();
        }
    }

    void moveArm2() {
        Vector3 rosPos = UnityToRosPositionAxisConversion(manipulator.transform.position - root.transform.position);
        string message = rosPos.x + " " + rosPos.y + " " + rosPos.z + " 0 1 0 0 moveToEEPose ";
        //Debug.Log(message);
        //wsc.SendEinMessage(message, "right");
    }

    void moveArm(float thresh = 0.1f) {
        //Debug.Log("moving arm");
        if (Mathf.Abs(currPos.x - startPos.x) > thresh) {
            if (currPos.x < startPos.x) {
                wsc.SendEinMessage(" yUp yUp yUp yUp yUp yUp yUp yUp ", "right");

                //Debug.Log("yUp");
            }
            else {
                wsc.SendEinMessage(" yDown yDown yDown yDown yDown yDown yDown yDown ", "right");
                //Debug.Log("yDown");
            }
            startPos = currPos;
        }
        else if (Mathf.Abs(currPos.y - startPos.y) > thresh) {
            if (currPos.y > startPos.y) {
                wsc.SendEinMessage(" zUp zUp zUp zUp zUp zUp zUp zUp ", "right");
                //Debug.Log("zUp");
            }
            else {
                wsc.SendEinMessage(" zDown zDown zDown zDown zDown zDown zDown zDown ", "right");
                //Debug.Log("zDown");
            }
            startPos = currPos;
        }
        else if (Mathf.Abs(currPos.z - startPos.z) > thresh) {
            if (currPos.z > startPos.z) {
                wsc.SendEinMessage(" xUp xUp xUp xUp xUp xUp xUp xUp ", "right");
                //Debug.Log("xUp");
            }
            else {
                wsc.SendEinMessage(" xDown xDown xDown xDown xDown xDown xDown xDown ", "right");
                //Debug.Log("xDown");
            }
            startPos = currPos;
        }
    }

    Vector3 UnityToRosPositionAxisConversion(Vector3 rosIn) {
        return new Vector3(-rosIn.x, -rosIn.z, rosIn.y);
    }
}