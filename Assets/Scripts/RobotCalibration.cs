using Academy.HoloToolkit.Unity;
using System;
using UnityEngine;

/// <summary>
/// GestureAction performs custom actions based on 
/// which gesture is being performed.
/// </summary>

public class RobotCalibration : MonoBehaviour
{
    [Tooltip("Rotation max speed controls amount of rotation.")]
    public float RotationSensitivity = 10.0f;

    private Vector3 manipulationPreviousPosition;
    private Vector3 startPos;
    private float rotationFactor;


    private void Awake()
    {

    }

    void Update()
    {
        // PerformRotation();
    }

    private void PerformRotation()
    {
        if (GestureManager.Instance.IsNavigating) //&& !GestureManager.Instance.RobotCalibrating)
        {
            // This will help control the amount of rotation.
            rotationFactor = GestureManager.Instance.NavigationPosition.x * RotationSensitivity;

            // transform.Rotate along the Y axis using rotationFactor.
            transform.Rotate(new Vector3(0, -1 * rotationFactor, 0));

        }
    }

    void PerformManipulationStart(Vector3 position)
    {
        if (GestureManager.Instance.IsManipulating)
        {
            manipulationPreviousPosition = position;
        }
    }

    void PerformManipulationUpdate(Vector3 position)
    {
        if (GestureManager.Instance.IsManipulating)
        {
            /* TODO: DEVELOPER CODING EXERCISE 4.a */

            Vector3 moveVector = Vector3.zero;

            // 4.a: Calculate the moveVector as position - manipulationPreviousPosition.
            moveVector = position - manipulationPreviousPosition;

            // 4.a: Update the manipulationPreviousPosition with the current position.
            manipulationPreviousPosition = position;

            // 4.a: Increment this transform's position by the moveVector.
            //startPos = transform.position;
            transform.position += moveVector; // * 3
        }
    }
}
