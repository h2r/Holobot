using UnityEngine;
using UnityEngine.UI;
using System;

public class SpatialPlace : MonoBehaviour
{
    bool placing = false;
    bool firstPlace = true;
    int waypoint_ind = 0;

    // Called by GazeGestureManager when the user performs a Select gesture
    void OnSelect()
    {
        // On each Select gesture, toggle whether the user is in placing mode.
        placing = !placing;

        // If the user is in placing mode, display the spatial mapping mesh.
        if (placing)
        {
            SpatialMapping.Instance.DrawVisualMeshes = true;
        }
        // If the user is not in placing mode, hide the spatial mapping mesh.
        else
        {
            SpatialMapping.Instance.DrawVisualMeshes = false;
        }
        if (!placing && !firstPlace)
        {
            waypoint_ind++;
            GameObject newWaypoint = Instantiate(gameObject);
            newWaypoint.name = String.Format("Waypoint{0}", waypoint_ind);
            // TODO: change child text name
            Debug.Log(String.Format("New Waypoint: {0}", newWaypoint.name));
            placing = true;
            SpatialMapping.Instance.DrawVisualMeshes = true;
        }
    }

    // Update is called once per frame
    void Update()
    {
        // If the user is in placing mode,
        // update the placement to match the user's gaze.

        if (placing)
        {
            if (firstPlace)
            {
                firstPlace = false;
            }
            // Do a raycast into the world that will only hit the Spatial Mapping mesh.
            var headPosition = Camera.main.transform.position;
            var gazeDirection = Camera.main.transform.forward;

            RaycastHit hitInfo;
            if (Physics.Raycast(headPosition, gazeDirection, out hitInfo,
                30.0f, SpatialMapping.PhysicsRaycastMask))
            {
                // Move this object's parent object to
                // where the raycast hit the Spatial Mapping mesh.
                this.transform.position = hitInfo.point;

                // Rotate this object's parent object to face the user.
                Quaternion toQuat = Camera.main.transform.localRotation;
                toQuat.x = 0;
                toQuat.z = 0;
                this.transform.rotation = toQuat;
            }
            GameObject coord_text_obj = GameObject.Find(String.Format("WaypointCoord{0}", waypoint_ind));
            Text coord_text = coord_text_obj.GetComponent<Text>();
            string msg = string.Format("{0}\n({1}, {2})", gameObject.name, Math.Round(gameObject.transform.position.x, 1), Math.Round(gameObject.transform.position.z, 1));
            coord_text.text = msg;
        }
    }
}