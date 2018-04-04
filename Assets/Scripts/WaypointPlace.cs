using UnityEngine;
using UnityEngine.UI;
using System;

namespace Academy.HoloToolkit.Unity
{
    public class WaypointPlace : MonoBehaviour
    {
        bool placing = true;
        bool waypointPlaced = false;
        int waypointInd;
        GameObject waypointObj;
        GameObject coordTextObj;
        Text coordText;

        private void Start()
        {
            SpatialMapping.Instance.DrawVisualMeshes = true;
            InitializeWaypoint();
            Debug.Log(String.Format("{0} initialized!", waypointObj.name));
        }

        void InitializeWaypoint()
        {
            waypointObj = gameObject;
            waypointInd = WaypointManager.Instance.waypointInd;
            waypointObj.name = String.Format("Waypoint{0}", waypointInd);
            coordTextObj = waypointObj.transform.GetChild(0).gameObject.transform.GetChild(0).gameObject;
            coordTextObj.name = String.Format("WaypointCoord{0}", waypointInd);
            WaypointManager.Instance.AddWaypoint(waypointObj);
            WaypointManager.Instance.waypointInd++;
        }

        // Called by GazeGestureManager when the user performs a Select gesture
        void OnSelect()
        {
            // On each Select gesture, toggle whether the user is in placing mode.
            //placing = !placing;

            //if (firstPlace)
            //{
            //    waypointObj = gameObject;
            //    InitializeWaypoint();
            //    Debug.Assert(coordTextObj != null);
            //}
            if (waypointPlaced)
            {
                waypointObj = Instantiate(gameObject);
                placing = false;
            }
        }

        // Update is called once per frame
        void Update()
        {
            // If the user is in placing mode,
            // update the placement to match the user's gaze.

            if (placing)
            {
                //Debug.Log(String.Format("Placing Waypoint {0}", waypoint_ind));
                
                // Do a raycast into the world that will only hit the Spatial Mapping mesh.
                var headPosition = Camera.main.transform.position;
                var gazeDirection = Camera.main.transform.forward;

                RaycastHit hitInfo;
                if (Physics.Raycast(headPosition, gazeDirection, out hitInfo,
                    30.0f, SpatialMapping.PhysicsRaycastMask))
                {
                    // Move this object's parent object to
                    // where the raycast hit the Spatial Mapping mesh.
                    //this.transform.position = hitInfo.point;
                    waypointObj.transform.position = hitInfo.point;

                    // Rotate this object's parent object to face the user.
                    Quaternion toQuat = Camera.main.transform.localRotation;
                    toQuat.x = 0;
                    toQuat.z = 0;
                    //this.transform.rotation = toQuat;
                    waypointObj.transform.rotation = toQuat;
                }
                //waypointInd = WaypointManager.Instance.waypointInd - 1;
                //waypointObj = GameObject.Find(String.Format("Waypoint{0}", waypointInd));
                //coordTextObj = GameObject.Find(String.Format("WaypointCoord{0}", waypointInd));
                Debug.Assert(coordTextObj != null);
                coordText = coordTextObj.GetComponent<Text>();
                string msg = string.Format("{0}\n({1}, {2})", waypointObj.name, Math.Round(waypointObj.transform.position.x, 1), Math.Round(waypointObj.transform.position.z, 1));
                coordText.text = msg;
                if (!waypointPlaced)
                {
                    waypointPlaced = true;
                }
            }
        }
    }
}