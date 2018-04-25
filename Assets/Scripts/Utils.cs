using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Academy.HoloToolkit.Unity {
    public class Utils : MonoBehaviour {
        public static void RaycastPlace(Camera cam, GameObject obj) {
            Debug.Assert(SpatialMapping.Instance.DrawVisualMeshes == true);
            // Do a raycast into the world that will only hit the Spatial Mapping mesh.
            var headPosition = cam.transform.position;
            var gazeDirection = cam.transform.forward;

            RaycastHit hitInfo;
            if (Physics.Raycast(headPosition, gazeDirection, out hitInfo,
                30.0f, SpatialMapping.PhysicsRaycastMask)) {
                // Move this object's parent object to
                // where the raycast hit the Spatial Mapping mesh.
                //this.transform.position = hitInfo.point;
                obj.transform.position = hitInfo.point;

                // Rotate this object's parent object to face the user.
                Quaternion toQuat = cam.transform.localRotation;
                toQuat.x = 0;
                toQuat.z = 0;
                //this.transform.rotation = toQuat;
                obj.transform.rotation = toQuat;
            }
        }
        public static void SetSpatialMapping(bool state) {
            SpatialMapping.Instance.DrawVisualMeshes = state;
        }
    }
}