﻿using RosSharp.RosBridgeClient;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

namespace HoloToolkit.Unity {
    public class EinRightForthCommandsPublisher : Publisher {

        public GameObject leftController;
        public GameObject rightController;
        public GameObject Robot;

        [HideInInspector]
        public MoveItGoalPublisher MoveItGoalPublisher;

        private Vector3 outLeftPos, outRightPos;
        private Quaternion outLeftQuat, outRightQuat;

        private int moveitIdentityPoseRequestId;

        protected override void Start() {
            rosSocket = GetComponent<RosConnector>().RosSocket;

            publicationId = rosSocket.Advertise(Topic, "std_msgs/String");
            moveitIdentityPoseRequestId = rosSocket.Advertise("/holocontrol/identity_pose_request", "std_msgs/String");
            SendCommand("baseGoCfg");
            InvokeRepeating("SendEinCommands", .1f, .1f);
            MoveItGoalPublisher = GameObject.Find("RosConnector").GetComponent<MoveItGoalPublisher>();
        }

        float NormalizeRadRotation(float theta) {
            float TwoPI = (float)(2 * Math.PI);
            while (theta < -Math.PI) {
                theta += TwoPI;
            }
            while (theta > Math.PI) {
                theta -= TwoPI;
            }
            return theta;
        }

        void LookAtUser() {
            Vector3 relativePos = Camera.main.transform.position - GameObject.Find("pan_base_link (fixed Joint: pan_base_joint)").transform.position;
            Quaternion rotation = Quaternion.LookRotation(relativePos);
            Vector3 eulerRotation = rotation.eulerAngles;
            float movoYRot = StateManager.Instance.MovoBaseLink.transform.localRotation.eulerAngles.y;
            float panVal = NormalizeRadRotation(UtilFunctions.Deg2Rad(eulerRotation.y - movoYRot)); // TODO: fix
            float tiltVal = -NormalizeRadRotation(UtilFunctions.Deg2Rad(eulerRotation.x));
            string panTiltCmd = "";
            if (panVal > -1.568 && panVal < 1.568) {
                panTiltCmd += panVal.ToString() + " setTargetPanPos \n ";
                if (tiltVal > -1.568 && tiltVal < 1.568) {
                    panTiltCmd += tiltVal.ToString() + " setTargetTiltPos \n ";
                    SendCommand(panTiltCmd);
                }
            }
        }

        void SendEinCommands() {
            List<string> cmds = StateManager.Instance.EinCommandsToExecute;
            while (cmds.Count > 0) {
                SendCommand(cmds[0]);
                cmds.RemoveAt(0);
            }
            if (StateManager.Instance.LookAtUser) {
                LookAtUser();
            }
            var currState = StateManager.Instance.CurrentState;
            if (currState != StateManager.State.PuppetState && currState != StateManager.State.MotionIntentState) {
                return;
            }
            if (currState == StateManager.State.PuppetState) {
                GameObject.Find("RosConnector").GetComponent<MoveItGoalPublisher>().PublishMove();
            }
        }

        private void UpdateEndEffectorPoses() {
            outLeftPos = UtilFunctions.UnityToRosPositionAxisConversion(leftController.transform.localPosition);
            outRightPos = UtilFunctions.UnityToRosPositionAxisConversion(rightController.transform.localPosition);
            outLeftQuat = UtilFunctions.UnityToRosRotationAxisConversion(leftController.transform.localRotation);
            outRightQuat = UtilFunctions.UnityToRosRotationAxisConversion(rightController.transform.localRotation);
        }

        private void SendCommand(string command) {
            StandardString msg = new StandardString {
                data = command
            };
            rosSocket.Publish(publicationId, msg);
            Debug.Log("Sent command: " + msg.data);
        }

        //private GeometryPoseStamped PoseToPoseStamped(GeometryPose pose, string frame_id) {
        //    return new GeometryPoseStamped {
        //        pose = pose,
        //        header = new StandardHeader {
        //            frame_id = frame_id
        //        }
        //    };
        //}

        public void SendPlanRequest() {
            var currState = StateManager.Instance.CurrentState;
            if (currState != StateManager.State.PuppetState && currState != StateManager.State.MotionIntentState) {
                return;
            }
            UpdateEndEffectorPoses();
            try {
                GeometryPose rightTargetPose = new GeometryPose {
                    position = UtilFunctions.Vector3ToGeometryPoint(outRightPos),
                    orientation = UtilFunctions.QuaternionToGeometryQuaternion(outRightQuat),
                };
                GeometryPose leftTargetPose = new GeometryPose {
                    position = UtilFunctions.Vector3ToGeometryPoint(outLeftPos),
                    orientation = UtilFunctions.QuaternionToGeometryQuaternion(outLeftQuat)
                };

                MoveitTarget moveitTarget = new MoveitTarget {
                    right_arm = UtilFunctions.PoseToPoseStamped(rightTargetPose, "/base_link"),
                    left_arm = UtilFunctions.PoseToPoseStamped(leftTargetPose, "/base_link"),
                };
                MoveItGoalPublisher.PublishPlan(moveitTarget);
            }
            catch {
                Debug.Log("SendPlanRequest failed :(");
                return;
            }
        }
    }
}