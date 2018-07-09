using RosSharp.RosBridgeClient;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

namespace HoloToolkit.Unity {
    // This script publishes commands to the topic /ein/right/forth_commands
    public class EinRightForthCommandsPublisher : Publisher {

        public GameObject leftController;
        public GameObject rightController;
        public GameObject Robot;

        public MoveItGoalPublisher MoveItGoalPublisher;
        public string PlanTopic = "/goal_pose";

        private StandardString message;
        private string rightArmCmd;
        private string leftArmCmd;

        private Vector3 outLeftPos, outRightPos;
        private Quaternion outLeftQuat, outRightQuat;

        protected override void Start() {
            rosSocket = GetComponent<RosConnector>().RosSocket;

            publicationId = rosSocket.Advertise(Topic, "std_msgs/String");
            message = new StandardString();
            SendCommand("baseGoCfg");
            InvokeRepeating("SendEinCommands", .1f, .25f);
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
            if (StateManager.Instance.CurrentState != StateManager.State.PuppetState) {
                return;
            }
            outLeftPos = UtilFunctions.UnityToRosPositionAxisConversion(leftController.transform.localPosition);
            outRightPos = UtilFunctions.UnityToRosPositionAxisConversion(rightController.transform.localPosition);
            outLeftQuat = UtilFunctions.UnityToRosRotationAxisConversion(leftController.transform.localRotation);
            outRightQuat = UtilFunctions.UnityToRosRotationAxisConversion(rightController.transform.localRotation);

            leftArmCmd = "switchToLeftArm " + outLeftPos.x + " " + outLeftPos.y + " " + outLeftPos.z + " " +
                    outLeftQuat.x + " " + outLeftQuat.y + " " + outLeftQuat.z + " " + outLeftQuat.w + " moveToEEPose";
            rightArmCmd = "switchToRightArm " + outRightPos.x + " " + outRightPos.y + " " + outRightPos.z + " " +
                    outRightQuat.x + " " + outRightQuat.y + " " + outRightQuat.z + " " + outRightQuat.w + " moveToEEPose";
            if (StateManager.Instance.UpdateRightArm) {
                SendCommand(rightArmCmd);
            }
            else if (StateManager.Instance.UpdateLeftArm) {
                SendCommand(leftArmCmd);
            }
        }

        public void SendCommand(string command) {
            StandardString msg = new StandardString {
                data = command
            };
            rosSocket.Publish(publicationId, msg);
            Debug.Log("Sent command: " + msg.data);
        }

        private GeometryQuaternion QuaternionToGeometryQuaternion(Quaternion q) {
            return new GeometryQuaternion {
                x = q.x,
                y = q.y,
                z = q.z,
                w = q.w
            };
        }

        private GeometryPoint Vector3ToGeometryPoint(Vector3 v) {
            return new GeometryPoint {
                x = v.x,
                y = v.y,
                z = v.z
            };
        }

        public void SendPlanRequest() {
            try {
                GeometryPose rightTargetPose = new GeometryPose {
                    position = Vector3ToGeometryPoint(outRightPos),
                    orientation = QuaternionToGeometryQuaternion(outRightQuat)
                };
                GeometryPose leftTargetPose = new GeometryPose {
                    position = Vector3ToGeometryPoint(outLeftPos),
                    orientation = QuaternionToGeometryQuaternion(outLeftQuat)
                };
                MoveitTarget moveitTarget = new MoveitTarget {
                    right_arm = rightTargetPose,
                    left_arm = leftTargetPose
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