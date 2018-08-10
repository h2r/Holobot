using HoloToolkit.UI.Keyboard;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

namespace HoloToolkit.Unity {
    //namespace Academy.HoloToolkit.Unity {
    public class TextboxManager : Singleton<TextboxManager> {
        private GameObject currTextboxOwner = null;
        private GameObject textBox;
        private HoloToolkit.UI.Keyboard.KeyboardInputField keyboardInputField;
        private Boolean isTextBoxGone = true;


        void Start() {
            textBox = GameObject.Find("KeyboardTestCanvas");
            keyboardInputField = textBox.transform.GetChild(0).transform.GetChild(0).GetComponent<KeyboardInputField>();
            textBox.transform.position = new Vector3(99, 99, 99);
        }

        public void OnLabelTap(GameObject callerWhoIsTapped) {
            if(callerWhoIsTapped == currTextboxOwner) {
                if(isTextBoxGone) {
                    textBox.transform.position = new Vector3(callerWhoIsTapped.transform.position.x, callerWhoIsTapped.transform.position.y + 0.25f, callerWhoIsTapped.transform.position.z);
                    isTextBoxGone = false;
                }
                else {
                    //keyboardInputField.text //text for to update the name
                    textBox.transform.position = new Vector3(99, 99, 99);
                    isTextBoxGone = true;
                }
            }
            else {
                textBox.transform.position = new Vector3(callerWhoIsTapped.transform.position.x, callerWhoIsTapped.transform.position.y + 0.25f, callerWhoIsTapped.transform.position.z);
                textBox.transform.GetChild(0).transform.GetChild(0).transform.GetChild(2).GetComponent<Text>().text = callerWhoIsTapped.name;
                keyboardInputField.text = "";
                currTextboxOwner = callerWhoIsTapped;
                isTextBoxGone = false;
            }
            

        }


        private void Update() {
        }

    }
}