// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License. See LICENSE in the project root for license information.

using System;
using System.IO;
using UnityEngine;
using HoloToolkit.Unity.InputModule;
using HoloToolkit.Unity.InputModule.Utilities.Interactions;
using System.Collections.Generic;
using HoloToolkit.UI.Keyboard;

/// <summary>
/// This class implements IInputClickHandler to handle the tap gesture.
/// It increases the scale of the object when tapped.
/// </summary>
/// 
public class TapToSpawnTextbox : MonoBehaviour, IInputClickHandler
{
    private Boolean isTextBoxThere = false;
    private GameObject textBox;
    private HoloToolkit.UI.Keyboard.KeyboardInputField keyboardInputField;

    public void Start()
    {
        textBox = GameObject.Find("KeyboardTestCanvas");
        keyboardInputField = textBox.transform.GetChild(0).transform.GetChild(0).GetComponent<KeyboardInputField>();
        //textBox.SetActive(false);
        textBox.transform.position = new Vector3(99, 99, 99);
        
    }

    public void Update()
    {
    }

    public void OnInputClicked(InputClickedEventData eventData)
    {
        //Debug.Log("Cube was clicked!");
        if (!isTextBoxThere) {
            textBox.transform.position = new Vector3(gameObject.transform.position.x, gameObject.transform.position.y + 0.25f, gameObject.transform.position.z);
            //textBox.SetActive(true);
            isTextBoxThere = true;
        }
        else {
            //textBox.SetActive(false);
            textBox.transform.position = new Vector3(99, 99, 99);
            isTextBoxThere = false;
        }

        eventData.Use(); // Mark the event as used, so it doesn't fall through to other handlers.
    }
}