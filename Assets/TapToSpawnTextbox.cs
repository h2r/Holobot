// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License. See LICENSE in the project root for license information.

using System;
using System.IO;
using UnityEngine;
using HoloToolkit.Unity.InputModule;
using HoloToolkit.Unity.InputModule.Utilities.Interactions;
using System.Collections.Generic;

/// <summary>
/// This class implements IInputClickHandler to handle the tap gesture.
/// It increases the scale of the object when tapped.
/// </summary>
/// 
public class TapToSpawnTextbox : MonoBehaviour, IInputClickHandler
{
    public GameObject textBox;
    public HoloToolkit.UI.Keyboard.KeyboardInputField keyboardInputField;

    public void Start()
    {
        textBox.SetActive(false);
         //textBox = GameObject.Find("SceneContent");

    }

    public void Update()
    {
    }

    public void OnInputClicked(InputClickedEventData eventData)
    {
        Debug.Log("Cube was clicked!");
        textBox.transform.position = new Vector3(gameObject.transform.position.x, gameObject.transform.position.y + 0.25f, gameObject.transform.position.z);
        textBox.SetActive(true);

        eventData.Use(); // Mark the event as used, so it doesn't fall through to other handlers.
    }
}