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
public class MoveToSphere : MonoBehaviour, IInputClickHandler {

    public void OnInputClicked(InputClickedEventData eventData) {
       
        gameObject.transform.localScale += 0.05f * gameObject.transform.localScale;

        eventData.Use(); // Mark the event as used, so it doesn't fall through to other handlers.
    }
}