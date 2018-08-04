// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License. See LICENSE in the project root for license information.

using System;
using UnityEngine;
using HoloToolkit.Unity.InputModule;
using HoloToolkit.Unity.InputModule.Utilities.Interactions;

/// <summary>
/// This class implements IInputClickHandler to handle the tap gesture.
/// It increases the scale of the object when tapped.
/// </summary>
public class TapForSpheres : MonoBehaviour, IInputClickHandler {

    public GameObject NewSphere;
    public void OnInputClicked(InputClickedEventData eventData) {
        // Increase the scale of the object just as a response.

        gameObject.transform.localScale += 0.05f * gameObject.transform.localScale;

        //GameObject s = new GameObject();
        //s = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        //s.transform.localScale = new Vector3(0.1f, 0.1f, 0.1f);
        
        GameObject s = Instantiate(NewSphere, this.transform.position, this.transform.rotation);
        //GameObject s = (GameObject)Instantiate(Resources.Load("Prefabs/RedSphere")); ;
        s.transform.position = new Vector3(this.transform.position.x, this.transform.position.y+0.5f, this.transform.position.z);
        s.transform.localScale = new Vector3(0.5f, 0.5f, 0.5f);
        this.GetComponent<SphereNameManager>().addToDictionary(s); //adds the sphere to the dictionary




        /*GameObject g = new GameObject();
        g.AddComponent<MeshRenderer>();
        g.AddComponent<TextMesh>();
        g.GetComponent<TextMesh>().text = "HELLO PEOPLE LOOK AT ME HERE I HOPE IM BIG";
        g.GetComponent<TextMesh>().fontSize = 288;
        g.GetComponent<TextMesh>().color = Color.red;
        g.transform.localScale = new Vector3(0.005F, 0.005f, 0.005f);
        g.transform.parent = s.transform;
        */
        eventData.Use(); // Mark the event as used, so it doesn't fall through to other handlers.
    }
}