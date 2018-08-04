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
public class TapToSaveSpheres : MonoBehaviour, IInputClickHandler {
    private GameObject robot;
    public void OnInputClicked(InputClickedEventData eventData) {
        // Increase the scale of the object just as a response.
        robot = GameObject.Find("baxter");
        gameObject.transform.localScale += 0.05f * gameObject.transform.localScale;

        string path = Path.Combine(Application.persistentDataPath, "MyFile2.txt");
        using (TextWriter writer = File.CreateText(path)) {
            // TODO write text here
            //writer.WriteLine("hello here is a file i have made by eric");
            SphereNameManager snm = GameObject.Find("TapForMoreSpheres").GetComponent<SphereNameManager>();
            Dictionary<GameObject, int> sphereDict = snm.getDict();
            foreach (KeyValuePair<GameObject, int> sphere_num in sphereDict) {
                //Now you can access the key and value both separately from this attachStat as:
                float xd = sphere_num.Key.transform.position.x - robot.transform.position.x;
                float yd = sphere_num.Key.transform.position.y - robot.transform.position.y;
                float zd = sphere_num.Key.transform.position.z - robot.transform.position.z;
                String sphere_name = sphere_num.Key.name;
                int sphere_val = sphere_num.Value;
                writer.WriteLine(sphere_val.ToString() + " " + sphere_name + " " + xd + " " + yd + " " + zd);
            }
            
        }

        string path2 = Path.Combine(Application.persistentDataPath, "robot_test.txt");
        using (TextWriter writer = File.CreateText(path2)) {
            // TODO write text here
            //writer.WriteLine("hello here is a file i have made by eric");

                //Now you can access the key and value both separately from this attachStat as:
                float xd = robot.transform.position.x;
                float yd = robot.transform.position.y;
                float zd = robot.transform.position.z;

                float xr = robot.transform.rotation.x;
                float yr = robot.transform.rotation.y;
                float zr = robot.transform.rotation.z;
                writer.WriteLine( xd + " " + yd + " " + zd + " " + xr + " " + yr + " " + zr);

        }

        eventData.Use(); // Mark the event as used, so it doesn't fall through to other handlers.
    }
}