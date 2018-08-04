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
public class TapToLoadSpheres : MonoBehaviour, IInputClickHandler {
    private GameObject robot;
    protected FileInfo theSourceFile = null;
    protected StreamReader reader = null;
    protected string text = " "; // assigned to allow first line to be read below
    public GameObject NewSphere;

    public void OnInputClicked(InputClickedEventData eventData) {
        // Increase the scale of the object just as a response.
        robot = GameObject.Find("baxter");
        gameObject.transform.localScale += 0.05f * gameObject.transform.localScale;

        string path = Path.Combine(Application.persistentDataPath, "MyFile2.txt");
        theSourceFile = new FileInfo(path);
        using (reader = theSourceFile.OpenText()) {

            while (text != null) {
                text = reader.ReadLine();
                //Console.WriteLine(text);
                string[] splitString = text.Split(new string[] { " " }, StringSplitOptions.None);
                float xpos = float.Parse(splitString[2]);
                float ypos = float.Parse(splitString[3]);
                float zpos = float.Parse(splitString[4]);
                string itemname = splitString[5];

                float relx = xpos + robot.transform.position.x;
                float rely = ypos + robot.transform.position.y;
                float relz = zpos + robot.transform.position.z;


                GameObject s = Instantiate(NewSphere, this.transform.position, this.transform.rotation);
                //GameObject s = (GameObject)Instantiate(Resources.Load("Prefabs/RedSphere")); ;
                //s.transform.position = new Vector3(robot.transform.position.x, robot.transform.position.y, robot.transform.position.z);
                s.transform.position = new Vector3(relx, rely, relz);
                //s.transform.position = new Vector3(-2.1f+ robot.transform.position.x, 0.72f+ robot.transform.position.y, robot.transform.position.z+0.71f);
                s.transform.localScale = new Vector3(0.5f, 0.5f, 0.5f);
                //s.name = "spoon";
                s.name = itemname;
                //this.GetComponent<SphereNameManager>().addToDictionary(s); //adds the sphere to the dictionary
                //s.transform.GetChild(0).GetComponent<GetRelativePosition>().setItemName("bubbawubba!");
            }
        }
        eventData.Use(); // Mark the event as used, so it doesn't fall through to other handlers.
    }
}