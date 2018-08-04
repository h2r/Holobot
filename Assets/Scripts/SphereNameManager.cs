using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SphereNameManager : MonoBehaviour {

    private int sphereNum;
    private Dictionary<GameObject, int> sphereDict = new Dictionary<GameObject, int>();
    // Use this for initialization
    void Start() {
        sphereNum = 0;
    }

    // Update is called once per frame
    void Update() {

    }

    public void addToDictionary(GameObject sphere) 
    {
        sphereDict.Add(sphere, sphereNum);
        sphereNum += 1;
    }

    public int getSphereNum(GameObject sphere) {
        try {
            return sphereDict[sphere];
        }
        catch (Exception e) {
            return -1;
        }
    }

    public Dictionary<GameObject, int> getDict() {
        return sphereDict;
    }

   
}
