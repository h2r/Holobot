using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GetRelativePosition : MonoBehaviour {

    public TextMesh tm;
    private GameObject robot;
    private SphereNameManager snm;
    private string itemName;

	// Use this for initialization
	void Start () {
        robot = GameObject.Find("baxter");
        snm = GameObject.Find("TapForMoreSpheres").GetComponent<SphereNameManager>();

        try {

            int num = snm.getSphereNum(this.transform.parent.gameObject);
            float xd = this.transform.parent.transform.position.x - robot.transform.position.x;
            float yd = this.transform.parent.transform.position.y - robot.transform.position.y;
            float zd = this.transform.parent.transform.position.z - robot.transform.position.z;

            tm.text = "" + num + " " + xd + " " + yd + " " + zd;
        }
        catch (Exception e) {

        }
    }
	
	// Update is called once per frame
	void Update () {
        tm.text = this.transform.parent.name;
    }
}
