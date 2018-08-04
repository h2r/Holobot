using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GetPosition : MonoBehaviour {

    public TextMesh tm;

	// Use this for initialization
	void Start () {
		
	}
	
	// Update is called once per frame
	void Update () {
        tm.text = "" + this.transform.position.x + " " + this.transform.position.y + " " + this.transform.position.z;
    }
}
