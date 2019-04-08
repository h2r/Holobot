using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ObjectHighlighter : MonoBehaviour {

    private GameObject MarkerTemplate;
    private List<GameObject> Markers;
    private List<string> MarkerCoords;

	// Use this for initialization
	void Start () {
        Debug.Log("Initialized ObjectHighlighter");
        MarkerTemplate = GameObject.Find("MarkerTemplate");
        Markers = new List<GameObject>();
        MarkerCoords = new List<string>();
        PlaceMarker("2,1,3");
	}
	
	// Update is called once per frame
	void Update () {
		
	}

    void PlaceMarker(string coordstring) {
        var coords = coordstring.Split(',');
        Debug.Assert(coords.Length == 3);
        Debug.Log(string.Format("({0}, {1}, {2})", coords[0], coords[1], coords[2]));
        GameObject markerObj = Instantiate(MarkerTemplate);
        markerObj.transform.position = GetMarkerPosition(coordstring);
    }

    Vector3 GetMarkerPosition(string coordstring) {
        var coords = coordstring.Split(',');
        float shelfNum = float.Parse(coords[0]);
        float shelfRow = float.Parse(coords[1]);
        float shelfCol = float.Parse(coords[2]);
        float x = (float)(0.45 - shelfCol * 0.1);
        float y = (float)(0.05 + shelfNum * 0.2);
        float z = (float)(0.25 - shelfRow * 0.1);
        return new Vector3(x, y, z);
    }
}
