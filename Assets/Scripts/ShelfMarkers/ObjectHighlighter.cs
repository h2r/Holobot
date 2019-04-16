using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ObjectHighlighter : MonoBehaviour {

    private GameObject MarkerTemplate;
    private List<GameObject> Markers;
    private List<string> MarkerCoords;
    private readonly int numShelves = 4;

	// Use this for initialization
	void Start () {
        Debug.Log("Initialized ObjectHighlighter");
        MarkerTemplate = GameObject.Find("MarkerTemplate");
        Markers = new List<GameObject>();
        MarkerCoords = new List<string>();
        //PlaceMarker("2,1,3"); // shelf from bottom, row, col
        for (int shelf = 0; shelf < 4; shelf++) {
            for (int row = 0; row < 3; row++) {
                for (int col = 0; col < 15; col++) {
                    PlaceMarker(shelf, row, col);
                }
            }
        }
        HighlightMarker("2,1,3");
	}
	
	// Update is called once per frame
	void Update () {
		
	}

    void PlaceMarker(string coordstring) {
        var coords = coordstring.Split(',');
        Debug.Assert(coords.Length == 3);
        Debug.Log(string.Format("({0}, {1}, {2})", coords[0], coords[1], coords[2]));
        GameObject markerObj = Instantiate(MarkerTemplate);
        markerObj.name = "Marker_" + coordstring;
        markerObj.transform.parent = GameObject.Find("Shelf").transform;
        markerObj.transform.localPosition = GetMarkerPosition(coordstring);
    }

    void PlaceMarker(int shelf, int row, int col) {
        PlaceMarker(string.Format("{0},{1},{2}", shelf, row, col));
    }

    void HighlightMarker(string coordstring) {
        GameObject marker = GameObject.Find("Marker_" + coordstring);
        if (marker != null) { // marker exists
            Material highlightMaterial = (Material)Resources.Load("Assets/Materials/MarkerActiveMaterial", typeof(Material));
            marker.GetComponent<Renderer>().material = highlightMaterial;
            //marker.GetComponent<Renderer>().material.color.a = 1.0;
        }
    }

    Vector3 GetMarkerPosition(string coordstring) {
        var coords = coordstring.Split(','); // shelf, row, col
        float shelfNum = numShelves - float.Parse(coords[0]) - 1;
        float shelfRow = float.Parse(coords[1]);
        float shelfCol = float.Parse(coords[2]);
        Debug.Assert(0 <= shelfNum && shelfNum < 4); // shelf number from bottom
        Debug.Assert(0 <= shelfRow && shelfRow < 3); // 3 rows per shelf
        Debug.Assert(0 <= shelfCol && shelfCol < 15); // 15 objects across three cubicles
        float x = (float)(0.425 - (shelfCol * 0.05866666666));
        float shelfGap = 0.015f;
        if (shelfCol > 4) {
            x -= shelfGap;
        }
        if (shelfCol > 9) {
            x -= shelfGap;
        }
        float y = (float)(0.03 + shelfNum * 0.3);
        float z = (float)(0.1 - shelfRow * 0.1);
        return new Vector3(x, y, z);
    }
}
