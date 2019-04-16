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
        InitializeMarkers();
        HighlightMarker("2,1,3");
        HighlightMarker("3,1,3");
        HighlightMarker("0,0,3");
	}
	
	// Update is called once per frame
	void Update () {
		
	}

    void InitializeMarkers() {
        for (int shelf = 0; shelf < 4; shelf++) {
            for (int row = 0; row < 3; row++) {
                for (int col = 0; col < 15; col++) {
                    PlaceMarker(shelf, row, col);
                }
            }
        }
    }

    bool IsValidCoordString(string coordstring) {
        Debug.Assert(!coordstring.Contains("."));
        var coords = coordstring.Split(','); // shelf, row, col
        var coordVals = ExtractCoords(coordstring);
        int shelfNum = coordVals[0];
        int shelfRow = coordVals[1];
        int shelfCol = coordVals[2];
        return (0 <= shelfNum && shelfNum < 4) && (0 <= shelfRow && shelfRow < 3) && (0 <= shelfCol && shelfCol < 15);
    }

    int[] ExtractCoords(string coordstring) {
        var coords = coordstring.Split(','); // shelf, row, col
        var coordVals = new int[3];
        coordVals[0] = numShelves - int.Parse(coords[0]) - 1; // shelf number from top
        coordVals[1] = int.Parse(coords[1]); // 3 rows per shelf
        coordVals[2] = int.Parse(coords[2]); // 15 objects across three cubicles
        return coordVals;
    }

    void PlaceMarker(string coordstring) {
        var coords = coordstring.Split(',');
        Debug.Assert(coords.Length == 3);
        Debug.Log(string.Format("({0}, {1}, {2})", coords[0], coords[1], coords[2]));
        GameObject markerObj = Instantiate(MarkerTemplate);
        markerObj.name = "Marker_" + coordstring;
        markerObj.tag = "Marker";
        markerObj.transform.parent = GameObject.Find("Shelf").transform;
        markerObj.transform.localPosition = GetMarkerPosition(coordstring);
    }

    void PlaceMarker(int shelf, int row, int col) {
        PlaceMarker(string.Format("{0},{1},{2}", shelf, row, col));
    }

    void HighlightMarker(string coordstring) {
        Debug.Assert(IsValidCoordString(coordstring));
        var markers = GameObject.FindGameObjectsWithTag("Marker");
        foreach (GameObject m in markers) {
            m.GetComponent<Renderer>().material = (Material)Resources.Load("MarkerInactiveMaterial", typeof(Material));
        }
        GameObject marker = GameObject.Find("Marker_" + coordstring);
        Debug.Assert(marker != null);
        marker.GetComponent<Renderer>().material = (Material)Resources.Load("MarkerActiveMaterial", typeof(Material));
    }

    Vector3 GetMarkerPosition(string coordstring) {
        Debug.Assert(IsValidCoordString(coordstring));
        var coordVals = ExtractCoords(coordstring);
        int shelfNum = coordVals[0];
        int shelfRow = coordVals[1];
        int shelfCol = coordVals[2];
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
