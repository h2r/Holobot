using System;
using HoloToolkit.Unity;
using HoloToolkit.Unity.InputModule;
using UnityEngine;

public class ScanManager : MonoBehaviour, IInputClickHandler {
    public TextMesh InstructionTextMesh;
    public Transform FloorPrefab;
    public Transform WallPrefab;
    public Transform SurfacePrefag;

    // Use this for initialization
    void Start() {
        InputManager.Instance.PushFallbackInputHandler(this.gameObject);
        SpatialUnderstanding.Instance.RequestBeginScanning();
        SpatialUnderstanding.Instance.ScanStateChanged += ScanStateChanged;
    }

    private void ScanStateChanged() {
        if (SpatialUnderstanding.Instance.ScanState == SpatialUnderstanding.ScanStates.Scanning) {
            LogSurfaceState();
        }
        else if (SpatialUnderstanding.Instance.ScanState == SpatialUnderstanding.ScanStates.Done) {
            InstanciateObjectOnFloor();
            InstanciateObjectOnSurface();
            InstanciateObjectOnWall();
        }
    }

    private void OnDestroy() {
        SpatialUnderstanding.Instance.ScanStateChanged -= ScanStateChanged;
    }

    // Update is called once per frame
    void Update() {
        switch (SpatialUnderstanding.Instance.ScanState) {
            case SpatialUnderstanding.ScanStates.None:
                break;
            case SpatialUnderstanding.ScanStates.ReadyToScan:
                break;
            case SpatialUnderstanding.ScanStates.Scanning:
                this.LogSurfaceState();
                break;
            case SpatialUnderstanding.ScanStates.Finishing:
                this.InstructionTextMesh.text = "State: Finishing Scan";
                break;
            case SpatialUnderstanding.ScanStates.Done:
                this.InstructionTextMesh.text = "State: Scan Finished";
                break;
            default:
                break;
        }
    }

    private void LogSurfaceState() {
        IntPtr statsPtr = SpatialUnderstanding.Instance.UnderstandingDLL.GetStaticPlayspaceStatsPtr();
        if (SpatialUnderstandingDll.Imports.QueryPlayspaceStats(statsPtr) != 0) {
            var stats = SpatialUnderstanding.Instance.UnderstandingDLL.GetStaticPlayspaceStats();
            this.InstructionTextMesh.text = string.Format("TotalSurfaceArea: {0:0.##}\nWallSurfaceArea: {1:0.##}\nHorizSurfaceArea: {2:0.##}", stats.TotalSurfaceArea, stats.WallSurfaceArea, stats.HorizSurfaceArea);
        }
    }

    public void OnInputClicked(InputClickedEventData eventData) {
        this.InstructionTextMesh.text = "Requested Finish Scan";

        SpatialUnderstanding.Instance.RequestFinishScan();
    }

    private void InstanciateObjectOnFloor() {
        const int QueryResultMaxCount = 512;

        SpatialUnderstandingDllTopology.TopologyResult[] _resultsTopology = new SpatialUnderstandingDllTopology.TopologyResult[QueryResultMaxCount];

        var minLengthFloorSpace = 0.25f;
        var minWidthFloorSpace = 0.25f;

        var resultsTopologyPtr = SpatialUnderstanding.Instance.UnderstandingDLL.PinObject(_resultsTopology);
        var locationCount = SpatialUnderstandingDllTopology.QueryTopology_FindPositionsOnFloor(minLengthFloorSpace, minWidthFloorSpace, _resultsTopology.Length, resultsTopologyPtr);

        if (locationCount > 0) {
            Instantiate(this.FloorPrefab, _resultsTopology[0].position, Quaternion.LookRotation(_resultsTopology[0].normal, Vector3.up));

            this.InstructionTextMesh.text = "Placed the hologram";
        }
        else {
            this.InstructionTextMesh.text = "I can't found the enough space to place the hologram.";
        }
    }

    private void InstanciateObjectOnWall() {
        const int QueryResultMaxCount = 512;

        SpatialUnderstandingDllTopology.TopologyResult[] _resultsTopology = new SpatialUnderstandingDllTopology.TopologyResult[QueryResultMaxCount];

        var minWidthOfWallSpace = 0.1f;
        var minHeightAboveFloor = 1f;
        var minHeightOfWallSpace = 1f;
        var minFacingClearance = 0.1f;

        var resultsTopologyPtr = SpatialUnderstanding.Instance.UnderstandingDLL.PinObject(_resultsTopology);
        var locationCount = SpatialUnderstandingDllTopology.QueryTopology_FindPositionsOnWalls(
            minHeightOfWallSpace, minWidthOfWallSpace, minHeightAboveFloor, minFacingClearance,
            _resultsTopology.Length, resultsTopologyPtr);

        if (locationCount > 0) {
            Instantiate(this.WallPrefab,
                _resultsTopology[0].position,
                Quaternion.LookRotation(_resultsTopology[0].normal, Vector3.up));
        }
    }

    private void InstanciateObjectOnSurface() {
        const int QueryResultMaxCount = 512;

        SpatialUnderstandingDllTopology.TopologyResult[] _resultsTopology = new SpatialUnderstandingDllTopology.TopologyResult[QueryResultMaxCount];

        var minHeight = 0.02f;
        var maxHeight = 1f;
        var minFacingClearance = 0.02f;

        var resultsTopologyPtr = SpatialUnderstanding.Instance.UnderstandingDLL.PinObject(_resultsTopology);
        var locationCount = SpatialUnderstandingDllTopology.QueryTopology_FindPositionsSittable(
            minHeight, maxHeight, minFacingClearance,
            _resultsTopology.Length, resultsTopologyPtr);

        if (locationCount > 0) {
            Instantiate(this.SurfacePrefag,
                _resultsTopology[0].position,
                Quaternion.LookRotation(_resultsTopology[0].normal, Vector3.up));
        }
    }
}