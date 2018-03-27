using UnityEngine;
using UnityEngine.XR.WSA.Input;
using System.Collections.Generic;
using System.IO;

namespace Academy.HoloToolkit.Unity
{
    public class GestureManager : Singleton<GestureManager>
    {
        public GestureRecognizer NavigationRecognizer { get; private set; }
        public GestureRecognizer ManipulationRecognizer { get; private set; }
        public GestureRecognizer ActiveRecognizer { get; private set; }
        public bool IsNavigating { get; private set; }
        public Vector3 NavigationPosition { get; private set; }
        public bool IsManipulating { get; private set; }
        public Vector3 ManipulationPosition { get; private set; }
        public bool IsRecordingData { get; set; }
        public List<Record> PathRecord { get; private set; }
        public List<int> UndoPoints { get; private set; } // list of indicies in Path Record
        public List<GameObject> SavePointObjects { get; private set; }
        public Vector3 RobotOffset { get; set; }
        public bool RobotCalibrating { get; set; }
        public Vector3 RobotStart { get; set; }
        public bool HasCalibratedSphere { get; set; }

        void Awake()
        {
            IsRecordingData = false;
            PathRecord = new List<Record>();
            UndoPoints = new List<int>();
            SavePointObjects = new List<GameObject>();
            RobotOffset = Vector3.zero;
            RobotStart = Vector3.zero;
            RobotCalibrating = false;
            HasCalibratedSphere = false;

            NavigationRecognizer = new GestureRecognizer();
            NavigationRecognizer.SetRecognizableGestures(
                GestureSettings.Tap |
                GestureSettings.NavigationX); 

            NavigationRecognizer.Tapped += NavigationRecognizer_Tapped;
            NavigationRecognizer.NavigationStarted += NavigationRecognizer_NavigationStarted;
            NavigationRecognizer.NavigationUpdated += NavigationRecognizer_NavigationUpdated;
            NavigationRecognizer.NavigationCompleted += NavigationRecognizer_NavigationCompleted;
            NavigationRecognizer.NavigationCanceled += NavigationRecognizer_NavigationCanceled;

            // Instantiate the ManipulationRecognizer.
            ManipulationRecognizer = new GestureRecognizer();
            ManipulationRecognizer.SetRecognizableGestures(
                GestureSettings.ManipulationTranslate |
                GestureSettings.Tap);

            // Register for the Manipulation events on the ManipulationRecognizer.
            ManipulationRecognizer.Tapped += ManipulationRecognizer_Tapped;
            ManipulationRecognizer.ManipulationStarted += ManipulationRecognizer_ManipulationStarted;
            ManipulationRecognizer.ManipulationUpdated += ManipulationRecognizer_ManipulationUpdated;
            ManipulationRecognizer.ManipulationCompleted += ManipulationRecognizer_ManipulationCompleted;
            ManipulationRecognizer.ManipulationCanceled += ManipulationRecognizer_ManipulationCanceled;

            ResetGestureRecognizers();
        }

        public void Reset() // Not yet tested
        {
            IsRecordingData = false;
            PathRecord = new List<Record>();
            UndoPoints = new List<int>();
            SavePointObjects = new List<GameObject>();
        }

        void OnDestroy()
        {
            // Unregister the Tapped and Navigation events on the NavigationRecognizer.
            NavigationRecognizer.Tapped -= NavigationRecognizer_Tapped;

            NavigationRecognizer.NavigationStarted -= NavigationRecognizer_NavigationStarted;
            NavigationRecognizer.NavigationUpdated -= NavigationRecognizer_NavigationUpdated;
            NavigationRecognizer.NavigationCompleted -= NavigationRecognizer_NavigationCompleted;
            NavigationRecognizer.NavigationCanceled -= NavigationRecognizer_NavigationCanceled;

            // Unregister the Manipulation events on the ManipulationRecognizer.
            ManipulationRecognizer.ManipulationStarted -= ManipulationRecognizer_ManipulationStarted;
            ManipulationRecognizer.ManipulationUpdated -= ManipulationRecognizer_ManipulationUpdated;
            ManipulationRecognizer.ManipulationCompleted -= ManipulationRecognizer_ManipulationCompleted;
            ManipulationRecognizer.ManipulationCanceled -= ManipulationRecognizer_ManipulationCanceled;
        }


        // Revert back to the default GestureRecognizer.
        public void ResetGestureRecognizers()
        {
            // Default to the movement gestures.
            if (ActiveRecognizer != null)
            {
                return;
            }
            Transition(ManipulationRecognizer);
        }

        // set the current GestureRecognizer
        public void Transition(GestureRecognizer newRecognizer)
        {
            if (newRecognizer == null)
            {
                return;
            }

            if (ActiveRecognizer != null)
            {
                if (ActiveRecognizer == newRecognizer)
                {
                    return;
                }
                ActiveRecognizer.CancelGestures();
                ActiveRecognizer.StopCapturingGestures();
            }

            newRecognizer.StartCapturingGestures();
            ActiveRecognizer = newRecognizer;
        }

        public void RecordMovement(Vector3 pos, float delTime)
        {
            // TODO: add radius filtering to not record small jitters
            PathRecord.Add(new Record(pos, delTime));
            if (PathRecord.Count == 1)
            {
                RecordUndoPoint();
            }
        }

        public void RecordUndoPoint()
        {
            if (PathRecord.Count > 0)
            {
                if (UndoPoints.Count == 0)
                {
                    UndoPoints.Add(0); // get the first recorded point
                }
                else
                {
                    UndoPoints.Add(PathRecord.Count - 1);
                }
            }

            // code to visualize the last undo point

            //GameObject realCube = GameObject.Find("Cube");
            //Vector3 pos = PathRecord[UndoPoints[UndoPoints.Count - 1]].Position;
            //Quaternion rot = PathRecord[UndoPoints[UndoPoints.Count - 1]].Rotation;
            //GameObject shadow = Instantiate(realCube, pos, rot);
            //shadow.layer = 1; // the transparent layer that the cursor does not repond to
            //shadow.GetComponent<Renderer>().material.color = Color.cyan;
            //SavePointObjects.Add(shadow);

        }

        public void UndoAction()
        {
            if (SavePointObjects.Count > 0)
            {
                GameObject realCube = GameObject.Find("Cube");
                GameObject savedCube = SavePointObjects[SavePointObjects.Count - 1];
                realCube.transform.position = savedCube.transform.position;
                realCube.transform.rotation = savedCube.transform.rotation;
                int startIdx = UndoPoints[UndoPoints.Count - 1];
                PathRecord.RemoveRange(startIdx, PathRecord.Count - startIdx);
                Destroy(SavePointObjects[SavePointObjects.Count - 1]);
                SavePointObjects.RemoveAt(SavePointObjects.Count - 1);
                UndoPoints.RemoveAt(UndoPoints.Count - 1);
            }

        }

        public void WritePathData()
        {
            string path = @"C:\Users\brown\Documents\Unity\HoloControl\Holobot\data.txt";
            if (!File.Exists(path))
            {
                File.Create(path); 
            }
            TextWriter writer = new StreamWriter(path);
            writer.WriteLine("dt, sx, sy, sz");
            // Debug.Log("Line written");
            for (int i = 0; i < PathRecord.Count; i++)
            {
                float dt = PathRecord[i].DelTime;
                Vector3 pos = PathRecord[i].Position;
                writer.WriteLine("{0}, {1}, {2}, {3}",
                    dt, pos.x, pos.y, pos.z);
            }
            

        }

        // Private Methods
        private void NavigationRecognizer_NavigationStarted(NavigationStartedEventArgs obj)
        {
            if (HandsManager.Instance.FocusedGameObject != null)
            {
                IsNavigating = true;
                NavigationPosition = Vector3.zero;
                HandsManager.Instance.FocusedGameObject.SendMessageUpwards("PerformRotation");
            }
        }

        private void NavigationRecognizer_NavigationUpdated(NavigationUpdatedEventArgs obj)
        {
            if (HandsManager.Instance.FocusedGameObject != null)
            {
                IsNavigating = true;
                NavigationPosition = obj.normalizedOffset;
                HandsManager.Instance.FocusedGameObject.SendMessageUpwards("PerformRotation");
            }
        }

        private void NavigationRecognizer_NavigationCompleted(NavigationCompletedEventArgs obj)
        {
            IsNavigating = false;
        }

        private void NavigationRecognizer_NavigationCanceled(NavigationCanceledEventArgs obj)
        {
            IsNavigating = false;
        }

        private void ManipulationRecognizer_ManipulationStarted(ManipulationStartedEventArgs obj)
        {
            if (HandsManager.Instance.FocusedGameObject != null)
            {
                IsManipulating = true;

                ManipulationPosition = Vector3.zero;

                HandsManager.Instance.FocusedGameObject.SendMessageUpwards("PerformManipulationStart", ManipulationPosition);
            }
        }

        private void ManipulationRecognizer_ManipulationUpdated(ManipulationUpdatedEventArgs obj)
        {
            if (HandsManager.Instance.FocusedGameObject != null)
            {
                IsManipulating = true;

                ManipulationPosition = obj.cumulativeDelta;

                HandsManager.Instance.FocusedGameObject.SendMessageUpwards("PerformManipulationUpdate", ManipulationPosition);
            }
        }

        private void ManipulationRecognizer_ManipulationCompleted(ManipulationCompletedEventArgs obj)
        {
            IsManipulating = false;
        }

        private void ManipulationRecognizer_ManipulationCanceled(ManipulationCanceledEventArgs obj)
        {
            IsManipulating = false;
        }

        private void ManipulationRecognizer_Tapped(TappedEventArgs obj)
        {
            GameObject focusedObject = InteractibleManager.Instance.FocusedGameObject;

            if (focusedObject != null)
            {
                focusedObject.SendMessageUpwards("OnSelect", focusedObject);
            }
        }

        private void NavigationRecognizer_Tapped(TappedEventArgs obj)
        {
            GameObject focusedObject = InteractibleManager.Instance.FocusedGameObject;

            if (focusedObject != null)
            {
                focusedObject.SendMessageUpwards("OnSelect", focusedObject);
            }
        }
    }

    public class Record
    {
        public Vector3 Position { get; private set; }
        // public Quaternion Rotation { get; private set; }
        public float DelTime;
        public Record(Vector3 pos, float dt)
        {
            Position = pos;
            DelTime = dt;
        }
    }
}