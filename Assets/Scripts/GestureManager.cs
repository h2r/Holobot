﻿using UnityEngine;
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

        public Vector3 RobotOffset { get; set; }
        public bool RobotCalibrating { get; set; }
        public Vector3 RobotStart { get; set; }
        public bool HasCalibratedSphere { get; set; }
        public List<Vector3> UnityMotionPlanEndpoints { get; set; }

        void Awake()
        {
            IsRecordingData = false;
            RobotOffset = Vector3.zero;
            RobotStart = Vector3.zero;
            RobotCalibrating = false;
            HasCalibratedSphere = false;
            UnityMotionPlanEndpoints = new List<Vector3>();
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
}