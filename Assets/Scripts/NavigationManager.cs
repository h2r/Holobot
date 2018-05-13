using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Academy.HoloToolkit.Unity {
    public class NavigationManager : Singleton<NavigationManager> {
        bool navigating = false;
        // Use this for initialization
        void Start() {

        }

        // Update is called once per frame
        void Update() {
            if (StateManager.Instance.CurrentState == StateManager.State.NavigatingState) {

            }
        }
    }
}