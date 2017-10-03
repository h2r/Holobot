using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public abstract class UniversalWebsocketClient : MonoBehaviour {
    public Dictionary<string, string> messages = new Dictionary<string, string>();



    public abstract void Subscribe(string topic, string v1, string v2, int v3);

    public abstract void Advertise(string v1, string v2);

    public abstract void SendEinMessage(string v1, string v2);
}
