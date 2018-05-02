﻿#if UNITY_EDITOR
using UnityEngine;
using WebSocketSharp;
using System;
using System.Threading;
using System.Collections;
using System.Collections.Generic;


public class WebsocketClient : UniversalWebsocketClient
{
    private WebSocket ws;
    private int counter = 1;
	//public string message;
	private bool connected = false;
	//public Dictionary<string, string> messages = new Dictionary<string, string>();
    //public string ip_address;
    // "ws://138.16.160.16:9090" this is the address for iorek
    // ws://138.16.160.221:5678 this is the address for ursula
    // ws://128.30.25.147:9090
    void Awake () // putting this stuff inside Start was giving me errors. Other scripts were trying to use the client before it had connected
	{

        Debug.Log("instantiating websocket");
        ws = new WebSocket("ws://138.16.160.211:9090");

        ws.OnOpen += OnOpenHandler;
		ws.OnMessage += OnMessageHandler;
		ws.OnClose += OnCloseHandler;

        Debug.Log("Connecting to websocket");
		ws.ConnectAsync();
    }

	void OnApplicationQuit() {
		ws.CloseAsync();
	}


	public override void Subscribe(string topic, string type, string compression, int throttle_rate) // "none" compression, 0 throttle_rate
	{
		string msg = "{\"op\":\"subscribe\",\"id\":\"subscribe:/" + topic + ":" + counter + "\",\"type\":\"" + type + "\",\"topic\":\"/" + topic + "\",\"compression\":\"" + compression + "\",\"throttle_rate\":" + throttle_rate.ToString() + ",\"queue_length\":0}";
		Debug.Log (msg);
		ws.SendAsync(msg, OnSendComplete);
		counter++;
	}

	public void Unsubscribe(string topic) {
		string msg = "{\"op\":\"unsubscribe\",\"id\":\"unsubscribe:/" + topic + ":" + counter + "\",\"topic\":\"" + topic + "\"}";
		Debug.Log (msg);
		ws.SendAsync (msg, OnSendComplete);
	}

	public override void Advertise(string topic, string type)
	{
		string msg = "{\"op\":\"advertise\",\"id\":\"advertise:/" + topic + ":" + counter + "\",\"type\":\"" + type + "\",\"topic\":\"/" + topic + "\",\"latch\":false,\"queue_size\":0}";
		Debug.Log (msg);
		ws.SendAsync(msg, OnSendComplete);
		counter++;

	}

	public override void Publish(string topic, string message)
	{
		string msg = "{\"op\":\"publish\",\"id\":\"publish:/" + topic + ":" + counter + "\",\"topic\":\"/" + topic + "\",\"msg\":{\"data\":\"" + message + "\"},\"latch\":false}";
		ws.SendAsync(msg, OnSendComplete);
		counter++;
	}

	public override void SendEinMessage(string message, string arm)
	{
		Publish ("ein/" + arm + "/forth_commands", message);
	}
    private void OnMessageHandler(object sender, MessageEventArgs e)
	{
        string[] input = e.Data.Split (new char[] { ',' }, 2);
        string topic = input [0].Substring (12).Replace("\"", "");
        messages[topic] = e.Data;
    }
    private void OnOpenHandler(object sender, System.EventArgs e)
	{
		Debug.Log("WebSocket connected!");
		connected = true;
	}
    private void OnCloseHandler(object sender, CloseEventArgs e)
	{
		Debug.Log("WebSocket closed");
	}

    private void OnSendComplete(bool success)
	{
		//Debug.Log("Message sent successfully? " + success);
	}

	public bool IsConnected() {
		return connected;
	}
}
#endif

