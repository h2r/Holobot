using System;
using System.Collections;
using System.Collections.Generic;

using UnityEngine;
#if !UNITY_EDITOR

using Windows.Networking.Sockets;
using Windows.Storage.Streams;

#endif

public class UWPWebSocketClient : UniversalWebsocketClient {

    // Windows UWP variables    
#if !UNITY_EDITOR
    private MessageWebSocket messageWebSocket;
    private DataWriter messageWriter;
#endif

    //General variables
    System.Uri uri = new System.Uri("ws://138.16.160.16:9090");
    //public Dictionary<string, string> messages = new Dictionary<string, string>();
    private int counter = 1;

    void Awake() {
        WebSocketStart();
    }
    
    void WebSocketStart () {
#if !UNITY_EDITOR
        messageWebSocket = new MessageWebSocket();
        messageWebSocket.Control.MessageType = SocketMessageType.Utf8;
        messageWebSocket.MessageReceived += MessageReceived;
        messageWebSocket.Closed += OnClosed;

        messageWebSocket.ConnectAsync(uri);
        messageWriter = new DataWriter(messageWebSocket.OutputStream);
#endif
    }

    public override void Subscribe(string topic, string type, string compression, int throttle_rate) {
        string msg = "{\"op\":\"subscribe\",\"id\":\"subscribe:/" + topic + ":" + counter + "\",\"type\":\"" + type + "\",\"topic\":\"/" + topic + "\",\"compression\":\"" + compression + "\",\"throttle_rate\":" + throttle_rate.ToString() + ",\"queue_length\":0}";
        SendAsync(msg);
        counter++;
    }


    void Publish(string topic, string message) {
        string msg = "{\"op\":\"publish\",\"id\":\"publish:/" + topic + ":" + counter + "\",\"topic\":\"/" + topic + "\",\"msg\":{\"data\":\"" + message + "\"},\"latch\":false}";
        SendAsync(msg);
        counter++;
    }

    public override void SendEinMessage(string message, string arm) {
        Publish("ein/" + arm + "/forth_commands", message);
    }

    public override void Advertise(string topic, string type) {
        string msg = "{\"op\":\"advertise\",\"id\":\"advertise:/" + topic + ":" + counter + "\",\"type\":\"" + type + "\",\"topic\":\"/" + topic + "\",\"latch\":false,\"queue_size\":0}";
        SendAsync(msg);
        counter++;
    }

    void SendAsync(string message) {
#if WINDOWS_UWP
        messageWriter.WriteString(message);
        try {
            messageWriter.StoreAsync();
        } catch (Exception ex) {
            Debug.Log(ex.ToString());
        }
#endif
    }
#if !UNITY_EDITOR

    //The MessageReceived event handler.
    private void MessageReceived(MessageWebSocket sender, MessageWebSocketMessageReceivedEventArgs args) {
        DataReader messageReader = args.GetDataReader();
        messageReader.UnicodeEncoding = UnicodeEncoding.Utf8;
        string messageString = messageReader.ReadString(messageReader.UnconsumedBufferLength);

        //Add code here to do something with the string that is received.
        string[] input = messageString.Split (new char[] { ',' }, 2);
		string topic = input [0].Substring (12).Replace("\"", "");
		string data = input [1].Split(new string[] { "data" }, StringSplitOptions.None)[1];
		data = data.Substring (4);
		data = data.Split('"')[0];
		messages [topic] = data;
    }

    //The Closed event handler
    private void OnClosed(IWebSocket sender, WebSocketClosedEventArgs args)
    {
   //Add code here to do something when the connection is closed locally or by the server
    }
#endif
}
