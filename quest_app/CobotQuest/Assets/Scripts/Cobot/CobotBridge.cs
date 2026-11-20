using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using UnityEngine;

namespace Cobot
{
    public class CobotBridge : MonoBehaviour
    {
        public static CobotBridge Instance { get; private set; }

        [SerializeField] string host = "192.168.1.10";
        [SerializeField] int port = 5560;
        [SerializeField] float reconnectSeconds = 2f;

        static readonly HashSet<string> LatestOnly = new HashSet<string>
        {
            Topics.SimScene, Topics.Objects, Topics.ArmState, Topics.Safety
        };

        readonly ConcurrentQueue<(string topic, JObject msg)> inbox = new ConcurrentQueue<(string, JObject)>();
        readonly ConcurrentQueue<string> outbox = new ConcurrentQueue<string>();
        readonly Dictionary<string, List<Action<JObject>>> handlers = new Dictionary<string, List<Action<JObject>>>();
        readonly Dictionary<string, JObject> coalesced = new Dictionary<string, JObject>();
        readonly List<(string topic, JObject msg)> ordered = new List<(string, JObject)>();

        Thread thread;
        volatile bool running;
        volatile bool connected;
        volatile string status = "not started";

        public bool Connected => connected;
        public string Status => status;

        public string Host
        {
            get => host;
            set => host = value;
        }

        void Awake()
        {
            Instance = this;
        }

        void OnEnable()
        {
            running = true;
            thread = new Thread(Run) { IsBackground = true, Name = "CobotBridge" };
            thread.Start();
        }

        void OnDisable()
        {
            running = false;
            thread?.Join(1000);
            thread = null;
        }

        public void Subscribe(string topic, Action<JObject> handler)
        {
            if (!handlers.TryGetValue(topic, out var list))
            {
                list = new List<Action<JObject>>();
                handlers[topic] = list;
            }
            list.Add(handler);
        }

        public void Unsubscribe(string topic, Action<JObject> handler)
        {
            if (handlers.TryGetValue(topic, out var list))
                list.Remove(handler);
        }

        public void Publish(string topic, JObject msg)
        {
            if (!connected)
                return;
            var packet = new JObject { ["topic"] = topic, ["msg"] = msg };
            outbox.Enqueue(packet.ToString(Formatting.None));
        }

        public static double Now() => DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() / 1000.0;

        void Update()
        {
            coalesced.Clear();
            ordered.Clear();
            while (inbox.TryDequeue(out var item))
            {
                if (LatestOnly.Contains(item.topic))
                    coalesced[item.topic] = item.msg;
                else
                    ordered.Add(item);
            }
            foreach (var kv in coalesced)
                ordered.Add((kv.Key, kv.Value));
            foreach (var (topic, msg) in ordered)
            {
                if (!handlers.TryGetValue(topic, out var list))
                    continue;
                foreach (var h in list.ToArray())
                {
                    try
                    {
                        h(msg);
                    }
                    catch (Exception e)
                    {
                        Debug.LogException(e);
                    }
                }
            }
        }

        void Run()
        {
            var buffer = new byte[1 << 16];
            var pending = new StringBuilder();
            while (running)
            {
                TcpClient client = null;
                try
                {
                    status = $"connecting to {host}:{port}";
                    client = new TcpClient { NoDelay = true };
                    var task = client.ConnectAsync(host, port);
                    if (!task.Wait(TimeSpan.FromSeconds(reconnectSeconds)) || !client.Connected)
                        throw new SocketException((int)SocketError.TimedOut);
                    var stream = client.GetStream();
                    while (outbox.TryDequeue(out _)) { }
                    pending.Clear();
                    connected = true;
                    status = $"connected to {host}:{port}";
                    while (running)
                    {
                        var idle = true;
                        while (outbox.TryDequeue(out var line))
                        {
                            var bytes = Encoding.UTF8.GetBytes(line + "\n");
                            stream.Write(bytes, 0, bytes.Length);
                            idle = false;
                        }
                        if (client.Client.Poll(0, SelectMode.SelectRead))
                        {
                            var n = stream.Read(buffer, 0, buffer.Length);
                            if (n == 0)
                                break;
                            pending.Append(Encoding.UTF8.GetString(buffer, 0, n));
                            ParseLines(pending);
                            idle = false;
                        }
                        if (idle)
                            Thread.Sleep(1);
                    }
                }
                catch (Exception e)
                {
                    status = $"{host}:{port} not reachable ({e.GetType().Name})";
                }
                finally
                {
                    connected = false;
                    client?.Close();
                }
                if (running)
                    Thread.Sleep(TimeSpan.FromSeconds(reconnectSeconds));
            }
            status = "stopped";
        }

        void ParseLines(StringBuilder pending)
        {
            var text = pending.ToString();
            var start = 0;
            int nl;
            while ((nl = text.IndexOf('\n', start)) >= 0)
            {
                var line = text.Substring(start, nl - start);
                start = nl + 1;
                if (line.Length == 0)
                    continue;
                try
                {
                    var packet = JObject.Parse(line);
                    inbox.Enqueue(((string)packet["topic"], (JObject)packet["msg"]));
                }
                catch (JsonException)
                {
                }
            }
            pending.Remove(0, start);
        }
    }
}
