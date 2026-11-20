using System.Collections.Generic;
using Newtonsoft.Json.Linq;
using UnityEngine;

namespace Cobot
{
    public class TaskOverlay : MonoBehaviour
    {
        [SerializeField] WorldAnchor anchor;
        [SerializeField] float ringRadius = 0.032f;
        [SerializeField] float labelHeight = 0.075f;
        [SerializeField] float pokeRadius = 0.03f;
        [SerializeField] float pokeDwell = 0.6f;
        [SerializeField] float pokeCooldown = 1.5f;
        [SerializeField] Vector3 summaryPosition = new Vector3(0.05f, 0f, 0.42f);

        static readonly Color RobotColor = new Color(0.25f, 0.65f, 1f);
        static readonly Color HumanColor = new Color(1f, 0.62f, 0.15f);
        static readonly Color HumanOnlyColor = new Color(1f, 0.3f, 0.25f);
        static readonly Color DoneColor = new Color(0.4f, 0.85f, 0.45f);

        class Marker
        {
            public Transform root;
            public LineRenderer ring;
            public TextMesh label;
            public Transform labelT;
            public JObject task;
            public Vector3 basePos;
            public float pokeStart = -1f;
            public float flashUntil;
            public string flash;
        }

        readonly Dictionary<int, Marker> markers = new Dictionary<int, Marker>();
        readonly Dictionary<int, Vector3> positions = new Dictionary<int, Vector3>();
        JObject state;
        Transform head;
        Material lineMaterial;
        Font font;
        TextMesh summary;
        LineRenderer route;
        float lastPoke = -10f;

        public bool Visible { get; private set; } = true;

        public void Toggle()
        {
            Visible = !Visible;
            foreach (var m in markers.Values)
                m.root.gameObject.SetActive(Visible);
            if (summary != null)
                summary.gameObject.SetActive(Visible);
            if (route != null)
                route.gameObject.SetActive(Visible);
        }

        public string Mode => state != null ? (string)state["mode"] : null;

        public float? Alpha => state?["plan"] is JObject plan && plan["alpha"] != null ? (float)plan["alpha"] : (float?)null;

        void Start()
        {
            if (anchor == null)
                anchor = WorldAnchor.Instance;
            if (Camera.main != null)
                head = Camera.main.transform;
            lineMaterial = new Material(Shader.Find("Sprites/Default"));
            font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
            summary = MakeText("TaskSummary", anchor.transform, 0.0022f, 70);
            summary.transform.localPosition = BaseFrame.ToUnity(summaryPosition.x, summaryPosition.y, summaryPosition.z);
            var routeGo = new GameObject("HumanRoute");
            routeGo.transform.SetParent(anchor.transform, false);
            route = routeGo.AddComponent<LineRenderer>();
            route.useWorldSpace = false;
            route.sharedMaterial = lineMaterial;
            route.widthMultiplier = 0.006f;
            route.numCornerVertices = 3;
            route.numCapVertices = 3;
            route.startColor = new Color(HumanColor.r, HumanColor.g, HumanColor.b, 0.25f);
            route.endColor = new Color(HumanColor.r, HumanColor.g, HumanColor.b, 0.9f);
            route.positionCount = 0;
            CobotBridge.Instance.Subscribe(Topics.TaskState, OnTaskState);
            CobotBridge.Instance.Subscribe(Topics.Objects, OnObjects);
        }

        void OnDestroy()
        {
            if (CobotBridge.Instance == null)
                return;
            CobotBridge.Instance.Unsubscribe(Topics.TaskState, OnTaskState);
            CobotBridge.Instance.Unsubscribe(Topics.Objects, OnObjects);
        }

        void OnObjects(JObject msg)
        {
            foreach (var o in (JArray)msg["objects"])
            {
                var p = (JArray)o["pos"];
                positions[(int)o["tag_id"]] = new Vector3((float)p[0], (float)p[1], (float)p[2]);
            }
        }

        void OnTaskState(JObject msg)
        {
            state = msg;
            foreach (var t in (JArray)msg["objects"])
            {
                var tag = (int)t["tag_id"];
                if (!markers.TryGetValue(tag, out var m))
                {
                    m = MakeMarker(tag);
                    markers[tag] = m;
                }
                m.task = (JObject)t;
            }
            UpdateSummary();
            UpdateRoute();
        }

        void UpdateRoute()
        {
            if (route == null)
                return;
            var pts = state["human_route"] as JArray;
            if (pts == null || pts.Count < 2)
            {
                route.positionCount = 0;
                return;
            }
            route.positionCount = pts.Count;
            for (var i = 0; i < pts.Count; i++)
                route.SetPosition(i, BaseFrame.ToUnity((float)pts[i][0], (float)pts[i][1], 0.004f));
        }

        Marker MakeMarker(int tag)
        {
            var m = new Marker();
            m.root = new GameObject($"task_{tag}").transform;
            m.root.SetParent(anchor.transform, false);
            m.root.gameObject.SetActive(Visible);

            var ringGo = new GameObject("ring");
            ringGo.transform.SetParent(m.root, false);
            m.ring = ringGo.AddComponent<LineRenderer>();
            m.ring.useWorldSpace = false;
            m.ring.loop = true;
            m.ring.positionCount = 48;
            m.ring.sharedMaterial = lineMaterial;
            m.ring.numCapVertices = 2;
            for (var i = 0; i < 48; i++)
            {
                var a = i / 48f * Mathf.PI * 2f;
                m.ring.SetPosition(i, new Vector3(Mathf.Cos(a) * ringRadius, 0f, Mathf.Sin(a) * ringRadius));
            }

            m.label = MakeText("label", m.root, 0.0016f, 64);
            m.labelT = m.label.transform;
            return m;
        }

        TextMesh MakeText(string name, Transform parent, float size, int fontSize)
        {
            var go = new GameObject(name);
            go.transform.SetParent(parent, false);
            var tm = go.AddComponent<TextMesh>();
            tm.font = font;
            go.GetComponent<MeshRenderer>().sharedMaterial = font.material;
            tm.characterSize = size;
            tm.fontSize = fontSize;
            tm.anchor = TextAnchor.LowerCenter;
            tm.alignment = TextAlignment.Center;
            tm.color = Color.white;
            return tm;
        }

        void Update()
        {
            if (state == null || anchor == null)
                return;
            var activeTag = state["active_tag_id"]?.Type == JTokenType.Integer ? (int)state["active_tag_id"] : -1;
            var nextTag = state["human_next_tag_id"]?.Type == JTokenType.Integer ? (int)state["human_next_tag_id"] : -1;
            var pulse = 0.5f + 0.5f * Mathf.Sin(Time.time * 6f);

            foreach (var kv in markers)
            {
                var tag = kv.Key;
                var m = kv.Value;
                if (m.task == null || !positions.TryGetValue(tag, out var p))
                    continue;
                m.basePos = p;
                var status = (string)m.task["status"];
                var owner = (string)m.task["owner"];
                var robotOk = (bool)m.task["robot_ok"];
                var robotReason = (string)m.task["robot_reason"];
                var order = (int)m.task["order"];
                var locked = m.task["locked"]?.Type == JTokenType.String;

                var show = status != "done" || Time.time < m.flashUntil;
                m.ring.enabled = show;
                m.label.gameObject.SetActive(show);
                if (!show)
                    continue;

                m.ring.transform.localPosition = BaseFrame.ToUnity(p.x, p.y, 0.002f);
                m.labelT.localPosition = BaseFrame.ToUnity(p.x, p.y, p.z + labelHeight - 0.015f);
                if (head != null)
                    m.labelT.rotation = Quaternion.LookRotation(m.labelT.position - head.position, Vector3.up);

                Color color;
                string text;
                var width = 0.004f;
                if (status == "robot_active" || tag == activeTag)
                {
                    color = RobotColor;
                    text = "robot: now";
                    width = 0.004f + 0.004f * pulse;
                }
                else if (status == "human_active")
                {
                    color = HumanColor;
                    text = "you: carrying";
                }
                else if (owner == "robot")
                {
                    color = RobotColor;
                    text = locked && order == 0 ? "robot: next" : $"robot {order + 1}";
                }
                else if (owner == "human")
                {
                    color = robotOk ? HumanColor : HumanOnlyColor;
                    var wait = m.task["est_start"] != null ? (float)m.task["est_start"] : 0f;
                    text = tag == nextTag ? (wait > 1f ? $"you: NEXT in {wait:0} s" : "you: NEXT") : $"you {order + 1}";
                    if (!robotOk)
                        text += "\nrobot can't: " + Pretty(robotReason);
                    if (tag == nextTag)
                        width = 0.005f + 0.005f * pulse;
                }
                else
                {
                    color = Color.gray;
                    text = "";
                }
                if (locked)
                    text += "\n(set by you)";
                if (Time.time < m.flashUntil)
                {
                    text = m.flash;
                    color = status == "done" ? DoneColor : HumanOnlyColor;
                }
                m.ring.startColor = m.ring.endColor = color;
                m.ring.widthMultiplier = width;
                m.label.text = text;
                m.label.color = color;
            }
            Poke();
        }

        void Poke()
        {
            if (Time.time - lastPoke < pokeCooldown)
                return;
            foreach (var isLeft in new[] { false, true })
            {
                if (!HandJoints.TryGetPosition(isLeft, OVRSkeleton.BoneId.XRHand_IndexTip, out var tip))
                    continue;
                foreach (var kv in markers)
                {
                    var m = kv.Value;
                    if (!m.label.gameObject.activeInHierarchy || m.task == null)
                        continue;
                    var near = Vector3.Distance(tip, m.labelT.position) < pokeRadius;
                    if (!near)
                    {
                        m.pokeStart = -1f;
                        continue;
                    }
                    if (m.pokeStart < 0f)
                        m.pokeStart = Time.time;
                    else if (Time.time - m.pokeStart > pokeDwell)
                    {
                        m.pokeStart = -1f;
                        lastPoke = Time.time;
                        Reassign(kv.Key, m);
                        return;
                    }
                }
            }
        }

        void Reassign(int tag, Marker m)
        {
            var owner = (string)m.task["owner"];
            var status = (string)m.task["status"];
            if (status != "pending")
                return;
            if (!(bool)m.task["robot_ok"])
            {
                m.flash = "robot can't:\n" + Pretty((string)m.task["robot_reason"]);
                m.flashUntil = Time.time + 1.5f;
                return;
            }
            var robotNext = owner == "robot" && (int)m.task["order"] == 0 &&
                            m.task["locked"]?.Type == JTokenType.String && (string)m.task["locked"] == "robot";
            var msg = new JObject
            {
                ["type"] = "OperatorCommand",
                ["tag_id"] = tag,
                ["stamp"] = CobotBridge.Now(),
            };
            if (robotNext)
            {
                msg["kind"] = "assign";
                msg["owner"] = "human";
                m.flash = "-> you";
            }
            else
            {
                msg["kind"] = "select_object";
                m.flash = "-> robot next";
            }
            m.flashUntil = Time.time + 0.8f;
            CobotBridge.Instance.Publish(Topics.OperatorCommand, msg);
        }

        void UpdateSummary()
        {
            if (summary == null || state == null)
                return;
            var mode = (string)state["mode"];
            var plan = state["plan"] as JObject;
            var msg = (string)state["message"];
            var human = 0;
            var robot = 0;
            var left = 0;
            foreach (var t in (JArray)state["objects"])
            {
                if ((string)t["status"] == "done")
                    continue;
                left++;
                if ((string)t["owner"] == "human")
                    human++;
                else if ((string)t["owner"] == "robot")
                    robot++;
            }
            var line = mode == "paused" ? "PAUSED, press Go on the wrist" : mode == "teleop" ? "TELEOP"
                : mode == "resetting" ? "RESETTING, arm going home" : "";
            var trial = state["trial"]?.Type == JTokenType.Integer ? (int)state["trial"] : 0;
            if (trial > 0)
                line = $"trial {trial}" + (line.Length > 0 ? " · " + line : "");
            if (plan != null && plan.Count > 0 && left > 0)
            {
                line += (line.Length > 0 ? "\n" : "") +
                        $"you {human} · robot {robot} · done in ~{(float)plan["makespan"]:0} s · mix {(float)plan["alpha"]:0.0}";
            }
            else if (left == 0)
                line = "all sorted";
            if (!string.IsNullOrEmpty(msg) && msg != "mode auto")
                line += "\n" + msg;
            summary.text = line;
            if (head != null)
                summary.transform.rotation = Quaternion.LookRotation(summary.transform.position - head.position, Vector3.up);
        }

        void LateUpdate()
        {
            if (summary != null && head != null)
                summary.transform.rotation = Quaternion.LookRotation(summary.transform.position - head.position, Vector3.up);
        }

        static string Pretty(string reason) =>
            string.IsNullOrEmpty(reason) ? "" : reason.Replace("_", " ").Replace(",", ", ");
    }
}
