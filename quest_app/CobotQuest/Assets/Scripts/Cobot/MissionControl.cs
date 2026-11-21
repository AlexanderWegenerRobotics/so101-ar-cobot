using System.Collections.Generic;
using Newtonsoft.Json.Linq;
using UnityEngine;

namespace Cobot
{
    public class MissionControl : MonoBehaviour
    {
        [SerializeField] WorldAnchor anchor;
        [SerializeField] Vector3 defaultPosition = new Vector3(0.3f, -0.62f, 0.6f);
        [SerializeField] Vector3 userPosition = new Vector3(0.95f, 0f, 0.75f);
        [SerializeField] float hoverDepth = 0.035f;
        [SerializeField] float pressDepth = 0.006f;
        [SerializeField] float rearmDepth = 0.02f;
        [SerializeField] float cooldown = 0.6f;
        [SerializeField] float pinchClose = 0.025f;
        [SerializeField] float pinchOpen = 0.045f;
        [SerializeField] float grabRadius = 0.06f;

        const float W = 0.46f;
        const float H = 0.36f;

        class Button
        {
            public string id;
            public Vector2 center;
            public Vector2 size;
            public Material bg;
            public TextMesh label;
            public Color color;
            public bool selected;
            public bool hover;
            public float flashUntil;
        }

        static readonly (string id, string label, float alpha)[] Mixes =
        {
            ("mix_fast", "FAST", 0f),
            ("mix_even", "EVEN", 0.5f),
            ("mix_robot", "ROBOT MAX", 1f),
        };

        Transform root;
        Transform head;
        Transform handle;
        Material handleMat;
        readonly List<Button> buttons = new List<Button>();
        TextMesh status, trial, elapsed, caption, robotCount, robotSub, humanCount, humanSub, safetyText, eventText;
        Material statusBg, progressFill, safetyDot;
        Transform progressFillT;

        JObject state;
        JObject safety;
        float stateTime = -10f;
        float safetyTime = -10f;
        bool armed = true;
        float lastPress;
        bool grabbing;
        bool grabLeft;
        Vector3 grabOffset;

        void Start()
        {
            if (anchor == null)
                anchor = WorldAnchor.Instance;
            if (Camera.main != null)
                head = Camera.main.transform;
            Build();
            PlaceDefault();
            CobotBridge.Instance.Subscribe(Topics.TaskState, OnTaskState);
            CobotBridge.Instance.Subscribe(Topics.Safety, OnSafety);
        }

        void OnDestroy()
        {
            if (CobotBridge.Instance == null)
                return;
            CobotBridge.Instance.Unsubscribe(Topics.TaskState, OnTaskState);
            CobotBridge.Instance.Unsubscribe(Topics.Safety, OnSafety);
        }

        void OnTaskState(JObject msg)
        {
            state = msg;
            stateTime = Time.time;
        }

        void OnSafety(JObject msg)
        {
            safety = msg;
            safetyTime = Time.time;
        }

        public void PlaceDefault()
        {
            var pos = BaseFrame.ToUnity(defaultPosition.x, defaultPosition.y, defaultPosition.z);
            var user = BaseFrame.ToUnity(userPosition.x, userPosition.y, userPosition.z);
            root.localPosition = pos;
            root.localRotation = Quaternion.LookRotation(pos - user, Vector3.up);
        }

        void Build()
        {
            root = new GameObject("MissionControl").transform;
            root.SetParent(anchor.transform, false);
            UiKit.Box(root, "panel", Vector2.zero, new Vector2(W, H), 0.022f, UiKit.Panel, 3000, 0.002f);

            handle = new GameObject("handle").transform;
            handle.SetParent(root, false);
            handle.localPosition = new Vector3(0f, H * 0.5f + 0.022f, 0f);
            handleMat = UiKit.Box(handle, "bar", Vector2.zero, new Vector2(0.12f, 0.012f), 0.006f, UiKit.Muted, 3001);

            var left = -W * 0.5f + 0.022f;
            var right = W * 0.5f - 0.022f;
            UiKit.Text(root, "title", new Vector2(left, 0.152f), 0.012f, TextAnchor.MiddleLeft, UiKit.Muted,
                FontStyle.Bold, "MISSION CONTROL");
            statusBg = UiKit.Box(root, "statusChip", new Vector2(right - 0.055f, 0.152f), new Vector2(0.11f, 0.028f), 0.014f,
                UiKit.Surface, 3001);
            status = UiKit.Text(root, "status", new Vector2(right - 0.055f, 0.152f), 0.012f, TextAnchor.MiddleCenter,
                Color.white, FontStyle.Bold, "OFFLINE");

            trial = UiKit.Text(root, "trial", new Vector2(left, 0.11f), 0.028f, TextAnchor.MiddleLeft, Color.white,
                FontStyle.Bold, "TRIAL -");
            elapsed = UiKit.Text(root, "elapsed", new Vector2(right, 0.11f), 0.028f, TextAnchor.MiddleRight, Color.white,
                FontStyle.Bold, "00:00");

            var barWidth = W - 0.044f;
            UiKit.Box(root, "progressBg", new Vector2(0f, 0.071f), new Vector2(barWidth, 0.012f), 0.006f, UiKit.Surface, 3001);
            progressFill = UiKit.Box(root, "progressFill", new Vector2(0f, 0.071f), new Vector2(barWidth, 0.012f), 0.006f,
                UiKit.Good, 3002, -0.0005f);
            progressFillT = root.Find("progressFill");
            caption = UiKit.Text(root, "caption", new Vector2(left, 0.049f), 0.011f, TextAnchor.MiddleLeft, UiKit.Muted);

            Block("robot", -W * 0.25f, "ROBOT", UiKit.Robot, out robotCount, out robotSub);
            Block("human", W * 0.25f, "YOU", UiKit.Human, out humanCount, out humanSub);

            UiKit.Text(root, "splitLabel", new Vector2(left, -0.04f), 0.009f, TextAnchor.MiddleLeft, UiKit.Muted,
                FontStyle.Bold, "WHO DOES WHAT");
            for (var i = 0; i < Mixes.Length; i++)
                AddButton(Mixes[i].id, Mixes[i].label, new Vector2(-0.145f + i * 0.145f, -0.066f), new Vector2(0.135f, 0.03f),
                    new Color(0.65f, 0.45f, 0.95f));

            safetyDot = UiKit.Box(root, "safetyDot", new Vector2(left + 0.006f, -0.104f), new Vector2(0.012f, 0.012f), 0.006f,
                UiKit.Muted, 3002);
            safetyText = UiKit.Text(root, "safety", new Vector2(left + 0.02f, -0.104f), 0.012f, TextAnchor.MiddleLeft,
                Color.white, FontStyle.Bold, "NO DATA");
            eventText = UiKit.Text(root, "event", new Vector2(left, -0.125f), 0.0095f, TextAnchor.MiddleLeft, UiKit.Muted);

            var y = -0.155f;
            AddButton("go", "GO", new Vector2(-0.145f, y), new Vector2(0.135f, 0.036f), UiKit.Good);
            AddButton("reset", "RESET TRIAL", new Vector2(0f, y), new Vector2(0.135f, 0.036f), UiKit.Warn);
            AddButton("shuffle", "NEW LAYOUT", new Vector2(0.145f, y), new Vector2(0.135f, 0.036f), UiKit.Robot);
        }

        void Block(string name, float x, string title, Color color, out TextMesh count, out TextMesh sub)
        {
            UiKit.Box(root, name + "Card", new Vector2(x, 0.0f), new Vector2(W * 0.5f - 0.03f, 0.068f), 0.012f, UiKit.Surface,
                3001);
            UiKit.Box(root, name + "Accent", new Vector2(x - (W * 0.5f - 0.03f) * 0.5f + 0.003f, 0.0f),
                new Vector2(0.006f, 0.068f), 0.003f, color, 3002, -0.0005f);
            UiKit.Text(root, name + "Title", new Vector2(x - 0.07f, 0.02f), 0.01f, TextAnchor.MiddleLeft, color, FontStyle.Bold,
                title);
            count = UiKit.Text(root, name + "Count", new Vector2(x - 0.07f, -0.006f), 0.03f, TextAnchor.MiddleLeft, Color.white,
                FontStyle.Bold, "0");
            sub = UiKit.Text(root, name + "Sub", new Vector2(x + 0.085f, -0.006f), 0.0095f, TextAnchor.MiddleRight, UiKit.Muted);
        }

        void AddButton(string id, string label, Vector2 center, Vector2 size, Color color)
        {
            var b = new Button { id = id, center = center, size = size, color = color };
            b.bg = UiKit.Box(root, "btn_" + id, center, size, Mathf.Min(size.y * 0.5f, 0.016f), UiKit.Surface, 3003);
            b.label = UiKit.Text(root, "lbl_" + id, center, 0.0115f, TextAnchor.MiddleCenter, Color.white, FontStyle.Bold, label);
            buttons.Add(b);
        }

        void Update()
        {
            if (root == null)
                return;
            Grab();
            Refresh();
            Touch();
            foreach (var b in buttons)
            {
                var flash = Time.time < b.flashUntil;
                var c = b.selected ? b.color : UiKit.Surface;
                if (b.hover && !b.selected)
                    c = Color.Lerp(UiKit.Surface, b.color, 0.35f);
                if (flash)
                    c = Color.Lerp(b.color, Color.white, 0.5f);
                b.bg.color = c;
                b.label.color = b.selected ? new Color(0.05f, 0.05f, 0.07f) : Color.white;
            }
        }

        void Refresh()
        {
            var online = Time.time - stateTime < 2f;
            var mode = online ? (string)state["mode"] : null;
            var msg = online ? (string)state["message"] ?? "" : "";
            var stats = online ? state["trial_stats"] as JObject : null;
            var objects = online ? state["objects"] as JArray : null;
            var plan = online ? state["plan"] as JObject : null;

            var done = 0;
            var total = 0;
            var robotLeft = 0;
            var humanLeft = 0;
            if (objects != null)
                foreach (var o in objects)
                {
                    total++;
                    if ((string)o["status"] == "done")
                    {
                        done++;
                        continue;
                    }
                    if ((string)o["owner"] == "robot")
                        robotLeft++;
                    else if ((string)o["owner"] == "human")
                        humanLeft++;
                }
            var finished = online && total > 0 && done == total;

            string statusText;
            Color statusColor;
            if (!online)
            {
                statusText = "OFFLINE";
                statusColor = UiKit.Surface;
            }
            else if (mode == "resetting")
            {
                statusText = "RESETTING";
                statusColor = UiKit.Robot;
            }
            else if (finished)
            {
                statusText = "DONE";
                statusColor = UiKit.Good;
            }
            else if (mode == "auto")
            {
                statusText = "RUNNING";
                statusColor = UiKit.Good;
            }
            else if (mode == "teleop")
            {
                statusText = "TELEOP";
                statusColor = new Color(0.65f, 0.45f, 0.95f);
            }
            else
            {
                statusText = "PAUSED";
                statusColor = UiKit.Warn;
            }
            status.text = statusText;
            statusBg.color = statusColor;
            status.color = online ? new Color(0.05f, 0.05f, 0.07f) : UiKit.Muted;

            var trialNo = online && state["trial"]?.Type == JTokenType.Integer ? (int)state["trial"] : 0;
            trial.text = trialNo > 0 ? $"TRIAL {trialNo}" : "TRIAL -";
            var layout = stats != null && stats["layout"] != null && stats["layout"].Type == JTokenType.Integer
                ? $"layout {(int)stats["layout"]}" : "default layout";
            var seconds = stats != null && stats["elapsed"] != null ? (float)stats["elapsed"] : 0f;
            if (stats != null && stats["running"] != null && (bool)stats["running"])
                seconds += Time.time - stateTime;
            elapsed.text = $"{(int)seconds / 60:00}:{(int)seconds % 60:00}";

            var frac = total > 0 ? (float)done / total : 0f;
            var barWidth = W - 0.044f;
            progressFillT.localScale = new Vector3(Mathf.Max(barWidth * frac, 0.0001f), 0.012f, 1f);
            progressFillT.localPosition = new Vector3(-barWidth * 0.5f + barWidth * frac * 0.5f, 0.071f, -0.0005f);
            var left = plan != null && plan["makespan"] != null && !finished ? (float)plan["makespan"] : -1f;
            caption.text = total == 0 ? "waiting for the task node"
                : $"{done} / {total} sorted" + (left > 0f && mode == "auto" ? $"  ·  about {left:0} s left" : "") + $"  ·  {layout}";

            var robotDone = stats != null && stats["robot_cubes"] != null ? (int)stats["robot_cubes"] : 0;
            var humanDone = stats != null && stats["human_cubes"] != null ? (int)stats["human_cubes"] : 0;
            robotCount.text = robotDone.ToString();
            humanCount.text = humanDone.ToString();
            robotSub.text = $"done\n{robotLeft} to go";
            humanSub.text = $"done\n{humanLeft} to go";

            var alpha = plan != null && plan["alpha"] != null ? (float)plan["alpha"] : -1f;
            foreach (var m in Mixes)
                Find(m.id).selected = Mathf.Abs(alpha - m.alpha) < 0.01f;

            var go = Find("go");
            var homing = online && (mode == "resetting" || (string)state["robot_step"] == "homing");
            if (!online)
            {
                go.label.text = "GO";
                go.selected = false;
            }
            else if (homing)
            {
                go.label.text = "HOMING";
                go.color = UiKit.Muted;
                go.selected = true;
            }
            else if (mode == "auto")
            {
                go.label.text = finished ? "DONE" : "STOP";
                go.color = finished ? UiKit.Muted : UiKit.Danger;
                go.selected = true;
            }
            else
            {
                go.label.text = "GO";
                go.color = UiKit.Good;
                go.selected = true;
            }

            var safetyOnline = Time.time - safetyTime < 1f;
            var zone = safetyOnline ? (string)safety["zone"] : null;
            var dist = safetyOnline && safety["min_distance"] != null && safety["min_distance"].Type != JTokenType.Null
                ? (float)safety["min_distance"] : -1f;
            var stops = stats != null && stats["stops"] != null ? (int)stats["stops"] : 0;
            var near = dist >= 0f ? $"hand {Mathf.Max(dist, 0f) * 100f:0} cm from the arm" : "no hand near the arm";
            if (!safetyOnline)
            {
                safetyText.text = "SAFETY: NO DATA";
                safetyDot.color = UiKit.Muted;
            }
            else if (zone == "stop")
            {
                safetyText.text = $"STOPPED  ·  {near}";
                safetyDot.color = UiKit.Danger;
            }
            else if (zone == "slow")
            {
                safetyText.text = $"SLOWING  ·  {near}";
                safetyDot.color = UiKit.Warn;
            }
            else
            {
                safetyText.text = $"CLEAR  ·  {near}";
                safetyDot.color = UiKit.Good;
            }
            if (stops > 0)
                safetyText.text += $"  ·  {stops} stop{(stops == 1 ? "" : "s")}";
            safetyText.color = safetyDot.color == UiKit.Muted ? UiKit.Muted : Color.white;
            eventText.text = msg.Length > 70 ? msg.Substring(0, 67) + "..." : msg;
        }

        Button Find(string id) => buttons.Find(b => b.id == id);

        void Touch()
        {
            var tips = new List<Vector3>();
            foreach (var isLeft in new[] { false, true })
            {
                if (grabbing && isLeft == grabLeft)
                    continue;
                if (HandJoints.TryGetPosition(isLeft, OVRSkeleton.BoneId.XRHand_IndexTip, out var tip))
                    tips.Add(tip);
            }
            var inv = Quaternion.Inverse(root.rotation);
            Button target = null;
            var depth = float.NegativeInfinity;
            foreach (var b in buttons)
                b.hover = false;
            foreach (var tip in tips)
            {
                var local = inv * (tip - root.position);
                foreach (var b in buttons)
                {
                    if (Mathf.Abs(local.x - b.center.x) > b.size.x * 0.5f + 0.004f ||
                        Mathf.Abs(local.y - b.center.y) > b.size.y * 0.5f + 0.004f || local.z < -hoverDepth)
                        continue;
                    b.hover = true;
                    if (local.z > depth)
                    {
                        depth = local.z;
                        target = b;
                    }
                }
            }
            if (target == null || depth < -rearmDepth)
            {
                armed = true;
                return;
            }
            if (!armed || depth < -pressDepth || Time.time - lastPress < cooldown)
                return;
            armed = false;
            lastPress = Time.time;
            target.flashUntil = Time.time + 0.2f;
            Press(target.id);
        }

        void Press(string id)
        {
            switch (id)
            {
                case "go":
                    var mode = state != null ? (string)state["mode"] : null;
                    if (mode != "resetting")
                        Send(mode == "auto" ? "pause" : "start");
                    break;
                case "reset":
                    Send("reset_scene", mode: "same_layout");
                    break;
                case "shuffle":
                    Send("reset_scene", mode: "new_layout");
                    break;
                default:
                    foreach (var m in Mixes)
                        if (m.id == id)
                            Send("set_alpha", m.alpha);
                    break;
            }
        }

        void Grab()
        {
            handleMat.color = grabbing ? Color.white : UiKit.Muted;
            var best = -1;
            var bestMid = Vector3.zero;
            for (var side = 0; side < 2; side++)
            {
                var isLeft = side == 1;
                if (!HandJoints.TryGetPosition(isLeft, OVRSkeleton.BoneId.XRHand_ThumbTip, out var thumb) ||
                    !HandJoints.TryGetPosition(isLeft, OVRSkeleton.BoneId.XRHand_IndexTip, out var index))
                {
                    if (grabbing && grabLeft == isLeft)
                        grabbing = false;
                    continue;
                }
                var gap = Vector3.Distance(thumb, index);
                var mid = (thumb + index) * 0.5f;
                if (grabbing && grabLeft == isLeft)
                {
                    if (gap > pinchOpen)
                    {
                        grabbing = false;
                        continue;
                    }
                    root.position = mid + grabOffset;
                    if (head != null)
                    {
                        var dir = root.position - head.position;
                        root.rotation = Quaternion.LookRotation(dir, Vector3.up);
                    }
                    return;
                }
                if (!grabbing && gap < pinchClose && Vector3.Distance(mid, handle.position) < grabRadius)
                {
                    best = side;
                    bestMid = mid;
                }
            }
            if (best >= 0)
            {
                grabbing = true;
                grabLeft = best == 1;
                grabOffset = root.position - bestMid;
            }
        }

        static void Send(string kind, float? value = null, string mode = null)
        {
            if (CobotBridge.Instance == null)
                return;
            var msg = new JObject
            {
                ["type"] = "OperatorCommand",
                ["kind"] = kind,
                ["stamp"] = CobotBridge.Now(),
            };
            if (value.HasValue)
                msg["value"] = value.Value;
            if (mode != null)
                msg["mode"] = mode;
            CobotBridge.Instance.Publish(Topics.OperatorCommand, msg);
        }
    }
}
