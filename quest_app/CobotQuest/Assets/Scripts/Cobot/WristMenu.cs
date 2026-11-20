using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;

namespace Cobot
{
    public class WristMenu : MonoBehaviour
    {
        [Serializable]
        public class Button
        {
            public string id;
            public string label;
            public string icon;
            public Color color = new Color(0.25f, 0.55f, 0.95f);
            public string group = "main";
            public UnityEvent onPress = new UnityEvent();
        }

        [SerializeField] List<Button> buttons = new List<Button>
        {
            new Button { id = "place_table", label = "Table", icon = "table", color = new Color(0.25f, 0.55f, 0.95f) },
            new Button { id = "reset_scene", label = "Reset", icon = "reset", color = new Color(0.95f, 0.55f, 0.2f) },
        };
        [SerializeField] bool menuOnLeftHand = true;
        [SerializeField] Material baseMaterial;
        [SerializeField] float discSize = 0.04f;
        [SerializeField] float buttonSpacing = 0.055f;
        [SerializeField] float labelHeight = 0.012f;
        [SerializeField] float hoverRadius = 0.045f;
        [SerializeField] float offsetTowardHead = 0.07f;
        [SerializeField] float lookThreshold = 0.75f;
        [SerializeField] float maxDistance = 0.6f;
        [SerializeField] float pressRadius = 0.018f;
        [SerializeField] float releaseRadius = 0.035f;
        [SerializeField] float cooldown = 1f;
        [SerializeField] float rowSpacing = 0.068f;
        [SerializeField] float popupSeconds = 6f;

        public const string SettingsId = "settings";
        bool popupOpen;
        float popupUntil;
        readonly Dictionary<string, bool> dimmed = new Dictionary<string, bool>();
        readonly Dictionary<string, TextMesh> labels = new Dictionary<string, TextMesh>();

        public event Action<string> Pressed;

        void OnDestroy()
        {
            foreach (var t in textures)
                Destroy(t);
        }

        Transform head;
        Transform root;
        static readonly Dictionary<string, string> DefaultIcons = new Dictionary<string, string>
        {
            { "place_table", "table" },
            { "reset_scene", "reset" },
            { "toggle_workspace", "zones" },
            { SettingsId, "gear" },
        };

        readonly List<(Transform t, Material m, Button b)> visuals = new List<(Transform, Material, Button)>();
        readonly List<Texture2D> textures = new List<Texture2D>();
        bool armed = true;
        float lastPress = -10f;
        float flashUntil;
        int flashIndex = -1;

        void Start()
        {
            if (Camera.main != null)
                head = Camera.main.transform;
            Build();
        }

        public bool HasButton(string id) => buttons.Exists(b => b.id == id);

        public void AddButton(string id, string label, Color color, string icon = null, string group = "main")
        {
            buttons.Add(new Button { id = id, label = label, icon = icon, color = color, group = group });
            Build();
        }

        public void SetDim(string id, bool dim)
        {
            if (dimmed.TryGetValue(id, out var d) && d == dim)
                return;
            dimmed[id] = dim;
            ApplyColors();
        }

        public void SetLabel(string id, string text)
        {
            if (labels.TryGetValue(id, out var tm) && tm.text != text)
                tm.text = text;
        }

        public bool PopupOpen => popupOpen;

        void ApplyColors()
        {
            for (var i = 0; i < visuals.Count; i++)
            {
                if (i == flashIndex)
                    continue;
                var dim = visuals[i].b.id != null && dimmed.TryGetValue(visuals[i].b.id, out var d) && d;
                visuals[i].m.color = dim ? new Color(0.55f, 0.55f, 0.55f, 0.55f) : Color.white;
            }
        }

        void Build()
        {
            if (root != null)
                Destroy(root.gameObject);
            foreach (var t in textures)
                Destroy(t);
            textures.Clear();
            visuals.Clear();
            labels.Clear();
            root = new GameObject("WristMenuVisual").transform;
            root.SetParent(transform, false);
            var font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
            var shader = Shader.Find("UI/Default");
            foreach (var b in buttons)
            {
                if (string.IsNullOrEmpty(b.icon) && b.id != null && DefaultIcons.TryGetValue(b.id, out var icon))
                    b.icon = icon;
                var hasIcon = !string.IsNullOrEmpty(b.icon);

                var holder = new GameObject(b.id).transform;
                holder.SetParent(root, false);

                var quad = GameObject.CreatePrimitive(PrimitiveType.Quad);
                Destroy(quad.GetComponent<Collider>());
                quad.name = "disc";
                quad.transform.SetParent(holder, false);
                quad.transform.localScale = Vector3.one * discSize;
                var tex = ButtonIcons.Make(hasIcon ? b.icon : null, b.color);
                textures.Add(tex);
                var mat = shader != null ? new Material(shader) : new Material(baseMaterial);
                mat.mainTexture = tex;
                mat.color = Color.white;
                quad.GetComponent<MeshRenderer>().sharedMaterial = mat;

                var label = new GameObject("label");
                label.transform.SetParent(holder, false);
                var tm = label.AddComponent<TextMesh>();
                tm.font = font;
                tm.GetComponent<MeshRenderer>().sharedMaterial = font.material;
                tm.text = b.label;
                tm.fontSize = 96;
                tm.characterSize = 1f;
                tm.anchor = hasIcon ? TextAnchor.UpperCenter : TextAnchor.MiddleCenter;
                tm.alignment = TextAlignment.Center;
                tm.color = Color.white;
                label.transform.localScale = Vector3.one * (labelHeight / (tm.fontSize * 0.1f));
                label.transform.localPosition = hasIcon
                    ? new Vector3(0f, -discSize * 0.58f, 0f)
                    : new Vector3(0f, 0f, -0.001f);
                visuals.Add((holder, mat, b));
                if (b.id != null)
                    labels[b.id] = tm;
            }
            root.gameObject.SetActive(false);
            ApplyColors();
        }

        bool Shown(int i) => visuals[i].b.group != "settings" || popupOpen;

        void Update()
        {
            if (head == null || visuals.Count == 0)
                return;
            if (!HandJoints.TryGetPosition(menuOnLeftHand, OVRSkeleton.BoneId.XRHand_Wrist, out var wrist) ||
                !HandJoints.TryGetPosition(menuOnLeftHand, OVRSkeleton.BoneId.XRHand_MiddleMetacarpal, out var knuckle))
            {
                root.gameObject.SetActive(false);
                return;
            }

            var toWrist = wrist - head.position;
            var visible = toWrist.magnitude < maxDistance && Vector3.Dot(head.forward, toWrist.normalized) > lookThreshold;
            root.gameObject.SetActive(visible);
            if (!visible)
            {
                armed = true;
                return;
            }

            var toHead = (head.position - wrist).normalized;
            var along = Vector3.ProjectOnPlane(knuckle - wrist, toHead).normalized;
            var center = wrist + toHead * offsetTowardHead;
            var up = Vector3.ProjectOnPlane(Vector3.up, toHead);
            if (up.sqrMagnitude < 1e-4f)
                up = Vector3.Cross(-toHead, along);
            root.SetPositionAndRotation(center, Quaternion.LookRotation(-toHead, up));
            if (popupOpen && Time.unscaledTime > popupUntil)
                popupOpen = false;
            var rowUp = root.up;
            var mainCount = 0;
            var popupCount = 0;
            foreach (var v in visuals)
            {
                if (v.b.group == "settings")
                    popupCount++;
                else
                    mainCount++;
            }
            int mi = 0, pi = 0;
            for (var i = 0; i < visuals.Count; i++)
            {
                var popup = visuals[i].b.group == "settings";
                var k = popup ? pi++ : mi++;
                var n = popup ? popupCount : mainCount;
                var offset = (k - (n - 1) * 0.5f) * buttonSpacing;
                visuals[i].t.gameObject.SetActive(Shown(i));
                visuals[i].t.position = center + along * offset + (popup ? rowUp * rowSpacing : Vector3.zero);
                visuals[i].t.rotation = root.rotation;
            }

            if (Time.unscaledTime > flashUntil && flashIndex >= 0)
            {
                flashIndex = -1;
                ApplyColors();
            }

            if (!HandJoints.TryGetPosition(!menuOnLeftHand, OVRSkeleton.BoneId.XRHand_IndexTip, out var tip))
            {
                foreach (var v in visuals)
                    v.t.localScale = Vector3.one;
                return;
            }
            var nearest = -1;
            var nearestDist = float.MaxValue;
            for (var i = 0; i < visuals.Count; i++)
            {
                if (!Shown(i))
                    continue;
                var d = Vector3.Distance(tip, visuals[i].t.position);
                if (d < nearestDist)
                {
                    nearest = i;
                    nearestDist = d;
                }
            }
            for (var i = 0; i < visuals.Count; i++)
                visuals[i].t.localScale = Vector3.one * (i == nearest && nearestDist < hoverRadius ? 1.15f : 1f);
            if (nearestDist > releaseRadius)
                armed = true;
            if (armed && nearestDist < pressRadius && Time.unscaledTime - lastPress > cooldown)
            {
                armed = false;
                lastPress = Time.unscaledTime;
                Press(nearest);
            }
        }

        void Press(int index)
        {
            var b = visuals[index].b;
            visuals[index].m.color = new Color(0.6f, 0.6f, 0.6f, 1f);
            flashIndex = index;
            flashUntil = Time.unscaledTime + 0.25f;
            if (b.id == SettingsId)
            {
                popupOpen = !popupOpen;
                popupUntil = Time.unscaledTime + popupSeconds;
            }
            else if (b.group == "settings")
                popupOpen = false;
            b.onPress?.Invoke();
            Pressed?.Invoke(b.id);
        }

    }
}
