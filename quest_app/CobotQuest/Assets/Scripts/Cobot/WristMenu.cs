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
        [SerializeField] float offsetAboveHand = 0.075f;
        [SerializeField] float offsetAlongHand = -0.02f;
        [SerializeField] float showFacing = 0.55f;
        [SerializeField] float hideFacing = 0.35f;
        [SerializeField] float smoothing = 18f;
        [SerializeField] float faceHeadBlend = 0.5f;
        [SerializeField] float lockRadius = 0.16f;
        [SerializeField] float hoverDepth = 0.035f;
        [SerializeField] float pressDepth = 0.004f;
        [SerializeField] float rearmDepth = 0.02f;
        [SerializeField] float maxDistance = 0.6f;
        [SerializeField] float pressRadius = 0.018f;
        [SerializeField] float releaseRadius = 0.035f;
        [SerializeField] float cooldown = 1f;
        [SerializeField] float rowSpacing = 0.068f;
        [SerializeField] float popupSeconds = 6f;

        public const string SettingsId = "settings";
        bool shown;
        bool locked;
        float lastTipTime;
        bool smoothInit;
        Vector3 smoothPos;
        Quaternion smoothRot;
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
            var hasTip = HandJoints.TryGetPosition(!menuOnLeftHand, OVRSkeleton.BoneId.XRHand_IndexTip, out var tip);
            if (hasTip)
                lastTipTime = Time.unscaledTime;
            if (locked)
            {
                var away = !hasTip ? Time.unscaledTime - lastTipTime > 0.5f
                    : Vector3.Distance(tip, smoothPos) > lockRadius + 0.04f;
                if (away)
                    locked = false;
            }

            if (!locked)
            {
                if (!TryHandPose(out var target, out var targetRot, out var facing, out var close))
                {
                    root.gameObject.SetActive(false);
                    shown = false;
                    return;
                }
                shown = close && (shown ? facing > hideFacing : facing > showFacing);
                if (!shown)
                {
                    root.gameObject.SetActive(false);
                    armed = true;
                    smoothInit = false;
                    return;
                }
                if (!smoothInit)
                {
                    smoothPos = target;
                    smoothRot = targetRot;
                    smoothInit = true;
                }
                var blend = 1f - Mathf.Exp(-smoothing * Time.unscaledDeltaTime);
                smoothPos = Vector3.Lerp(smoothPos, target, blend);
                smoothRot = Quaternion.Slerp(smoothRot, targetRot, blend);
                if (hasTip && Vector3.Distance(tip, smoothPos) < lockRadius)
                    locked = true;
            }
            root.gameObject.SetActive(true);
            root.SetPositionAndRotation(smoothPos, smoothRot);
            var center = smoothPos;
            var along = root.right;
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

            if (!hasTip)
            {
                foreach (var v in visuals)
                    v.t.localScale = Vector3.one;
                return;
            }
            var inv = Quaternion.Inverse(root.rotation);
            var nearest = -1;
            var nearestLateral = float.MaxValue;
            var nearestDepth = 0f;
            for (var i = 0; i < visuals.Count; i++)
            {
                if (!Shown(i))
                    continue;
                var local = inv * (tip - visuals[i].t.position);
                var lateral = new Vector2(local.x, local.y).magnitude;
                if (lateral < nearestLateral)
                {
                    nearest = i;
                    nearestLateral = lateral;
                    nearestDepth = local.z;
                }
            }
            var radius = discSize * 0.5f;
            var hovering = nearest >= 0 && nearestLateral < radius * 1.3f && nearestDepth > -hoverDepth;
            for (var i = 0; i < visuals.Count; i++)
                visuals[i].t.localScale = Vector3.one * (i == nearest && hovering ? 1.15f : 1f);
            if (nearest < 0 || nearestLateral > radius * 1.6f || nearestDepth < -rearmDepth)
                armed = true;
            if (armed && hovering && nearestLateral < radius * 1.2f && nearestDepth > -pressDepth
                && Time.unscaledTime - lastPress > cooldown)
            {
                armed = false;
                lastPress = Time.unscaledTime;
                Press(nearest);
            }
        }

        bool TryHandPose(out Vector3 position, out Quaternion rotation, out float facing, out bool close)
        {
            position = default;
            rotation = Quaternion.identity;
            facing = 0f;
            close = false;
            if (!HandJoints.TryGetPosition(menuOnLeftHand, OVRSkeleton.BoneId.XRHand_Wrist, out var wrist) ||
                !HandJoints.TryGetPosition(menuOnLeftHand, OVRSkeleton.BoneId.XRHand_MiddleMetacarpal, out var knuckle) ||
                !HandJoints.TryGetPosition(menuOnLeftHand, OVRSkeleton.BoneId.XRHand_IndexMetacarpal, out var index) ||
                !HandJoints.TryGetPosition(menuOnLeftHand, OVRSkeleton.BoneId.XRHand_LittleMetacarpal, out var little))
                return false;
            var fingers = (knuckle - wrist).normalized;
            var thumbSide = Vector3.ProjectOnPlane(index - little, fingers).normalized;
            var back = Vector3.Cross(thumbSide, fingers) * (menuOnLeftHand ? -1f : 1f);
            var toHead = head.position - wrist;
            facing = Vector3.Dot(back, toHead.normalized);
            close = toHead.magnitude < maxDistance;
            var tilt = Vector3.Slerp(back, toHead.normalized, faceHeadBlend).normalized;
            position = wrist + back * offsetAboveHand + fingers * offsetAlongHand;
            rotation = Quaternion.LookRotation(-tilt, thumbSide);
            return true;
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
