using System.Collections.Generic;
using UnityEngine;

namespace Cobot
{
    public static class UiKit
    {
        public static readonly Color Robot = new Color(0.25f, 0.65f, 1f);
        public static readonly Color Human = new Color(1f, 0.62f, 0.15f);
        public static readonly Color Danger = new Color(1f, 0.3f, 0.25f);
        public static readonly Color Good = new Color(0.3f, 0.85f, 0.45f);
        public static readonly Color Warn = new Color(1f, 0.78f, 0.2f);
        public static readonly Color Muted = new Color(0.62f, 0.65f, 0.7f);
        public static readonly Color Panel = new Color(0.06f, 0.07f, 0.09f, 0.92f);
        public static readonly Color Surface = new Color(0.14f, 0.15f, 0.18f, 0.95f);

        static readonly Dictionary<(int, int, int), Texture2D> textures = new Dictionary<(int, int, int), Texture2D>();
        static Font font;
        static Material textMaterial;
        static Shader sprite;

        public static Font Font => font != null ? font : font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");

        public static Material TextMaterial =>
            textMaterial != null ? textMaterial : textMaterial = new Material(Font.material) { renderQueue = 3010 };

        public static Texture2D Rounded(float width, float height, float radius)
        {
            const float pxPerMeter = 1600f;
            var w = Mathf.Clamp(Mathf.RoundToInt(width * pxPerMeter), 8, 1024);
            var h = Mathf.Clamp(Mathf.RoundToInt(height * pxPerMeter), 8, 1024);
            var r = Mathf.Clamp(Mathf.RoundToInt(radius * pxPerMeter), 1, Mathf.Min(w, h) / 2);
            var key = (w, h, r);
            if (textures.TryGetValue(key, out var tex))
                return tex;
            tex = new Texture2D(w, h, TextureFormat.RGBA32, false) { wrapMode = TextureWrapMode.Clamp };
            var px = new Color32[w * h];
            for (var y = 0; y < h; y++)
            for (var x = 0; x < w; x++)
            {
                var dx = Mathf.Max(r - x - 0.5f, x + 0.5f - (w - r), 0f);
                var dy = Mathf.Max(r - y - 0.5f, y + 0.5f - (h - r), 0f);
                var a = Mathf.Clamp01(r - Mathf.Sqrt(dx * dx + dy * dy) + 0.5f);
                px[y * w + x] = new Color32(255, 255, 255, (byte)(a * 255));
            }
            tex.SetPixels32(px);
            tex.Apply();
            textures[key] = tex;
            return tex;
        }

        public static Material Box(Transform parent, string name, Vector2 center, Vector2 size, float radius, Color color,
            int queue, float z = 0f)
        {
            if (sprite == null)
                sprite = Shader.Find("Sprites/Default");
            var go = GameObject.CreatePrimitive(PrimitiveType.Quad);
            Object.Destroy(go.GetComponent<Collider>());
            go.name = name;
            go.transform.SetParent(parent, false);
            go.transform.localPosition = new Vector3(center.x, center.y, z);
            go.transform.localScale = new Vector3(size.x, size.y, 1f);
            var mat = new Material(sprite) { renderQueue = queue, color = color };
            if (radius > 0f)
                mat.mainTexture = Rounded(size.x, size.y, radius);
            go.GetComponent<MeshRenderer>().sharedMaterial = mat;
            return mat;
        }

        public static TextMesh Text(Transform parent, string name, Vector2 pos, float height, TextAnchor anchor,
            Color color, FontStyle style = FontStyle.Normal, string text = "")
        {
            var go = new GameObject(name);
            go.transform.SetParent(parent, false);
            go.transform.localPosition = new Vector3(pos.x, pos.y, -0.0015f);
            var tm = go.AddComponent<TextMesh>();
            tm.font = Font;
            tm.fontSize = 96;
            tm.fontStyle = style;
            tm.characterSize = 1f;
            tm.anchor = anchor;
            tm.alignment = anchor == TextAnchor.MiddleLeft ? TextAlignment.Left
                : anchor == TextAnchor.MiddleRight ? TextAlignment.Right : TextAlignment.Center;
            tm.color = color;
            tm.text = text;
            go.GetComponent<MeshRenderer>().sharedMaterial = TextMaterial;
            go.transform.localScale = Vector3.one * (height / (tm.fontSize * 0.1f));
            return tm;
        }
    }
}
