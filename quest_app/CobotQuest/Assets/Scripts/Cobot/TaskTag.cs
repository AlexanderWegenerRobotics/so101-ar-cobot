using UnityEngine;

namespace Cobot
{
    public class TaskTag
    {
        public const float Width = 0.085f;
        public const float Height = 0.038f;

        static Texture2D rounded;

        public readonly Transform root;
        readonly Transform card;
        readonly Material borderMat;
        readonly Material fillMat;
        readonly Material barMat;
        readonly Transform bar;
        readonly TextMesh title;
        readonly TextMesh sub;

        public TaskTag(Transform parent, Font font)
        {
            root = new GameObject("tag").transform;
            root.SetParent(parent, false);
            card = new GameObject("card").transform;
            card.SetParent(root, false);
            var shader = Shader.Find("Sprites/Default");
            var tex = Rounded();
            borderMat = new Material(shader) { mainTexture = tex, renderQueue = 3000 };
            fillMat = new Material(shader) { mainTexture = tex, renderQueue = 3001, color = new Color(0.07f, 0.08f, 0.1f, 0.9f) };
            barMat = new Material(shader) { renderQueue = 3002 };
            Quad("border", borderMat, new Vector3(0f, 0f, 0.0008f), new Vector3(Width + 0.004f, Height + 0.004f, 1f));
            Quad("fill", fillMat, Vector3.zero, new Vector3(Width, Height, 1f));
            bar = Quad("progress", barMat, new Vector3(0f, -Height * 0.5f + 0.0035f, -0.0004f), new Vector3(0f, 0.003f, 1f));
            var textMat = new Material(font.material) { renderQueue = 3003 };
            title = Text("title", font, textMat, new Vector3(0f, 0.0055f, -0.0008f), 0.0115f, FontStyle.Bold);
            sub = Text("sub", font, textMat, new Vector3(0f, -0.0085f, -0.0008f), 0.0068f, FontStyle.Normal);
            sub.color = new Color(0.82f, 0.84f, 0.88f);
        }

        Transform Quad(string name, Material mat, Vector3 pos, Vector3 scale)
        {
            var go = GameObject.CreatePrimitive(PrimitiveType.Quad);
            Object.Destroy(go.GetComponent<Collider>());
            go.name = name;
            go.transform.SetParent(card, false);
            go.transform.localPosition = pos;
            go.transform.localScale = scale;
            go.GetComponent<MeshRenderer>().sharedMaterial = mat;
            return go.transform;
        }

        TextMesh Text(string name, Font font, Material mat, Vector3 pos, float height, FontStyle style)
        {
            var go = new GameObject(name);
            go.transform.SetParent(card, false);
            go.transform.localPosition = pos;
            var tm = go.AddComponent<TextMesh>();
            tm.font = font;
            tm.fontSize = 96;
            tm.fontStyle = style;
            tm.characterSize = 1f;
            tm.anchor = TextAnchor.MiddleCenter;
            tm.alignment = TextAlignment.Center;
            tm.color = Color.white;
            go.GetComponent<MeshRenderer>().sharedMaterial = mat;
            go.transform.localScale = Vector3.one * (height / (tm.fontSize * 0.1f));
            return tm;
        }

        public void Set(string titleText, string subText, Color color, bool emphasize, float progress, bool hover)
        {
            title.text = titleText;
            sub.text = subText;
            var pulse = emphasize ? 0.75f + 0.25f * Mathf.Sin(Time.time * 6f) : 1f;
            borderMat.color = new Color(color.r * pulse, color.g * pulse, color.b * pulse, 1f);
            title.color = Color.Lerp(Color.white, color, 0.35f);
            barMat.color = color;
            var w = Width * Mathf.Clamp01(progress);
            bar.localScale = new Vector3(w, 0.003f, 1f);
            bar.localPosition = new Vector3(-Width * 0.5f + w * 0.5f, bar.localPosition.y, bar.localPosition.z);
            card.localScale = Vector3.one * (hover ? 1.1f : 1f);
        }

        public void Face(Vector3 position, Transform head)
        {
            root.position = position;
            if (head != null)
                root.rotation = Quaternion.LookRotation(position - head.position, Vector3.up);
        }

        public bool Touch(Vector3 world, out float depth)
        {
            var local = Quaternion.Inverse(root.rotation) * (world - root.position);
            depth = local.z;
            return Mathf.Abs(local.x) < Width * 0.5f + 0.008f && Mathf.Abs(local.y) < Height * 0.5f + 0.008f;
        }

        public void SetActive(bool active) => root.gameObject.SetActive(active);

        static Texture2D Rounded()
        {
            if (rounded != null)
                return rounded;
            const int w = 170, h = 76, r = 16;
            rounded = new Texture2D(w, h, TextureFormat.RGBA32, false) { wrapMode = TextureWrapMode.Clamp };
            for (var y = 0; y < h; y++)
            for (var x = 0; x < w; x++)
            {
                var dx = Mathf.Max(r - x - 0.5f, x + 0.5f - (w - r), 0f);
                var dy = Mathf.Max(r - y - 0.5f, y + 0.5f - (h - r), 0f);
                var a = Mathf.Clamp01(r - Mathf.Sqrt(dx * dx + dy * dy) + 0.5f);
                rounded.SetPixel(x, y, new Color(1f, 1f, 1f, a));
            }
            rounded.Apply();
            return rounded;
        }
    }
}
