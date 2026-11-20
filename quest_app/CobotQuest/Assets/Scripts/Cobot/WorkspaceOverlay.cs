using System;
using Newtonsoft.Json.Linq;
using UnityEngine;

namespace Cobot
{
    public class WorkspaceOverlay : MonoBehaviour
    {
        [SerializeField] WorldAnchor anchor;
        [SerializeField] bool visible = true;
        [SerializeField] float lift = 0.001f;

        GameObject overlay;
        Material material;
        Texture2D texture;
        string version;

        public bool Visible
        {
            get => visible;
            set
            {
                visible = value;
                if (overlay != null)
                    overlay.SetActive(visible);
            }
        }

        public void Toggle() => Visible = !Visible;

        void Start()
        {
            if (anchor == null)
                anchor = WorldAnchor.Instance;
            CobotBridge.Instance.Subscribe(Topics.ArmWorkspace, OnWorkspace);
        }

        void OnDestroy()
        {
            if (CobotBridge.Instance != null)
                CobotBridge.Instance.Unsubscribe(Topics.ArmWorkspace, OnWorkspace);
            if (texture != null)
                Destroy(texture);
        }

        void OnWorkspace(JObject msg)
        {
            var v = (string)msg["version"];
            if (v == version)
                return;
            version = v;
            Build(msg);
        }

        void Build(JObject msg)
        {
            if (overlay != null)
                Destroy(overlay);
            if (texture != null)
                Destroy(texture);

            texture = new Texture2D(2, 2) { wrapMode = TextureWrapMode.Clamp, filterMode = FilterMode.Bilinear };
            texture.LoadImage(Convert.FromBase64String((string)msg["image"]));

            var b = (JArray)msg["bounds"];
            float x0 = (float)b[0], x1 = (float)b[1], y0 = (float)b[2], y1 = (float)b[3];
            var z = (float)msg["z"] + lift;
            var mesh = new Mesh { name = "workspace" };
            mesh.vertices = new[]
            {
                BaseFrame.ToUnity(x0, y0, z),
                BaseFrame.ToUnity(x1, y0, z),
                BaseFrame.ToUnity(x1, y1, z),
                BaseFrame.ToUnity(x0, y1, z),
            };
            mesh.uv = new[] { new Vector2(0, 0), new Vector2(1, 0), new Vector2(1, 1), new Vector2(0, 1) };
            mesh.triangles = new[] { 0, 1, 2, 0, 2, 3 };
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();

            overlay = new GameObject("WorkspaceOverlay");
            overlay.transform.SetParent(anchor.transform, false);
            overlay.AddComponent<MeshFilter>().sharedMesh = mesh;
            material = new Material(Shader.Find("UI/Default")) { mainTexture = texture };
            overlay.AddComponent<MeshRenderer>().sharedMaterial = material;
            overlay.SetActive(visible);

            foreach (var zone in (JArray)msg["zones"])
                Debug.Log($"Workspace zone {(string)zone["name"]}: {(float)zone["min_reach"]:0.00}-{(float)zone["max_reach"]:0.00} m");
        }
    }
}
