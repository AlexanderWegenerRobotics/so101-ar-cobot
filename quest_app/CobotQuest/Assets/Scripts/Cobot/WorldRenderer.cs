using System;
using System.Collections.Generic;
using System.IO;
using Newtonsoft.Json.Linq;
using UnityEngine;
using UnityEngine.Rendering;

namespace Cobot
{
    public class WorldRenderer : MonoBehaviour
    {
        const string MeshFolder = "SimScene/Meshes/";
        const string AssetVersionFile = "SimScene/asset_version";

        [SerializeField] WorldAnchor anchor;
        [SerializeField] Material baseMaterial;
        [SerializeField] bool showSimHands;
        [SerializeField] bool showMarkers = true;
        [SerializeField] bool flipMeshWinding;

        readonly Dictionary<string, Transform> bodies = new Dictionary<string, Transform>();
        readonly Dictionary<string, Mesh> meshes = new Dictionary<string, Mesh>();
        readonly Dictionary<string, Texture2D> textures = new Dictionary<string, Texture2D>();
        readonly Dictionary<string, List<GameObject>> byKind = new Dictionary<string, List<GameObject>>();
        Transform root;
        string bundledAssetVersion;

        public string SceneVersion { get; private set; }
        public bool AssetsMatch { get; private set; } = true;
        public double LastPoseTime { get; private set; }

        public bool ShowSimHands
        {
            get => showSimHands;
            set
            {
                showSimHands = value;
                ApplyVisibility();
            }
        }

        void Start()
        {
            if (anchor == null)
                anchor = WorldAnchor.Instance;
            var version = Resources.Load<TextAsset>(AssetVersionFile);
            bundledAssetVersion = version != null ? version.text.Trim() : null;
            CobotBridge.Instance.Subscribe(Topics.SimSceneInfo, OnSceneInfo);
            CobotBridge.Instance.Subscribe(Topics.SimScene, OnScenePoses);
        }

        void OnDestroy()
        {
            if (CobotBridge.Instance == null)
                return;
            CobotBridge.Instance.Unsubscribe(Topics.SimSceneInfo, OnSceneInfo);
            CobotBridge.Instance.Unsubscribe(Topics.SimScene, OnScenePoses);
        }

        void OnSceneInfo(JObject msg)
        {
            var version = (string)msg["version"];
            if (version == SceneVersion)
                return;
            var assetVersion = (string)msg["asset_version"];
            AssetsMatch = assetVersion == bundledAssetVersion;
            if (!AssetsMatch)
                Debug.LogWarning($"Sim meshes in the build ({bundledAssetVersion}) differ from the sim ({assetVersion}). " +
                                 "Run python -m sim.scene_export and rebuild.");
            Build(msg);
            SceneVersion = version;
        }

        void OnScenePoses(JObject msg)
        {
            if ((string)msg["version"] != SceneVersion)
                return;
            var names = (JArray)msg["names"];
            var poses = (JArray)msg["poses"];
            for (var i = 0; i < names.Count; i++)
            {
                if (!bodies.TryGetValue((string)names[i], out var t))
                    continue;
                var p = poses[i];
                t.localPosition = BaseFrame.ToUnity(p, 0);
                t.localRotation = BaseFrame.ToUnityRotation(p, 3);
            }
            LastPoseTime = CobotBridge.Now();
        }

        void Build(JObject info)
        {
            if (root != null)
                Destroy(root.gameObject);
            bodies.Clear();
            byKind.Clear();
            foreach (var tex in textures.Values)
                Destroy(tex);
            textures.Clear();

            root = new GameObject("SimWorld").transform;
            root.SetParent(anchor.transform, false);

            if (info["textures"] is JObject texs)
            {
                foreach (var kv in texs)
                {
                    var tex = new Texture2D(2, 2) { filterMode = FilterMode.Point, wrapMode = TextureWrapMode.Clamp };
                    tex.LoadImage(Convert.FromBase64String((string)kv.Value));
                    textures[kv.Key] = tex;
                }
            }

            foreach (var body in (JArray)info["bodies"])
            {
                var name = (string)body["name"];
                var kind = (string)body["kind"];
                var go = new GameObject(name);
                go.transform.SetParent(root, false);
                if (body["pose"] is JArray pose)
                {
                    go.transform.localPosition = BaseFrame.ToUnity(pose, 0);
                    go.transform.localRotation = BaseFrame.ToUnityRotation(pose, 3);
                }
                foreach (var geom in (JArray)body["geoms"])
                    CreateGeom(go.transform, (JObject)geom);
                bodies[name] = go.transform;
                if (!byKind.TryGetValue(kind, out var list))
                {
                    list = new List<GameObject>();
                    byKind[kind] = list;
                }
                list.Add(go);
            }
            ApplyVisibility();
            ApplyTableHeight(info);
            Debug.Log($"Sim world built: {bodies.Count} bodies, scene {(string)info["version"]}");
        }

        void ApplyTableHeight(JObject info)
        {
            foreach (var body in (JArray)info["bodies"])
            {
                if ((string)body["kind"] != "world")
                    continue;
                var bottom = float.MaxValue;
                foreach (var geom in (JArray)body["geoms"])
                {
                    var name = (string)geom["name"];
                    if (name == null || !name.StartsWith("table"))
                        continue;
                    bottom = Mathf.Min(bottom, (float)geom["pos"][2] - (float)geom["size"][2]);
                }
                if (bottom < float.MaxValue)
                    anchor.SetTableHeight(-bottom);
                return;
            }
        }

        void ApplyVisibility()
        {
            SetKindVisible("hand_left", showSimHands);
            SetKindVisible("hand_right", showSimHands);
            SetKindVisible("marker", showMarkers);
        }

        void SetKindVisible(string kind, bool visible)
        {
            if (!byKind.TryGetValue(kind, out var list))
                return;
            foreach (var go in list)
                go.SetActive(visible);
        }

        void CreateGeom(Transform parent, JObject geom)
        {
            var type = (string)geom["type"];
            var size = geom["size"];
            var frame = new GameObject((string)geom["name"]).transform;
            frame.SetParent(parent, false);
            frame.localPosition = BaseFrame.ToUnity(geom["pos"]);
            frame.localRotation = BaseFrame.ToUnityRotation(geom["quat"]);
            var material = MakeMaterial(geom["rgba"], null);

            switch (type)
            {
                case "mesh":
                    var mesh = LoadMesh((string)geom["mesh"]);
                    if (mesh == null)
                        return;
                    var go = new GameObject("mesh");
                    go.transform.SetParent(frame, false);
                    go.AddComponent<MeshFilter>().sharedMesh = mesh;
                    go.AddComponent<MeshRenderer>().sharedMaterial = material;
                    break;
                case "box":
                    Primitive(PrimitiveType.Cube, frame, BaseFrame.BoxScale(size), material);
                    break;
                case "sphere":
                    Primitive(PrimitiveType.Sphere, frame, Vector3.one * 2f * (float)size[0], material);
                    break;
                case "ellipsoid":
                    Primitive(PrimitiveType.Sphere, frame, BaseFrame.BoxScale(size), material);
                    break;
                case "cylinder":
                    Primitive(PrimitiveType.Cylinder, frame,
                        new Vector3(2f * (float)size[0], (float)size[1], 2f * (float)size[0]), material);
                    break;
                case "capsule":
                    Primitive(PrimitiveType.Capsule, frame,
                        new Vector3(2f * (float)size[0], (float)size[1] + (float)size[0], 2f * (float)size[0]), material);
                    break;
            }

            var texName = (string)geom["texture"];
            if (type == "box" && texName != null && textures.TryGetValue(texName, out var tex))
                TopFaceTexture(frame, size, tex);
        }

        void TopFaceTexture(Transform frame, JToken halfSize, Texture2D tex)
        {
            var quad = GameObject.CreatePrimitive(PrimitiveType.Quad);
            Destroy(quad.GetComponent<Collider>());
            quad.name = "texture";
            quad.transform.SetParent(frame, false);
            quad.transform.localPosition = new Vector3(0f, (float)halfSize[2] + 0.0003f, 0f);
            quad.transform.localRotation = Quaternion.LookRotation(Vector3.down, Vector3.left);
            quad.transform.localScale = new Vector3(2f * (float)halfSize[0], 2f * (float)halfSize[1], 1f);
            quad.GetComponent<MeshRenderer>().sharedMaterial = MakeMaterial(null, tex);
        }

        void Primitive(PrimitiveType kind, Transform parent, Vector3 scale, Material material)
        {
            var go = GameObject.CreatePrimitive(kind);
            Destroy(go.GetComponent<Collider>());
            go.transform.SetParent(parent, false);
            go.transform.localScale = scale;
            go.GetComponent<MeshRenderer>().sharedMaterial = material;
        }

        Material MakeMaterial(JToken rgba, Texture2D tex)
        {
            var m = baseMaterial != null
                ? new Material(baseMaterial)
                : new Material(Shader.Find("Universal Render Pipeline/Lit"));
            var color = rgba is JArray c
                ? new Color((float)c[0], (float)c[1], (float)c[2], (float)c[3])
                : Color.white;
            m.color = color;
            if (m.HasProperty("_BaseColor"))
                m.SetColor("_BaseColor", color);
            if (tex != null)
            {
                m.mainTexture = tex;
                if (m.HasProperty("_BaseMap"))
                    m.SetTexture("_BaseMap", tex);
            }
            return m;
        }

        Mesh LoadMesh(string name)
        {
            if (meshes.TryGetValue(name, out var cached))
                return cached;
            var asset = Resources.Load<TextAsset>(MeshFolder + name);
            if (asset == null)
            {
                Debug.LogWarning($"Mesh {name} is not in Resources/{MeshFolder}. Run python -m sim.scene_export.");
                meshes[name] = null;
                return null;
            }
            var mesh = ReadMesh(asset.bytes, name);
            meshes[name] = mesh;
            return mesh;
        }

        Mesh ReadMesh(byte[] data, string name)
        {
            using var reader = new BinaryReader(new MemoryStream(data));
            var nVerts = reader.ReadInt32();
            var nFaces = reader.ReadInt32();
            var verts = new Vector3[nVerts];
            for (var i = 0; i < nVerts; i++)
                verts[i] = BaseFrame.ToUnity(reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle());
            var flatVerts = new Vector3[nFaces * 3];
            var tris = new int[nFaces * 3];
            for (var f = 0; f < nFaces; f++)
            {
                var a = reader.ReadInt32();
                var b = reader.ReadInt32();
                var c = reader.ReadInt32();
                flatVerts[3 * f] = verts[a];
                flatVerts[3 * f + 1] = verts[b];
                flatVerts[3 * f + 2] = verts[c];
                tris[3 * f] = 3 * f;
                tris[3 * f + 1] = flipMeshWinding ? 3 * f + 1 : 3 * f + 2;
                tris[3 * f + 2] = flipMeshWinding ? 3 * f + 2 : 3 * f + 1;
            }
            var mesh = new Mesh { name = name };
            if (flatVerts.Length > 65000)
                mesh.indexFormat = IndexFormat.UInt32;
            mesh.vertices = flatVerts;
            mesh.triangles = tris;
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();
            return mesh;
        }
    }
}
