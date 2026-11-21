using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace Cobot.EditorTools
{
    public static class CobotSceneSetup
    {
        const string LitMaterial = "Packages/com.unity.render-pipelines.universal/Runtime/Materials/Lit.mat";

        [MenuItem("Cobot/Add VR World To Scene")]
        public static void AddToScene()
        {
            var existingBridge = Object.FindFirstObjectByType<CobotBridge>();
            var bridgeGo = existingBridge != null ? existingBridge.gameObject : new GameObject("Cobot");
            GetOrAdd<CobotBridge>(bridgeGo);
            var streamer = GetOrAdd<HandStreamer>(bridgeGo);
            var wristMenu = GetOrAdd<WristMenu>(bridgeGo);
            GetOrAdd<CobotWristActions>(bridgeGo);

            var existingAnchor = Object.FindFirstObjectByType<WorldAnchor>();
            var anchorGo = existingAnchor != null ? existingAnchor.gameObject : new GameObject("SimBaseFrame");
            var anchor = GetOrAdd<WorldAnchor>(anchorGo);
            var renderer = GetOrAdd<WorldRenderer>(anchorGo);
            var workspace = GetOrAdd<WorkspaceOverlay>(anchorGo);
            var tasks = GetOrAdd<TaskOverlay>(anchorGo);
            var mission = GetOrAdd<MissionControl>(anchorGo);

            var cam = Camera.main;
            Set(anchor, "head", cam != null ? cam.transform : null);
            Set(renderer, "anchor", anchor);
            Set(workspace, "anchor", anchor);
            Set(tasks, "anchor", anchor);
            Set(mission, "anchor", anchor);
            Set(renderer, "baseMaterial", AssetDatabase.LoadAssetAtPath<Material>(LitMaterial));
            Set(streamer, "anchor", anchor);
            Set(wristMenu, "baseMaterial", AssetDatabase.LoadAssetAtPath<Material>(LitMaterial));

            if (cam != null && Object.FindFirstObjectByType<StatusLabel>() == null)
            {
                var label = new GameObject("CobotStatus");
                label.transform.SetParent(cam.transform, false);
                label.transform.localPosition = new Vector3(0f, -0.12f, 0.6f);
                var tm = label.AddComponent<TextMesh>();
                tm.font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
                label.GetComponent<MeshRenderer>().sharedMaterial = tm.font.material;
                tm.characterSize = 0.004f;
                tm.fontSize = 60;
                tm.anchor = TextAnchor.MiddleCenter;
                tm.alignment = TextAlignment.Center;
                tm.color = Color.white;
                var status = label.AddComponent<StatusLabel>();
                Set(status, "world", renderer);
            }

            if (Object.FindFirstObjectByType<OVRSkeleton>() == null)
                Debug.LogWarning("No hand skeleton in the scene. Add the Hand Tracking building block so HandStreamer can send your hands.");

            var config = OVRProjectConfig.CachedProjectConfig;
            if (config.boundaryVisibilitySupport == OVRProjectConfig.FeatureSupport.None)
            {
                config.boundaryVisibilitySupport = OVRProjectConfig.FeatureSupport.Supported;
                OVRProjectConfig.CommitProjectConfig(config);
            }

            PlayerSettings.Android.forceInternetPermission = true;
            EditorSceneManager.MarkSceneDirty(anchorGo.scene);
            Selection.activeGameObject = bridgeGo;
            Debug.Log("Cobot VR world added. Set the Mac's IP address in Cobot > CobotBridge > Host.");
        }

        static T GetOrAdd<T>(GameObject go) where T : Component =>
            go.TryGetComponent<T>(out var c) ? c : go.AddComponent<T>();

        static void Set(Object target, string field, Object value)
        {
            var so = new SerializedObject(target);
            var prop = so.FindProperty(field);
            if (prop == null)
            {
                Debug.LogWarning($"{target.GetType().Name}.{field} not found");
                return;
            }
            prop.objectReferenceValue = value;
            so.ApplyModifiedPropertiesWithoutUndo();
        }
    }
}
