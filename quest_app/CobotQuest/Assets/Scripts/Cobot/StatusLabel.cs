using UnityEngine;

namespace Cobot
{
    public class StatusLabel : MonoBehaviour
    {
        [SerializeField] WorldRenderer world;
        [SerializeField] HandStreamer handStreamer;
        [SerializeField] float hideAfterOkSeconds = 3f;

        TextMesh text;
        float okSince = -1f;

        void Start()
        {
            text = GetComponent<TextMesh>();
            if (world == null)
                world = FindFirstObjectByType<WorldRenderer>();
            if (handStreamer == null)
                handStreamer = FindFirstObjectByType<HandStreamer>();
        }

        void Update()
        {
            var bridge = CobotBridge.Instance;
            string line;
            var ok = false;
            if (bridge == null)
                line = "no CobotBridge in the scene";
            else if (!bridge.Connected)
                line = bridge.Status;
            else if (world == null || world.SceneVersion == null)
                line = "connected, waiting for sim/scene_info";
            else if (CobotBridge.Now() - world.LastPoseTime > 0.5)
                line = $"scene {world.SceneVersion}, no poses";
            else
            {
                line = world.AssetsMatch ? $"scene {world.SceneVersion} live" : "meshes outdated, rerun scene_export";
                ok = world.AssetsMatch;
            }
            if (ok && okSince < 0f)
                okSince = Time.unscaledTime;
            if (!ok)
                okSince = -1f;
            var main = ok && Time.unscaledTime - okSince > hideAfterOkSeconds ? "" : line;
            text.text = string.IsNullOrEmpty(main) ? HandLine() : main + "\n" + HandLine();
        }

        string HandLine()
        {
            if (handStreamer == null)
                return "no HandStreamer";
            if (handStreamer.HandCount == 0)
                return Time.unscaledTime > 5f ? "no hand skeleton, add the Hand Tracking block" : "";
            if (Time.unscaledTime - handStreamer.LastTrackedTime > 2f)
                return "hands not tracked (put the controllers down)";
            return "";
        }
    }
}
