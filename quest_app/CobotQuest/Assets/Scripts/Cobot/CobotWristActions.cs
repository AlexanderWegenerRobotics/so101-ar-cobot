using Newtonsoft.Json.Linq;
using UnityEngine;

namespace Cobot
{
    [RequireComponent(typeof(WristMenu))]
    public class CobotWristActions : MonoBehaviour
    {
        WristMenu menu;
        WorkspaceOverlay workspace;
        TaskOverlay tasks;

        static readonly (string id, string label, string icon, float alpha)[] Mixes =
        {
            ("mix_fast", "Fast", "fast", 0f),
            ("mix_even", "Even", "even", 0.5f),
            ("mix_robot", "Robot max", "robot", 1f),
        };

        void OnEnable()
        {
            menu = GetComponent<WristMenu>();
            menu.Pressed += OnPressed;
        }

        void Start()
        {
            workspace = FindFirstObjectByType<WorkspaceOverlay>();
            tasks = FindFirstObjectByType<TaskOverlay>();
            if (tasks != null && !menu.HasButton("toggle_run"))
                menu.AddButton("toggle_run", "Go", new Color(0.2f, 0.7f, 0.35f), "run");
            if (!menu.HasButton(WristMenu.SettingsId))
                menu.AddButton(WristMenu.SettingsId, "Settings", new Color(0.45f, 0.45f, 0.5f), "gear");
            if (workspace != null && !menu.HasButton("toggle_workspace"))
                menu.AddButton("toggle_workspace", "Reach", new Color(0.3f, 0.75f, 0.4f), "zones", "settings");
            if (tasks != null)
            {
                if (!menu.HasButton("toggle_tasks"))
                    menu.AddButton("toggle_tasks", "Tasks", new Color(1f, 0.6f, 0.15f), "tasks", "settings");
                foreach (var m in Mixes)
                    if (!menu.HasButton(m.id))
                        menu.AddButton(m.id, m.label, new Color(0.65f, 0.4f, 0.9f), m.icon, "settings");
            }
        }

        void OnDisable()
        {
            menu.Pressed -= OnPressed;
        }

        void Update()
        {
            if (workspace != null)
                menu.SetDim("toggle_workspace", !workspace.Visible);
            if (tasks == null)
                return;
            menu.SetDim("toggle_tasks", !tasks.Visible);
            var running = tasks.Mode == "auto";
            menu.SetLabel("toggle_run", tasks.Mode == "resetting" ? "Wait" : running ? "Stop" : "Go");
            var alpha = tasks.Alpha;
            foreach (var m in Mixes)
                menu.SetDim(m.id, !alpha.HasValue || Mathf.Abs(alpha.Value - m.alpha) > 0.01f);
        }

        void OnPressed(string id)
        {
            switch (id)
            {
                case "place_table":
                    if (WorldAnchor.Instance != null)
                        WorldAnchor.Instance.PlaceInFrontOfUser();
                    break;
                case "reset_scene":
                    SendCommand("reset_scene");
                    break;
                case "toggle_workspace":
                    if (workspace != null)
                        workspace.Toggle();
                    break;
                case "toggle_run":
                    SendCommand(tasks != null && tasks.Mode == "auto" ? "pause" : "start");
                    break;
                case "toggle_tasks":
                    if (tasks != null)
                        tasks.Toggle();
                    break;
                default:
                    foreach (var m in Mixes)
                        if (m.id == id)
                            SendCommand("set_alpha", m.alpha);
                    break;
            }
        }

        static void SendCommand(string kind, float? value = null)
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
            CobotBridge.Instance.Publish(Topics.OperatorCommand, msg);
        }
    }
}
