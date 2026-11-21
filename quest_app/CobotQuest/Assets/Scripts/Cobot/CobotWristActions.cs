using UnityEngine;

namespace Cobot
{
    [RequireComponent(typeof(WristMenu))]
    public class CobotWristActions : MonoBehaviour
    {
        WristMenu menu;

        void OnEnable()
        {
            menu = GetComponent<WristMenu>();
            menu.Pressed += OnPressed;
        }

        void Start()
        {
            foreach (var id in new[] { "reset_scene", "toggle_run", WristMenu.SettingsId, "toggle_workspace", "toggle_tasks",
                         "mix_fast", "mix_even", "mix_robot" })
                menu.RemoveButton(id);
            if (!menu.HasButton("place_table"))
                menu.AddButton("place_table", "Table", new Color(0.25f, 0.55f, 0.95f), "table");
        }

        void OnDisable()
        {
            menu.Pressed -= OnPressed;
        }

        void OnPressed(string id)
        {
            if (id == "place_table" && WorldAnchor.Instance != null)
                WorldAnchor.Instance.PlaceInFrontOfUser();
        }
    }
}
