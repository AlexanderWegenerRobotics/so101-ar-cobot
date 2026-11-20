using System.Collections;
using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

namespace Cobot
{
    public class WorldAnchor : MonoBehaviour
    {
        public static WorldAnchor Instance { get; private set; }

        [SerializeField] Transform head;
        [SerializeField] float distance = 0.75f;
        [SerializeField] float tableHeight = 0.9f;
        [SerializeField] bool placeOnStart = true;
        [SerializeField] bool suppressBoundary = true;

        public float TableHeight => tableHeight;

        void Awake()
        {
            Instance = this;
        }

        IEnumerator Start()
        {
            if (head == null && Camera.main != null)
                head = Camera.main.transform;
            if (suppressBoundary && OVRManager.instance != null)
                OVRManager.instance.shouldBoundaryVisibilityBeSuppressed = true;
            if (!placeOnStart || head == null)
                yield break;
            var waited = 0f;
            while (head.localPosition.y < 0.3f && waited < 5f)
            {
                waited += Time.unscaledDeltaTime;
                yield return null;
            }
            yield return new WaitForSeconds(0.5f);
            PlaceInFrontOfUser();
        }

        void Update()
        {
            if (OVRInput.GetDown(OVRInput.Button.One))
                PlaceInFrontOfUser();
#if ENABLE_INPUT_SYSTEM
            if (Keyboard.current != null && Keyboard.current.rKey.wasPressedThisFrame)
                PlaceInFrontOfUser();
#endif
        }

        public void SetTableHeight(float height)
        {
            tableHeight = height;
            var p = transform.position;
            transform.position = new Vector3(p.x, FloorY() + tableHeight, p.z);
        }

        public void PlaceInFrontOfUser()
        {
            if (head == null)
                return;
            var forward = head.forward;
            forward.y = 0f;
            if (forward.sqrMagnitude < 1e-4f)
                forward = Vector3.forward;
            forward.Normalize();
            var p = head.position + forward * distance;
            transform.SetPositionAndRotation(new Vector3(p.x, FloorY() + tableHeight, p.z),
                Quaternion.LookRotation(-forward, Vector3.up));
        }

        float FloorY() => head != null && head.parent != null ? head.parent.position.y : 0f;

        public void SetPose(Vector3 position, Quaternion rotation)
        {
            transform.SetPositionAndRotation(position, rotation);
        }

        public Vector3 ToBase(Vector3 worldPosition) => BaseFrame.ToBase(transform.InverseTransformPoint(worldPosition));

        public Vector3 ToBaseDirection(Vector3 worldDirection) =>
            BaseFrame.ToBase(transform.InverseTransformDirection(worldDirection));
    }
}
