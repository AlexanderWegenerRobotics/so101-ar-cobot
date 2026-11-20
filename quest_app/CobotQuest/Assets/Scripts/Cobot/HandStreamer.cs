using System.Collections.Generic;
using Newtonsoft.Json.Linq;
using UnityEngine;

namespace Cobot
{
    public class HandStreamer : MonoBehaviour
    {
        [SerializeField] WorldAnchor anchor;
        [SerializeField] float rateHz = 60f;
        [SerializeField] float pinchClosed = 0.015f;
        [SerializeField] float pinchOpen = 0.06f;

        static readonly OVRSkeleton.BoneId[] Joints =
        {
            OVRSkeleton.BoneId.XRHand_Palm,
            OVRSkeleton.BoneId.XRHand_Wrist,
            OVRSkeleton.BoneId.XRHand_ThumbMetacarpal,
            OVRSkeleton.BoneId.XRHand_ThumbProximal,
            OVRSkeleton.BoneId.XRHand_ThumbDistal,
            OVRSkeleton.BoneId.XRHand_ThumbTip,
            OVRSkeleton.BoneId.XRHand_IndexMetacarpal,
            OVRSkeleton.BoneId.XRHand_IndexProximal,
            OVRSkeleton.BoneId.XRHand_IndexIntermediate,
            OVRSkeleton.BoneId.XRHand_IndexDistal,
            OVRSkeleton.BoneId.XRHand_IndexTip,
            OVRSkeleton.BoneId.XRHand_MiddleMetacarpal,
            OVRSkeleton.BoneId.XRHand_MiddleProximal,
            OVRSkeleton.BoneId.XRHand_MiddleIntermediate,
            OVRSkeleton.BoneId.XRHand_MiddleDistal,
            OVRSkeleton.BoneId.XRHand_MiddleTip,
            OVRSkeleton.BoneId.XRHand_RingMetacarpal,
            OVRSkeleton.BoneId.XRHand_RingProximal,
            OVRSkeleton.BoneId.XRHand_RingIntermediate,
            OVRSkeleton.BoneId.XRHand_RingDistal,
            OVRSkeleton.BoneId.XRHand_RingTip,
            OVRSkeleton.BoneId.XRHand_LittleMetacarpal,
            OVRSkeleton.BoneId.XRHand_LittleProximal,
            OVRSkeleton.BoneId.XRHand_LittleIntermediate,
            OVRSkeleton.BoneId.XRHand_LittleDistal,
            OVRSkeleton.BoneId.XRHand_LittleTip,
        };

        const int ThumbTip = 5;
        const int IndexProximal = 7;
        const int IndexTip = 10;

        readonly List<(string side, OVRSkeleton skeleton, OVRHand hand)> hands = new List<(string, OVRSkeleton, OVRHand)>();
        readonly Dictionary<OVRSkeleton.BoneId, Transform> bones = new Dictionary<OVRSkeleton.BoneId, Transform>();
        readonly Vector3[] points = new Vector3[26];
        float nextSend;
        float nextSearch;
        bool warned;

        public int HandCount => hands.Count;
        public float LastTrackedTime { get; private set; } = -1f;

        void Start()
        {
            if (anchor == null)
                anchor = WorldAnchor.Instance;
        }

        void Update()
        {
            if (hands.Count < 2 && Time.unscaledTime >= nextSearch)
                FindHands();
            var bridge = CobotBridge.Instance;
            if (bridge == null || !bridge.Connected || Time.unscaledTime < nextSend)
                return;
            nextSend = Time.unscaledTime + 1f / rateHz;
            foreach (var (side, skeleton, hand) in hands)
                Send(bridge, side, skeleton, hand);
        }

        void FindHands()
        {
            nextSearch = Time.unscaledTime + 1f;
            hands.Clear();
            foreach (var skeleton in FindObjectsByType<OVRSkeleton>(FindObjectsSortMode.None))
            {
                var type = skeleton.GetSkeletonType();
                string side = null;
                if (type == OVRSkeleton.SkeletonType.XRHandLeft)
                    side = "left";
                else if (type == OVRSkeleton.SkeletonType.XRHandRight)
                    side = "right";
                if (side != null)
                    hands.Add((side, skeleton, skeleton.GetComponent<OVRHand>()));
            }
            if (hands.Count == 0 && !warned && Time.unscaledTime > 5f)
            {
                Debug.LogWarning("HandStreamer: no OVRSkeleton with an XRHand skeleton in the scene. Add the Hand Tracking building block.");
                warned = true;
            }
        }

        void Send(CobotBridge bridge, string side, OVRSkeleton skeleton, OVRHand hand)
        {
            var msg = new JObject
            {
                ["type"] = "HandState",
                ["side"] = side,
                ["source"] = "quest",
                ["stamp"] = CobotBridge.Now(),
            };
            var tracked = skeleton != null && skeleton.IsInitialized && skeleton.IsDataValid &&
                          (hand == null || hand.IsTracked) && ReadJoints(skeleton);
            if (!tracked)
            {
                msg["tracked"] = false;
                msg["joints"] = new JArray();
                bridge.Publish(Topics.Hand, msg);
                return;
            }
            var joints = new JArray();
            foreach (var p in points)
                joints.Add(BaseFrame.ToArray(p));
            var pinch = 1f - Mathf.InverseLerp(pinchClosed, pinchOpen, Vector3.Distance(points[ThumbTip], points[IndexTip]));
            var dir = (points[IndexTip] - points[IndexProximal]).normalized;
            LastTrackedTime = Time.unscaledTime;
            msg["tracked"] = true;
            msg["joints"] = joints;
            msg["pinch"] = System.Math.Round(pinch, 3);
            msg["ray_origin"] = BaseFrame.ToArray(points[IndexTip]);
            msg["ray_dir"] = BaseFrame.ToArray(dir);
            bridge.Publish(Topics.Hand, msg);
        }

        bool ReadJoints(OVRSkeleton skeleton)
        {
            bones.Clear();
            foreach (var bone in skeleton.Bones)
                if (bone != null && bone.Transform != null)
                    bones[bone.Id] = bone.Transform;
            for (var i = 0; i < Joints.Length; i++)
            {
                if (!bones.TryGetValue(Joints[i], out var t))
                    return false;
                points[i] = anchor.ToBase(t.position);
            }
            return true;
        }
    }
}
