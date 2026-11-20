using UnityEngine;

namespace Cobot
{
    public static class HandJoints
    {
        static OVRSkeleton left;
        static OVRSkeleton right;
        static float nextSearch;

        public static OVRSkeleton Get(bool isLeft)
        {
            if ((left == null || right == null) && Time.unscaledTime >= nextSearch)
            {
                nextSearch = Time.unscaledTime + 1f;
                foreach (var s in Object.FindObjectsByType<OVRSkeleton>(FindObjectsSortMode.None))
                {
                    var type = s.GetSkeletonType();
                    if (type == OVRSkeleton.SkeletonType.XRHandLeft)
                        left = s;
                    else if (type == OVRSkeleton.SkeletonType.XRHandRight)
                        right = s;
                }
            }
            return isLeft ? left : right;
        }

        public static bool IsTracked(OVRSkeleton skeleton)
        {
            if (skeleton == null || !skeleton.IsInitialized || !skeleton.IsDataValid)
                return false;
            var hand = skeleton.GetComponent<OVRHand>();
            return hand == null || hand.IsTracked;
        }

        public static bool TryGetPosition(bool isLeft, OVRSkeleton.BoneId id, out Vector3 position)
        {
            position = default;
            var skeleton = Get(isLeft);
            if (!IsTracked(skeleton))
                return false;
            foreach (var bone in skeleton.Bones)
            {
                if (bone != null && bone.Id == id && bone.Transform != null)
                {
                    position = bone.Transform.position;
                    return true;
                }
            }
            return false;
        }
    }
}
