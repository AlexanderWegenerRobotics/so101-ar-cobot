using Newtonsoft.Json.Linq;
using UnityEngine;

namespace Cobot
{
    public static class BaseFrame
    {
        public static Vector3 ToUnity(float x, float y, float z) => new Vector3(-y, z, x);

        public static Vector3 ToUnity(JToken a, int offset = 0) =>
            ToUnity((float)a[offset], (float)a[offset + 1], (float)a[offset + 2]);

        public static Quaternion ToUnityRotation(float w, float x, float y, float z) => new Quaternion(y, -z, -x, w);

        public static Quaternion ToUnityRotation(JToken wxyz, int offset = 0) =>
            ToUnityRotation((float)wxyz[offset], (float)wxyz[offset + 1], (float)wxyz[offset + 2], (float)wxyz[offset + 3]);

        public static Vector3 ToBase(Vector3 u) => new Vector3(u.z, -u.x, u.y);

        public static Vector3 BoxScale(JToken halfSize) =>
            new Vector3(2f * (float)halfSize[1], 2f * (float)halfSize[2], 2f * (float)halfSize[0]);

        public static JArray ToArray(Vector3 v) => new JArray(Round(v.x), Round(v.y), Round(v.z));

        static double Round(float v) => System.Math.Round(v, 5);
    }
}
