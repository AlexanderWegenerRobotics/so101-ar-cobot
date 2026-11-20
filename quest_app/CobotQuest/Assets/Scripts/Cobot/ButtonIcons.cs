using System;
using UnityEngine;

namespace Cobot
{
    public static class ButtonIcons
    {
        const int Size = 256;

        public static Texture2D Make(string icon, Color fill)
        {
            var tex = new Texture2D(Size, Size, TextureFormat.RGBA32, true)
            {
                wrapMode = TextureWrapMode.Clamp,
                filterMode = FilterMode.Trilinear,
                anisoLevel = 4,
            };
            Func<Vector2, bool> shape = Shape(icon);
            var pixels = new Color32[Size * Size];
            for (var y = 0; y < Size; y++)
            {
                for (var x = 0; x < Size; x++)
                {
                    var inDisc = 0;
                    var white = 0;
                    for (var sy = 0; sy < 3; sy++)
                    {
                        for (var sx = 0; sx < 3; sx++)
                        {
                            var p = new Vector2((x + (sx + 0.5f) / 3f) / Size, (y + (sy + 0.5f) / 3f) / Size);
                            var r = Vector2.Distance(p, new Vector2(0.5f, 0.5f));
                            if (r > 0.48f)
                                continue;
                            inDisc++;
                            if (r > 0.435f || (shape != null && shape(p)))
                                white++;
                        }
                    }
                    var a = inDisc / 9f;
                    var w = inDisc > 0 ? white / (float)inDisc : 0f;
                    var c = Color.Lerp(fill, Color.white, w);
                    c.a = a;
                    pixels[y * Size + x] = c;
                }
            }
            tex.SetPixels32(pixels);
            tex.Apply(true);
            return tex;
        }

        static Func<Vector2, bool> Shape(string icon)
        {
            switch (icon)
            {
                case "table":
                    return p => Box(p, 0.5f, 0.6f, 0.24f, 0.04f)
                                || Box(p, 0.33f, 0.43f, 0.035f, 0.15f)
                                || Box(p, 0.67f, 0.43f, 0.035f, 0.15f);
                case "reset":
                    return Reset;
                case "zones":
                    return Zones;
                case "run":
                    return p => InTriangle(p, new Vector2(0.38f, 0.28f), new Vector2(0.38f, 0.72f), new Vector2(0.72f, 0.5f));
                case "mix":
                    return p => Box(p, 0.34f, 0.42f, 0.07f, 0.12f) || Box(p, 0.66f, 0.56f, 0.07f, 0.12f)
                                || Box(p, 0.5f, 0.5f, 0.24f, 0.025f);
                case "tasks":
                    return Tasks;
                case "gear":
                    return Gear;
                case "fast":
                    return p => Chevron(p, 0.38f) || Chevron(p, 0.58f);
                case "even":
                    return p => Box(p, 0.5f, 0.42f, 0.025f, 0.18f) || Box(p, 0.5f, 0.6f, 0.22f, 0.02f)
                                || Box(p, 0.3f, 0.5f, 0.05f, 0.05f) || Box(p, 0.7f, 0.5f, 0.05f, 0.05f);
                case "robot":
                    return RobotArm;
                default:
                    return null;
            }
        }

        static bool Box(Vector2 p, float cx, float cy, float hx, float hy) =>
            Mathf.Abs(p.x - cx) <= hx && Mathf.Abs(p.y - cy) <= hy;

        static bool Reset(Vector2 p)
        {
            var d = p - new Vector2(0.5f, 0.5f);
            var r = d.magnitude;
            var angle = Mathf.Atan2(d.y, d.x) * Mathf.Rad2Deg;
            if (angle < 0f)
                angle += 360f;
            var onArc = Mathf.Abs(r - 0.2f) < 0.04f && angle > 60f;
            if (onArc)
                return true;
            var tip = Polar(60f, 0.2f) + new Vector2(0.5f, 0.5f);
            var tangent = new Vector2(-Mathf.Sin(60f * Mathf.Deg2Rad), Mathf.Cos(60f * Mathf.Deg2Rad));
            var normal = new Vector2(tangent.y, -tangent.x);
            var a = tip - tangent * 0.11f;
            var b = tip + normal * 0.11f;
            var c = tip - normal * 0.11f;
            return InTriangle(p, a, b, c);
        }

        static bool Zones(Vector2 p)
        {
            var d = p - new Vector2(0.5f, 0.32f);
            var r = d.magnitude;
            var angle = Mathf.Atan2(d.y, d.x) * Mathf.Rad2Deg;
            var inFan = angle > 20f && angle < 160f;
            var band = inFan && r > 0.12f && r < 0.31f;
            var robot = Mathf.Abs(d.x) < 0.05f && d.y > -0.05f && d.y < 0.03f;
            return band || robot;
        }

        static bool Gear(Vector2 p)
        {
            var d = p - new Vector2(0.5f, 0.5f);
            var r = d.magnitude;
            var a = Mathf.Atan2(d.y, d.x);
            var teeth = Mathf.Cos(a * 8f) > 0.3f;
            return (r > 0.09f && r < 0.2f) || (teeth && r >= 0.2f && r < 0.27f);
        }

        static bool Chevron(Vector2 p, float x0)
        {
            var dy = Mathf.Abs(p.y - 0.5f);
            var x = x0 + 0.16f - dy;
            return dy < 0.17f && p.x > x - 0.07f && p.x < x;
        }

        static bool RobotArm(Vector2 p)
        {
            var baseBox = Box(p, 0.4f, 0.3f, 0.12f, 0.04f);
            var upper = Segment(p, new Vector2(0.4f, 0.32f), new Vector2(0.45f, 0.62f), 0.035f);
            var fore = Segment(p, new Vector2(0.45f, 0.62f), new Vector2(0.68f, 0.52f), 0.03f);
            var jaw = Box(p, 0.71f, 0.46f, 0.02f, 0.06f);
            return baseBox || upper || fore || jaw;
        }

        static bool Segment(Vector2 p, Vector2 a, Vector2 b, float w)
        {
            var ab = b - a;
            var t = Mathf.Clamp01(Vector2.Dot(p - a, ab) / ab.sqrMagnitude);
            return Vector2.Distance(p, a + t * ab) < w;
        }

        static bool Tasks(Vector2 p)
        {
            var r = Vector2.Distance(p, new Vector2(0.5f, 0.42f));
            var ring = Mathf.Abs(r - 0.17f) < 0.035f;
            var pin = Box(p, 0.5f, 0.68f, 0.025f, 0.09f);
            return ring || pin;
        }

        static Vector2 Polar(float deg, float r) =>
            new Vector2(Mathf.Cos(deg * Mathf.Deg2Rad), Mathf.Sin(deg * Mathf.Deg2Rad)) * r;

        static bool InTriangle(Vector2 p, Vector2 a, Vector2 b, Vector2 c)
        {
            var d1 = Sign(p, a, b);
            var d2 = Sign(p, b, c);
            var d3 = Sign(p, c, a);
            var neg = d1 < 0 || d2 < 0 || d3 < 0;
            var pos = d1 > 0 || d2 > 0 || d3 > 0;
            return !(neg && pos);
        }

        static float Sign(Vector2 p, Vector2 a, Vector2 b) => (p.x - b.x) * (a.y - b.y) - (a.x - b.x) * (p.y - b.y);
    }
}
