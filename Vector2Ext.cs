using OpenTK;

namespace libPE2D
{
    public static class Vector2Ext
    {
        public static float Cross(Vector2 lhs, Vector2 rhs)
        {
            return lhs.X * rhs.Y - rhs.X * lhs.Y;
        }
        public static Vector2 Cross(float z, Vector2 rhs)
        {
            return new Vector2(-rhs.Y * z, rhs.X * z);
        }
        public static Vector2 Cross(Vector2 lhs, float z)
        {
            return new Vector2(lhs.Y * z, -lhs.X * z);
        }
    }
}
