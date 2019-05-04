using System;

namespace libPE2D
{
    internal class PhyMath
    {
        public const float Epsilon = 1e-8f;

        public static bool EpsilonEqual(float a, float b)
        {
            return Math.Abs(a - b) < Epsilon;
        }
    }
}
