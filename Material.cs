namespace libPE2D
{
    /// <summary> 材料。這個結構將會影響物件的重量和受力變化。 </summary>
    public struct Material
    {
        /// <summary> 材料密度。這個數值將會影響物件的重量。 </summary>
        public float Density { get; private set; }
        /// <summary> 恢復係數。這個數值將會影響碰撞時所受到的衝力。</summary>
        public float Restitution { get; private set; }
        /// <summary> 靜摩擦係數。</summary>
        public float us { get; private set; }
        /// <summary> 動摩擦係數。</summary>
        public float uk { get; private set; }

        /// <summary> 建立一個材料實體。 </summary>
        /// <param name="d"> 密度 </param>
        /// <param name="e"> 恢復係數 </param>
        /// <param name="us"> 靜摩擦係數 </param>
        /// <param name="uk"> 動摩擦係數 </param>
        public Material(float d, float e, float us, float uk)
        {
            this.Density = d;
            this.Restitution = e;
            this.us = us;
            this.uk = uk;
        }
        
        /// <summary> 石頭。 </summary>
        public static readonly Material Rock = new Material(0.6f, 0.10f, 0.55f, 0.45f);
        /// <summary> 木頭。 </summary>
        public static readonly Material Wood = new Material(0.3f, 0.20f, 0.6f, 0.5f);
        /// <summary> 金屬。 </summary>
        public static readonly Material Metal = new Material(1.2f, 0.05f, 0.15f, 0.1f);
        /// <summary> 彈力球。 </summary>
        public static readonly Material BouncyBall = new Material(0.3f, 0.80f, 0.5f, 0.4f);
        /// <summary> 超級彈力球。 </summary>
        public static readonly Material SuperBall = new Material(0.3f, 0.95f, 0.5f, 0.4f);
        /// <summary> 枕頭。 </summary>
        public static readonly Material Pillow = new Material(0.1f, 0.20f, 0.4f, 0.3f);
    }
}
