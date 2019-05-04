using OpenTK;

namespace libPE2D
{
    public enum ShapeType : int
    {
        Circle = 0,
        Polygon = 1
    }

    public abstract class IShape
    {
        public RigidBody Body { get; internal set; }
        public abstract ShapeType Type { get; }

        public abstract void CalculateMass(float density);
        public abstract void SetOrientation(float rad);
        public abstract AABB GetAABB();
    }

    public class Circle : IShape
    {
        public override ShapeType Type { get { return ShapeType.Circle; } }
        public float Radius { get; set; }

        public Circle(float r)
        {
            this.Radius = r;
        }

        public override void CalculateMass(float density)
        {
            if (this.Body == null)
                return;

            float radius_square = this.Radius * this.Radius;

            this.Body.Mass = MathHelper.Pi * radius_square * density;
            this.Body.Inertia = this.Body.Mass * radius_square / 2.0f;
            this.Body.InverseMass = this.Body.Mass == 0 ? 0 : (1.0f / this.Body.Mass);
            this.Body.InverseInertia = this.Body.Inertia == 0 ? 0 : (1.0f / this.Body.Inertia);
        }

        public override void SetOrientation(float rad)
        {
        }

        public override AABB GetAABB()
        {
            return new AABB(-this.Radius, -this.Radius, this.Radius, this.Radius);
        }
    }

    public class Polygon : IShape
    {
        public override ShapeType Type { get { return ShapeType.Polygon; } }
        public Vector2[] Vertices { get; private set; }
        public Vector2[] Normals { get; private set; }

        /// <summary> Create a box </summary>
        public Polygon(float left, float top, float right, float bottom)
        {
            this.Set(new Vector2[] {
                new Vector2(left, top), new Vector2(right, top),
                new Vector2(right, bottom), new Vector2(left, bottom) }
            );
        }
        public Polygon(Vector2[] vertices)
        {
            this.Set(vertices);
        }
        public void Set(Vector2[] vertices)
        {
            int count = vertices.Length;

            // 尋找最右方的頂點
            int rightMost = 0;
            float highestXCoord = vertices[0].X;
            for (int i = 1; i < count; ++i)
            {
                Vector2 v = vertices[i];
                if (v.X > highestXCoord || v.X == highestXCoord && v.Y < vertices[rightMost].Y)
                {
                    highestXCoord = v.X;
                    rightMost = i;
                }
            }

            // 重新組合索引
            int[] hull = new int[count];
            // 輸出個數
            int outCount = 0;
            int indexHull = rightMost;

            while (true)
            {
                hull[outCount] = indexHull;

                int nextHullIndex = 0;
                int lastIndex = hull[outCount];
                for (int i = 1; i < count; ++i)
                {
                    // 跳過重複的點
                    if (nextHullIndex == indexHull)
                    {
                        nextHullIndex = i;
                        continue;
                    }

                    Vector3 e1 = new Vector3(vertices[nextHullIndex]) - new Vector3(vertices[lastIndex]);
                    Vector3 e2 = new Vector3(vertices[i]) - new Vector3(vertices[lastIndex]);
                    float c = Vector3.Cross(e1, e2).Z;

                    if (c < 0.0f || (c == 0.0f && e2.LengthSquared > e1.LengthSquared))
                        nextHullIndex = i;
                }

                ++outCount;
                indexHull = nextHullIndex;

                if (nextHullIndex == rightMost)
                    break;
            }

            this.Vertices = new Vector2[outCount];
            this.Normals = new Vector2[outCount];
            this.mTransVertices = new Vector2[outCount];
            this.mTransNormals = new Vector2[outCount];

            for (int i = 0; i < outCount; ++i)
                this.Vertices[i] = this.mTransVertices[i] = vertices[hull[i]];

            for (int i = 0; i < outCount; ++i)
            {
                Vector2 dv = this.Vertices[(i + 1) % outCount] - this.Vertices[i];
                dv.Normalize();
                this.Normals[i] = this.mTransNormals[i] = new Vector2(dv.Y, -dv.X);
            }
            this.mLastOrientation = -1;
            this.SetOrientation(0);
        }
        public Vector2 GetSupport(Vector2 dir)
        {
            float d = float.MinValue;
            int j = 0;

            for (int i = 0; i < this.Vertices.Length; ++i)
            {
                float dot = Vector2.Dot(dir, this.mTransVertices[i]);

                if (dot > d)
                {
                    d = dot;
                    j = i;
                }
            }
            // return this.Vertices[j];
            return this.mTransVertices[j];
        }

        public override void CalculateMass(float density)
        {
            if (this.Body == null)
                return;

            float area = 0.0f;
            float inerial = 0.0f;
            Vector2 com = Vector2.Zero;

            int count = this.Vertices.Length;
            for (int i = 0; i < count; ++i)
            {
                Vector2 a = this.Vertices[i];
                Vector2 b = this.Vertices[(i + 1) % count];
                float D = a.X * b.Y - b.X * a.Y;
                float triangleArea = 0.5f * D;

                area += triangleArea;
                com += triangleArea * (a + b) / 3.0f;

                float intx2 = a.X * a.X + b.X * a.X + b.X * b.X;
                float inty2 = a.Y * a.Y + b.Y * a.Y + b.Y * b.Y;
                inerial += D * (intx2 + inty2);
            }
            com /= area;

            for (int i = 0; i < count; ++i)
                this.Vertices[i] -= com;

            this.Body.Mass = area * density;
            this.Body.Inertia = inerial * density / 12.0f;
            this.Body.InverseMass = this.Body.Mass == 0 ? 0 : (1.0f / this.Body.Mass);
            this.Body.InverseInertia = this.Body.Inertia == 0 ? 0 : (1.0f / this.Body.Inertia);
        }

        public override void SetOrientation(float rad)
        {
            if (rad == this.mLastOrientation)
                return;
            this.U = Quaternion.FromAxisAngle(new Vector3(0, 0, 1), rad);

            float l = float.MaxValue;
            float t = float.MaxValue;
            float r = float.MinValue;
            float b = float.MinValue;
            
            for (int i = 0; i < this.Vertices.Length; ++i)
            {

                this.mTransVertices[i] = Vector2.Transform(this.Vertices[i], this.U);
                this.mTransNormals[i] = Vector2.Transform(this.Normals[i], this.U);

                Vector2 v = this.mTransVertices[i];

                if (v.X < l)
                    l = v.X;
                if (v.Y < t)
                    t = v.Y;
                if (v.X > r)
                    r = v.X;
                if (v.Y > b)
                    b = v.Y;
            }
            this.mLastOrientation = rad;
            this.mTransAABB = new AABB(l, t, r, b);
        }

        public override AABB GetAABB()
        {
            return this.mTransAABB;
        }

        public Quaternion U { get; internal set; }

        internal Vector2[] mTransVertices { get; private set; }
        internal Vector2[] mTransNormals { get; private set; }
        internal AABB mTransAABB { get; private set; }

        private float mLastOrientation;
    }
}
