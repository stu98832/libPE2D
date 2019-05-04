using System;
using System.Collections.Generic;
using OpenTK;

namespace libPE2D
{
    public class BroadPhase
    {

        public BroadPhase()
        {
            this.mColliders = new List<AABBCollider>();
        }
        public void AddBody(RigidBody body)
        {
            this.mColliders.Add(new AABBCollider(body));
        }
        public List<Pair<RigidBody>> GetPairs()
        {
            List<Pair<RigidBody>> pairs = new List<Pair<RigidBody>>();

            for (int i = 0; i < this.mColliders.Count; ++i)
            {
                AABBCollider A = this.mColliders[i];
                for (int j = i + 1; j < this.mColliders.Count; ++j)
                {
                    AABBCollider B = this.mColliders[j];

                    if (PhyMath.EpsilonEqual(A.Body.InverseMass + B.Body.InverseMass, 0.0f))
                        continue;

                    if (A.AABB.IsIntersection(B.AABB))
                        pairs.Add(new Pair<RigidBody>(A.Body, B.Body));
                }
            }
            return pairs;
        }

        private List<AABBCollider> mColliders;
    }

    public struct Pair<T>
    {
        public T Left { get; private set; }
        public T Right { get; private set; }

        public Pair(T l, T r)
        {
            this.Left = l;
            this.Right = r;
        }
    }

    public class AABBCollider
    {
        public RigidBody Body { get; set; }
        public AABB AABB { get; set; }

        public AABBCollider(RigidBody body)
        {
            AABB aabb = body.Shape.GetAABB();
            aabb.Offset(body.Transform.Position);
            this.Body = body;
            this.AABB = aabb;
        }
    }

    public struct AABB
    {
        public float Left { get; private set; }
        public float Top { get; private set; }
        public float Right { get; private set; }
        public float Bottom { get; private set; }

        public AABB(float l, float t, float r, float b)
        {
            this.Left = l;
            this.Top = t;
            this.Right = r;
            this.Bottom = b;
        }

        public void Offset(Vector2 v)
        {
            this.Left += v.X;
            this.Top += v.Y;
            this.Right += v.X;
            this.Bottom += v.Y;
        }

        public bool IsIntersection(AABB aabb)
        {
            bool axisX = this.Right - aabb.Left >= 0 && aabb.Right - this.Left >= 0;
            bool axisy = this.Bottom - aabb.Top >= 0 && aabb.Bottom - this.Top >= 0;

            return axisX && axisy;
        }
    }
}
