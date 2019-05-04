using System;
using System.Collections.Generic;
using OpenTK;

namespace libPE2D
{
    public class Scene
    {
        public Vector2 Gravity { get; set; }
        public List<RigidBody> Objects { get; private set; }
        public float UpdateTime { get; private set; }
        public int IterationTimes { get; private set; }

        public Scene(float dt, int iter, Vector2 gravity)
        {
            this.UpdateTime = dt;
            this.IterationTimes = iter;
            this.Gravity = gravity;
            this.Contacts = new List<Manifold>();
            this.Objects = new List<RigidBody>();
        }
        public RigidBody AddBody(IShape shape, Material m)
        {
            RigidBody obj = new RigidBody(shape, m);
            this.Objects.Add(obj);
            return obj;
        }
        public void RemoveBody(RigidBody obj)
        {
            this.Objects.Remove(obj);
        }
        public void Step()
        {
            this.Contacts.Clear();

            BroadPhase broadphase = new BroadPhase();
            
            foreach (RigidBody obj in this.Objects)
                obj.Shape.SetOrientation(obj.Transform.Orientation);

            foreach (RigidBody obj in this.Objects)
                broadphase.AddBody(obj);

            foreach (Pair<RigidBody> pairs in broadphase.GetPairs())
            {
                Manifold m = new Manifold(pairs.Left, pairs.Right);
                m.Solve();
                if (m.ContactPoints != null)
                    this.Contacts.Add(m);
            }

            foreach (RigidBody obj in this.Objects)
                this.IntegreationForce(obj);

            foreach (Manifold m in this.Contacts)
                m.Init();

            for (int i = 0; i < this.IterationTimes; ++i)
                foreach (Manifold m in this.Contacts)
                    m.ApplyImpluse();

            foreach (RigidBody obj in this.Objects)
                this.IntegreationVelocity(obj);

            foreach (Manifold m in this.Contacts)
                m.PositionalCorrection();

            foreach (RigidBody obj in this.Objects)
                obj.ClearForce();
        }

        public List<Manifold> Contacts { get; private set; }

        private void IntegreationForce(RigidBody obj)
        {
            if (PhyMath.EpsilonEqual(obj.InverseMass, 0.0f))
                return;
            obj.Velocity += (obj.InverseMass * obj.Fource + this.Gravity * obj.GravityScale) * this.UpdateTime;
            obj.AngularVelocity += obj.InverseInertia * obj.Torque * this.UpdateTime;
        }
        private void IntegreationVelocity(RigidBody obj)
        {
            obj.Transform.Position += obj.Velocity * this.UpdateTime;
            obj.Transform.Orientation += obj.AngularVelocity * this.UpdateTime;
        }
    }
}
