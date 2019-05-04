using OpenTK;

namespace libPE2D
{
    public class RigidBody
    {
        public IShape Shape { get; private set; }
        public Material Material { get; private set; }
        public float Mass { get; internal set; }
        public float InverseMass { get; internal set; }
        public float Inertia { get; internal set; }
        public float InverseInertia { get; internal set; }

        public float GravityScale { get; set; }
        public Vector2 Fource { get; set; }
        public float Torque { get; set; }
        public Vector2 Velocity { get; set; }
        public float AngularVelocity { get; set; }
        public Transform Transform { get; set; }

        internal RigidBody(IShape shape, Material material)
        {
            shape.Body = this;

            this.Shape = shape;
            this.Material = material;
            this.Transform = new Transform();
            this.GravityScale = 1.0f;
            this.Velocity = Vector2.Zero;
            this.AngularVelocity = 0.0f;

            this.SetDynamic();
            this.ClearForce();
        }

        public void ApplyForce(Vector2 force)
        {
            this.ApplyForce(force, Vector2.Zero);
        }

        public void ApplyForce(Vector2 force, Vector2 contact)
        {
            this.Fource += force;
            this.Torque += Vector2Ext.Cross(contact, force);
        }

        public void ClearForce()
        {
            this.Fource = Vector2.Zero;
            this.Torque = 0.0f;
        }

        public void ApplyImpulse(Vector2 impulse, Vector2 contact)
        {
            this.Velocity += this.InverseMass * impulse;
            this.AngularVelocity += this.InverseInertia * Vector2Ext.Cross(contact, impulse);
        }

        public void SetStatic()
        {
            this.Mass = 0;
            this.Inertia = 0;
            this.InverseMass = 0;
            this.InverseInertia = 0;
        }

        public void SetDynamic()
        {
            this.Shape.CalculateMass(this.Material.Density);
        }
    }
}
