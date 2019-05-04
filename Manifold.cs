using System;
using OpenTK;

namespace libPE2D
{
    public class Manifold
    {
        public Manifold(RigidBody A, RigidBody B)
        {
            this.A = A;
            this.B = B;
        }

        public void Solve()
        {
            this.ContactPoints = null;
            Collision.Dispatch[(int)this.A.Shape.Type, (int)this.B.Shape.Type](this, this.A, this.B);
        }
        public void Init()
        {
            Material materialA = this.A.Material;
            Material materialB = this.B.Material;

            this.Restitutoin = (materialA.Restitution + materialB.Restitution) / 2.0f;
            this.us = (float)Math.Sqrt(materialA.us * materialA.us + materialB.us * materialB.us);
            this.uk = (float)Math.Sqrt(materialA.uk * materialA.uk + materialB.uk * materialB.uk);
        }
        public void PositionalCorrection()
        {
            const float k_percent = 0.2f;
            const float k_slop = 0.01f;

            Vector2 correction = (Math.Max(this.Penetration - k_slop, 0.0f) /
                (this.A.InverseMass + this.B.InverseMass)) * k_percent * this.Normal;

            this.A.Transform.Position -= correction * this.A.InverseMass;
            this.B.Transform.Position += correction * this.B.InverseMass;
        }
        public void ApplyImpluse()
        {
            float im_sum = (this.A.InverseMass + this.B.InverseMass);
            if (PhyMath.EpsilonEqual(im_sum, 0.0f))
            {
                this.A.Velocity = Vector2.Zero;
                this.B.Velocity = Vector2.Zero;
                return;
            }

            for (int i = 0; i < this.ContactPoints.Length; ++i)
            {
                Vector2 ra = this.ContactPoints[i] - this.A.Transform.Position;
                Vector2 rb = this.ContactPoints[i] - this.B.Transform.Position;
                Vector2 romega = Vector2Ext.Cross(this.B.AngularVelocity, rb) - Vector2Ext.Cross(this.A.AngularVelocity, ra);
                Vector2 rv = this.B.Velocity - this.A.Velocity + romega;

                float normal_len = Vector2.Dot(rv, this.Normal);

                if (normal_len > 0)
                    return;

                float ra_cross_normal = Vector2Ext.Cross(ra, this.Normal); 
                float rb_cross_normal = Vector2Ext.Cross(rb, this.Normal);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
                float j = -(this.Restitutoin + 1) * normal_len / (im_sum +
                    ra_cross_normal * ra_cross_normal * this.A.InverseInertia +
                    rb_cross_normal * rb_cross_normal * this.B.InverseInertia) /
                    this.ContactPoints.Length;

                Vector2 impulse = j * this.Normal;

                this.A.ApplyImpulse(-impulse, ra);
                this.B.ApplyImpulse(impulse, rb);

                Vector2 tangent = (rv - (normal_len * this.Normal));
                if (tangent.Length == 0.0f)
                    tangent = Vector2.Zero;
                else
                    tangent.Normalize();

                float ra_cross_tangent = Vector2Ext.Cross(ra, tangent);
                float rb_cross_tangent = Vector2Ext.Cross(rb, tangent);
                float jt = -Vector2.Dot(tangent, rv) / (im_sum +
                    ra_cross_tangent * ra_cross_tangent * this.A.InverseInertia +
                    rb_cross_tangent * rb_cross_tangent * this.B.InverseInertia) /
                    this.ContactPoints.Length;

                if (jt == 0)
                    return;

                Vector2 friction_impulse = tangent * ((Math.Abs(jt) < j * this.us) ? jt : -(j * this.uk));

                this.A.ApplyImpulse(-friction_impulse, ra);
                this.B.ApplyImpulse( friction_impulse, rb);
            }
        }

        public RigidBody A { get; internal set; }
        public RigidBody B { get; internal set; }

        public Vector2[] ContactPoints { get; internal set; }
        public Vector2 Normal { get; internal set; }
        public float Penetration { get; internal set; }
        public float Restitutoin { get; internal set; }
        public float us { get; internal set; }
        public float uk { get; internal set; }
    }
}
