using OpenTK;
using System;

namespace libPE2D
{
    public delegate void CollisionCallback(Manifold m, RigidBody a, RigidBody b);

    internal class Collision
    {
        public static CollisionCallback[,] Dispatch = {
            { CircleVsCircle, CircleVsPolygon },
            { PolygonVsCircle, PolygonVsPolygon },
        };

        static void CircleVsCircle(Manifold m, RigidBody a, RigidBody b)
        {
            Circle A = a.Shape as Circle;
            Circle B = b.Shape as Circle;

            Vector2 distance = b.Transform.Position - a.Transform.Position;
            float d = distance.LengthSquared;
            float r = A.Radius + B.Radius;

            if (d > r * r)
                return;

            m.ContactPoints = new Vector2[1];
            if (d == 0)
            {
                m.Normal = new Vector2(1, 0);
                m.Penetration = A.Radius;
                m.ContactPoints[0] = a.Transform.Position;
            }
            else
            {
                d = (float)Math.Sqrt(d);
                m.Normal = distance / d;
                m.Penetration = r - d;
                m.ContactPoints[0] = a.Transform.Position + m.Normal * A.Radius;
            }
        }
        static void CircleVsPolygon(Manifold m, RigidBody a, RigidBody b)
        {
            Circle A = a.Shape as Circle;
            Polygon B = b.Shape as Polygon;
            
            Vector2 locA = a.Transform.Position - b.Transform.Position;
            float greatest_separation = float.MinValue;
            int i_face = 0;
            
            for (int i = 0; i < B.Vertices.Length; ++i)
            {
                float d = Vector2.Dot(locA - B.mTransVertices[i], B.mTransNormals[i]);

                if (d > A.Radius)
                    return;

                if (d > greatest_separation)
                {
                    greatest_separation = d;
                    i_face = i;
                }
            }

            if (greatest_separation < PhyMath.Epsilon)
            {
                m.Normal = -B.mTransNormals[i_face];
                m.Penetration = A.Radius;
                m.ContactPoints = new Vector2[1] { a.Transform.Position + m.Normal * A.Radius };
                return;
            }
            
            Vector2 faceL = B.mTransVertices[i_face];
            Vector2 faceR = B.mTransVertices[(i_face + 1) % B.Vertices.Length];
            Vector2 dL = locA - faceL;
            Vector2 dR = locA - faceR;

            float d1 = Vector2.Dot(dL, faceR - faceL);
            float d2 = Vector2.Dot(dR, faceL - faceR);

            if (d1 <= 0.0f)
            {
                if (dL.LengthSquared > A.Radius * A.Radius)
                    return;
                m.Normal = -Vector2.Normalize(dL);
                m.Penetration = A.Radius - (locA - faceL).Length;
                m.ContactPoints = new Vector2[1] { faceL + b.Transform.Position };
                return;
            }
            if (d2 <= 0.0f)
            {
                if (dR.LengthSquared > A.Radius * A.Radius)
                    return;
                m.Normal = -Vector2.Normalize(dR);
                m.Penetration = A.Radius - (locA - faceR).Length;
                m.ContactPoints = new Vector2[1] { faceR + b.Transform.Position };
                return;
            }

            if (Vector2.Dot(dL, B.mTransNormals[i_face]) > A.Radius)
                return;
            
            m.Normal = -B.mTransNormals[i_face];
            m.Penetration = A.Radius - greatest_separation;
            m.ContactPoints = new Vector2[1] { a.Transform.Position + m.Normal * A.Radius };
        }

        static float FindGreatestSeparationAxis(out int i_face, Polygon polyA, Polygon polyB)
        {
            float greatest_separation = float.MinValue;

            i_face = 0;
            for (int i = 0; i < polyA.Vertices.Length; ++i)
            {
                // Vector2 n = Vector2.Transform(polyA.Normals[i], polyA.U * Quaternion.Invert(polyB.U));
                // Vector2 s = polyB.GetSupport(-n);
                // Vector2 v = Vector2.Transform(polyA.Vertices[i], polyA.U) + polyA.Body.Transform.Position;
                // v -= polyB.Body.Transform.Position;
                // v = Vector2.Transform(v, Quaternion.Invert(polyB.U));
                Vector2 n = polyA.mTransNormals[i];
                Vector2 s = polyB.GetSupport(-n);
                Vector2 v = polyA.mTransVertices[i] + polyA.Body.Transform.Position - polyB.Body.Transform.Position;

                float d = Vector2.Dot(n, s - v);

                if (d > greatest_separation)
                {
                    i_face = i;
                    greatest_separation = d;
                }
            }
            return greatest_separation;
        }
        static void FindIncidentFace(ref Vector2[] face, Polygon poly_ref, Polygon poly_inc, int i_ref)
        {
            Vector2 n = poly_ref.mTransNormals[i_ref];
            int i_inc = 0;
            float smallest_dot = float.MaxValue;
            for (int i = 0; i < poly_inc.Vertices.Length; ++i)
            {
                float dot = Vector2.Dot(n, poly_inc.mTransNormals[i]);
                if (dot < smallest_dot)
                {
                    smallest_dot = dot;
                    i_inc = i;
                }
            }
            
            face[0] = poly_inc.mTransVertices[i_inc] + poly_inc.Body.Transform.Position;
            face[1] = poly_inc.mTransVertices[(i_inc+1)%poly_inc.Vertices.Length] + poly_inc.Body.Transform.Position;
        }
        static int Clip(Vector2 n, float offset, ref Vector2[] face)
        {
            int clip_size = 0;
            Vector2[] temp = new Vector2[2] {
                face[0],
                face[1]
            };
            float d1 = Vector2.Dot(n, face[0]) - offset;
            float d2 = Vector2.Dot(n, face[1]) - offset;

            if (d1 <= 0.0f)
                temp[clip_size++] = face[0];
            if (d2 <= 0.0f)
                temp[clip_size++] = face[1];

            if (d1 * d2 < 0.0f) 
            {
                float alpha = d1 / (d1 - d2);
                temp[clip_size++] = face[0] + alpha * (face[1] - face[0]);
            }

            face[0] = temp[0];
            face[1] = temp[1];

            return clip_size;
        }

        static void PolygonVsPolygon(Manifold m, RigidBody a, RigidBody b)
        {
            Polygon A = a.Shape as Polygon;
            Polygon B = b.Shape as Polygon;
            
            int faceA;
            int faceB;

            float penetrationA = FindGreatestSeparationAxis(out faceA, A, B);
            if (penetrationA >= PhyMath.Epsilon)
                return;

            float penetrationB = FindGreatestSeparationAxis(out faceB, B, A);
            if (penetrationB >= PhyMath.Epsilon)
                return;

            bool flip = penetrationB > penetrationA;
            int face_ref = flip ? faceB : faceA;
            Polygon poly_ref = flip ? B : A;
            Polygon poly_inc = flip ? A : B;

            Vector2[] inc_face = new Vector2[2];
            FindIncidentFace(ref inc_face, poly_ref, poly_inc, face_ref);
            
            Vector2 v1 = poly_ref.mTransVertices[face_ref] + poly_ref.Body.Transform.Position;
            Vector2 v2 = poly_ref.mTransVertices[(face_ref + 1) % poly_ref.Vertices.Length] + poly_ref.Body.Transform.Position;

            Vector2 face_side_normal = Vector2.Normalize(v2 - v1);
            Vector2 face_normal = new Vector2(face_side_normal.Y, -face_side_normal.X);

            float refC = Vector2.Dot(face_normal, v1);
            float refL = -Vector2.Dot(face_side_normal, v1);
            float refR = Vector2.Dot(face_side_normal, v2);

            if (Clip(-face_side_normal, refL, ref inc_face) < 2)
                return;

            if (Clip(face_side_normal, refR, ref inc_face) < 2)
                return;

            int cp = 0;

            m.Normal = flip ? -face_normal : face_normal;
            m.Penetration = 0;

            Vector2[] contact = new Vector2[2];
            for (int i = 0; i < 2; ++i)
            {
                float separation = Vector2.Dot(face_normal, inc_face[i]) - refC;
                if (separation <= 0.0f)
                {
                    contact[cp++] = inc_face[i];
                    m.Penetration += -separation;
                }
            }
            if (cp == 0)
                return;
            m.Penetration /= cp;
            m.ContactPoints = new Vector2[cp];
            Array.Copy(contact, m.ContactPoints, cp);

        }
        static void PolygonVsCircle(Manifold m, RigidBody a, RigidBody b)
        {
            CircleVsPolygon(m, b, a);
            m.Normal = -m.Normal;
        }
    }
}
