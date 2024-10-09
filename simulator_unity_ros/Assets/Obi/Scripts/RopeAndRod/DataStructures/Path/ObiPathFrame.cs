using System;
using UnityEngine;

namespace Obi
{
	public struct ObiPathFrame
    {
        public enum Axis
        {
            X = 0,
            Y = 1,
            Z = 2
        }

		public Vector3 position;

        public Vector3 tangent;
        public Vector3 normal;
        public Vector3 binormal;

		public Vector4 color;
        public float thickness;
       
        public ObiPathFrame(Vector3 position, Vector3 tangent, Vector3 normal, Vector3 binormal, Vector4 color, float thickness){
			this.position = position;
			this.normal = normal;
			this.tangent = tangent;
            this.binormal = binormal;
			this.color = color;
            this.thickness = thickness;
		}

        public void Reset()
        {
            position = Vector3.zero;
            tangent = Vector3.forward;
            normal = Vector3.up;
            binormal = Vector3.right;
            color = Color.white;
            thickness = 0;
        }

		public static ObiPathFrame operator +(ObiPathFrame c1, ObiPathFrame c2) 
	    {
            return new ObiPathFrame(c1.position + c2.position,c1.tangent + c2.tangent,c1.normal + c2.normal,c1.binormal + c2.binormal,c1.color + c2.color, c1.thickness + c2.thickness);
	    }

		public static ObiPathFrame operator *(float f,ObiPathFrame c) 
	    {
            return new ObiPathFrame(c.position * f, c.tangent * f, c.normal * f, c.binormal * f,c.color * f, c.thickness * f);
	    }

        public static void WeightedSum(float w1, float w2, float w3, ref ObiPathFrame c1, ref ObiPathFrame c2, ref ObiPathFrame c3, ref ObiPathFrame sum)
        {
            sum.position.x = c1.position.x * w1 + c2.position.x * w2 + c3.position.x * w3;
            sum.position.y = c1.position.y * w1 + c2.position.y * w2 + c3.position.y * w3;
            sum.position.z = c1.position.z * w1 + c2.position.z * w2 + c3.position.z * w3;

            sum.tangent.x = c1.tangent.x * w1 + c2.tangent.x * w2 + c3.tangent.x * w3;
            sum.tangent.y = c1.tangent.y * w1 + c2.tangent.y * w2 + c3.tangent.y * w3;
            sum.tangent.z = c1.tangent.z * w1 + c2.tangent.z * w2 + c3.tangent.z * w3;

            sum.normal.x = c1.normal.x * w1 + c2.normal.x * w2 + c3.normal.x * w3;
            sum.normal.y = c1.normal.y * w1 + c2.normal.y * w2 + c3.normal.y * w3;
            sum.normal.z = c1.normal.z * w1 + c2.normal.z * w2 + c3.normal.z * w3;

            sum.binormal.x = c1.binormal.x * w1 + c2.binormal.x * w2 + c3.binormal.x * w3;
            sum.binormal.y = c1.binormal.y * w1 + c2.binormal.y * w2 + c3.binormal.y * w3;
            sum.binormal.z = c1.binormal.z * w1 + c2.binormal.z * w2 + c3.binormal.z * w3;

            sum.color.x = c1.color.x * w1 + c2.color.x * w2 + c3.color.x * w3;
            sum.color.y = c1.color.y * w1 + c2.color.y * w2 + c3.color.y * w3;
            sum.color.z = c1.color.z * w1 + c2.color.z * w2 + c3.color.z * w3;
            sum.color.w = c1.color.w * w1 + c2.color.w * w2 + c3.color.w * w3;

            sum.thickness = c1.thickness * w1 + c2.thickness * w2 + c3.thickness * w3;
        }

        public void SetTwist(float twist)
        {
            Quaternion twistQ = Quaternion.AngleAxis(twist, tangent);
            normal = twistQ * normal;
            binormal = twistQ * binormal;
        }

        public void SetTwistAndTangent(float twist, Vector3 tangent)
        {
            this.tangent = tangent;
            normal = new Vector3(tangent.y, tangent.x, 0).normalized;
            binormal = Vector3.Cross(normal, tangent);

            Quaternion twistQ = Quaternion.AngleAxis(twist, tangent);
            normal = twistQ * normal;
            binormal = twistQ * binormal;
        }

        public void Transport(ObiPathFrame frame, float twist)
        {
            // Calculate delta rotation:
            Quaternion rotQ = Quaternion.FromToRotation(tangent, frame.tangent);
            Quaternion twistQ = Quaternion.AngleAxis(twist, frame.tangent);
            Quaternion finalQ = twistQ * rotQ;

            // Rotate previous frame axes to obtain the new ones:
            normal = finalQ * normal;
            binormal = finalQ * binormal;
            tangent = frame.tangent;
            position = frame.position;
            thickness = frame.thickness;
            color = frame.color;
        }

        public void Transport(Vector3 newPosition, Vector3 newTangent, float twist)
        {
            // Calculate delta rotation:
            Quaternion rotQ = Quaternion.FromToRotation(tangent, newTangent);
            Quaternion twistQ = Quaternion.AngleAxis(twist, newTangent);
            Quaternion finalQ = twistQ * rotQ;

            // Rotate previous frame axes to obtain the new ones:
            normal = finalQ * normal;
            binormal = finalQ * binormal;
            tangent = newTangent;
            position = newPosition;

        }

        // Transport, hinting the normal.
        public void Transport(Vector3 newPosition, Vector3 newTangent, Vector3 newNormal, float twist)
        {
            normal = Quaternion.AngleAxis(twist, newTangent) * newNormal;
            tangent = newTangent;
            binormal = Vector3.Cross(normal, tangent);
            position = newPosition;
        }

        public Matrix4x4 ToMatrix(Axis mainAxis)
        {
            Matrix4x4 basis = new Matrix4x4();

            int xo = ((int)mainAxis) % 3 * 4;
            int yo = ((int)mainAxis + 1) % 3 * 4;
            int zo = ((int)mainAxis + 2) % 3 * 4;

            basis[xo]     = tangent[0];
            basis[xo + 1] = tangent[1];
            basis[xo + 2] = tangent[2];

            basis[yo]     = binormal[0];
            basis[yo + 1] = binormal[1];
            basis[yo + 2] = binormal[2];

            basis[zo]     = normal[0];
            basis[zo + 1] = normal[1];
            basis[zo + 2] = normal[2];

            return basis;
        }

        public void DebugDraw(float size)
        {
            Debug.DrawRay(position, binormal * size, Color.red);
            Debug.DrawRay(position, normal * size, Color.green);
            Debug.DrawRay(position, tangent * size, Color.blue);
        }
	}
}

