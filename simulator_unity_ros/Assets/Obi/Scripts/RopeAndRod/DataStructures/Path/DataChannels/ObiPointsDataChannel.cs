using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;

namespace Obi
{
    [Serializable]
    public class ObiPointsDataChannel : ObiPathDataChannel<ObiWingedPoint, Vector3>
    {

        public ObiPointsDataChannel() : base(new ObiCatmullRomInterpolator3D()) { }

        public Vector3 GetTangent(int index)
        {
            int nextCP = (index + 1) % Count;

            var wp1 = this[index];
            var wp2 = this[nextCP];

            return EvaluateFirstDerivative(wp1.position,
                                           wp1.outTangentEndpoint,
                                           wp2.inTangentEndpoint,
                                           wp2.position, 0);
        }

        public Vector3 GetAcceleration(int index)
        {
            int nextCP = (index + 1) % Count;

            var wp1 = this[index];
            var wp2 = this[nextCP];

            return EvaluateSecondDerivative(wp1.position,
                                            wp1.outTangentEndpoint,
                                            wp2.inTangentEndpoint,
                                            wp2.position, 0);
        }

        /**
        * Returns spline position at time mu, with 0<=mu<=1 where 0 is the start of the spline
        * and 1 is the end.
        */
        public Vector3 GetPositionAtMu(bool closed,float mu)
        {
            int cps = Count;
            if (cps >= 2)
            {

                float p;
                int i = GetSpanControlPointAtMu(closed, mu, out p);
                int nextCP = (i + 1) % cps;

                var wp1 = this[i];
                var wp2 = this[nextCP];

                return Evaluate(wp1.position,
                                wp1.outTangentEndpoint,
                                wp2.inTangentEndpoint,
                                wp2.position, p);
            }
            else
            {
                throw new InvalidOperationException("Cannot get position in path because it has zero control points.");
            }

        }

        /**
        * Returns normal tangent vector at time mu, with 0<=mu<=1 where 0 is the start of the spline
        * and 1 is the end.
        */
        public Vector3 GetTangentAtMu(bool closed, float mu)
        {

            int cps = Count;
            if (cps >= 2)
            {
                float p;
                int i = GetSpanControlPointAtMu(closed, mu, out p);
                int nextCP = (i + 1) % cps;

                var wp1 = this[i];
                var wp2 = this[nextCP];

                return EvaluateFirstDerivative(wp1.position,
                                               wp1.outTangentEndpoint,
                                               wp2.inTangentEndpoint,
                                               wp2.position, p);
            }
            else
            {
                throw new InvalidOperationException("Cannot get derivative in path because it has less than 2 control points.");
            }
        }

        /**
        * Returns acceleration at time mu, with 0<=mu<=1 where 0 is the start of the spline
        * and 1 is the end.
        */
        public Vector3 GetAccelerationAtMu(bool closed, float mu)
        {

            int cps = Count;
            if (cps >= 2)
            {
                float p;
                int i = GetSpanControlPointAtMu(closed, mu, out p);
                int nextCP = (i + 1) % cps;

                var wp1 = this[i];
                var wp2 = this[nextCP];

                return EvaluateSecondDerivative(wp1.position,
                                                wp1.outTangentEndpoint,
                                                wp2.inTangentEndpoint,
                                                wp2.position, p);
            }
            else
            {
                throw new InvalidOperationException("Cannot get second derivative in path because it has less than 2 control points.");
            }
        }
    }
}