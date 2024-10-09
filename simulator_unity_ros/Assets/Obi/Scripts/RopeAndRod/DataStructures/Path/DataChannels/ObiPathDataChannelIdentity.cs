using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;

namespace Obi
{
    public abstract class ObiPathDataChannelIdentity<T> : ObiPathDataChannel<T,T>
    {
        public ObiPathDataChannelIdentity(ObiInterpolator<T> interpolator) : base(interpolator)
        {
        }

        public T GetFirstDerivative(int index)
        {
            int nextCP = (index + 1) % Count;

            return EvaluateFirstDerivative(this[index],
                                           this[index],
                                           this[nextCP],
                                           this[nextCP], 0);
        }

        public T GetSecondDerivative(int index)
        {
            int nextCP = (index + 1) % Count;

            return EvaluateSecondDerivative(this[index],
                                            this[index],
                                            this[nextCP],
                                            this[nextCP], 0);
        }

        public T GetAtMu(bool closed, float mu)
        {
            int cps = Count;
            if (cps >= 2)
            {
                float p;
                int i = GetSpanControlPointAtMu(closed, mu, out p);
                int nextCP = (i + 1) % cps;

                return Evaluate(this[i],
                                this[i],
                                this[nextCP],
                                this[nextCP], p);
            }
            else
            {
                throw new InvalidOperationException("Cannot get property in path because it has less than 2 control points.");
            }
        }

        public T GetFirstDerivativeAtMu(bool closed, float mu)
        {
            int cps = Count;
            if (cps >= 2)
            {
                float p;
                int i = GetSpanControlPointAtMu(closed, mu, out p);
                int nextCP = (i + 1) % cps;

                return EvaluateFirstDerivative(this[i],
                                               this[i],
                                               this[nextCP],
                                               this[nextCP], p);
            }
            else
            {
                throw new InvalidOperationException("Cannot get derivative in path because it has less than 2 control points.");
            }
        }

        public T GetSecondDerivativeAtMu(bool closed, float mu)
        {
            int cps = Count;
            if (cps >= 2)
            {
                float p;
                int i = GetSpanControlPointAtMu(closed, mu, out p);
                int nextCP = (i + 1) % cps;

                return EvaluateSecondDerivative(this[i],
                                                this[i],
                                                this[nextCP],
                                                this[nextCP], p);
            }
            else
            {
                throw new InvalidOperationException("Cannot get second derivative in path because it has less than 2 control points.");
            }
        }
    }
}