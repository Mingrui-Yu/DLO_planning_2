using UnityEngine;
using System.Collections;

namespace Obi
{
    public class ObiCatmullRomInterpolator3D : ObiInterpolator<Vector3>
    {
        private ObiCatmullRomInterpolator interpolator = new ObiCatmullRomInterpolator();

        /**
        * 3D spline interpolation
        */
        public Vector3 Evaluate(Vector3 y0, Vector3 y1, Vector3 y2, Vector3 y3, float mu)
        {

            return new Vector3(interpolator.Evaluate(y0.x, y1.x, y2.x, y3.x, mu),
                               interpolator.Evaluate(y0.y, y1.y, y2.y, y3.y, mu),
                               interpolator.Evaluate(y0.z, y1.z, y2.z, y3.z, mu));

        }

        /**
        * 3D spline first derivative
        */
        public Vector3 EvaluateFirstDerivative(Vector3 y0, Vector3 y1, Vector3 y2, Vector3 y3, float mu)
        {

            return new Vector3(interpolator.EvaluateFirstDerivative(y0.x, y1.x, y2.x, y3.x, mu),
                               interpolator.EvaluateFirstDerivative(y0.y, y1.y, y2.y, y3.y, mu),
                               interpolator.EvaluateFirstDerivative(y0.z, y1.z, y2.z, y3.z, mu));

        }

        /**
        * 3D spline second derivative
        */
        public Vector3 EvaluateSecondDerivative(Vector3 y0, Vector3 y1, Vector3 y2, Vector3 y3, float mu)
        {

            return new Vector3(interpolator.EvaluateSecondDerivative(y0.x, y1.x, y2.x, y3.x, mu),
                               interpolator.EvaluateSecondDerivative(y0.y, y1.y, y2.y, y3.y, mu),
                               interpolator.EvaluateSecondDerivative(y0.z, y1.z, y2.z, y3.z, mu));

        }
    }
}