using UnityEngine;
using System.Collections;

namespace Obi
{
    public class ObiColorInterpolator3D : ObiInterpolator<Color>
    {
        private ObiCatmullRomInterpolator interpolator = new ObiCatmullRomInterpolator();

        /**
        * 3D spline interpolation
        */
        public Color Evaluate(Color y0, Color y1, Color y2, Color y3, float mu)
        {

            return new Color(interpolator.Evaluate(y0.r, y1.r, y2.r, y3.r, mu),
                             interpolator.Evaluate(y0.g, y1.g, y2.g, y3.g, mu),
                             interpolator.Evaluate(y0.b, y1.b, y2.b, y3.b, mu),
                             interpolator.Evaluate(y0.a, y1.a, y2.a, y3.a, mu));

        }

        /**
        * 3D spline first derivative
        */
        public Color EvaluateFirstDerivative(Color y0, Color y1, Color y2, Color y3, float mu)
        {

            return new Color(interpolator.EvaluateFirstDerivative(y0.r, y1.r, y2.r, y3.r, mu),
                             interpolator.EvaluateFirstDerivative(y0.g, y1.g, y2.g, y3.g, mu),
                             interpolator.EvaluateFirstDerivative(y0.b, y1.b, y2.b, y3.b, mu),
                             interpolator.EvaluateFirstDerivative(y0.a, y1.a, y2.a, y3.a, mu));

        }

        /**
        * 3D spline second derivative
        */
        public Color EvaluateSecondDerivative(Color y0, Color y1, Color y2, Color y3, float mu)
        {

            return new Color(interpolator.EvaluateSecondDerivative(y0.r, y1.r, y2.r, y3.r, mu),
                             interpolator.EvaluateSecondDerivative(y0.g, y1.g, y2.g, y3.g, mu),
                             interpolator.EvaluateSecondDerivative(y0.b, y1.b, y2.b, y3.b, mu),
                             interpolator.EvaluateSecondDerivative(y0.a, y1.a, y2.a, y3.a, mu));

        }
    }
}