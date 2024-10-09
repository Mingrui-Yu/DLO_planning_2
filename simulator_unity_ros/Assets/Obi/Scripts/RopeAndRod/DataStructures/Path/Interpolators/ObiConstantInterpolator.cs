using UnityEngine;
using System.Collections;

namespace Obi
{
    public class ObiConstantInterpolator : ObiInterpolator<int>
    {
        /**
        * constant interpolator
        */
        public int Evaluate(int y0, int y1, int y2, int y3, float mu)
        {
            return mu < 0.5f ? y1 : y2;
        }

        /**
        * derivative of constant value:
        */
        public int EvaluateFirstDerivative(int y0, int y1, int y2, int y3, float mu)
        {
            return 0;
        }


        /**
        * second derivative of constant value:
        */
        public int EvaluateSecondDerivative(int y0, int y1, int y2, int y3, float mu)
        {
            return 0;
        }
    }
}