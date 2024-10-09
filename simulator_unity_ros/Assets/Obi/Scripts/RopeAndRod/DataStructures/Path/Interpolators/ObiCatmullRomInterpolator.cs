using UnityEngine;
using System.Collections;

namespace Obi
{
    public class ObiCatmullRomInterpolator : ObiInterpolator<float>
    {
        /**
        * 1D bezier spline interpolation
        */
        public float Evaluate(float y0, float y1, float y2, float y3, float mu)
        {

            float imu = 1 - mu;
            return imu * imu * imu * y0 +
                3f * imu * imu * mu * y1 +
                3f * imu * mu * mu * y2 +
                mu * mu * mu * y3;

        }

        /**
        * 1D catmull rom spline second derivative
        */
        public float EvaluateFirstDerivative(float y0, float y1, float y2, float y3, float mu)
        {

            float imu = 1 - mu;
            return 3f * imu * imu * (y1 - y0) +
                    6f * imu * mu * (y2 - y1) +
                    3f * mu * mu * (y3 - y2);

        }


        /**
        * 1D catmull rom spline second derivative
        */
        public float EvaluateSecondDerivative(float y0, float y1, float y2, float y3, float mu)
        {

            float imu = 1 - mu;
            return 3f * imu * imu * (y1 - y0) +
                    6f * imu * mu * (y2 - y1) +
                    3f * mu * mu * (y3 - y2);

        }
    }
}