using UnityEngine;
using System.Collections;

namespace Obi
{
    
    public interface ObiInterpolator<T>
    {
        T Evaluate(T v0, T v1, T v2, T v3, float mu);
        T EvaluateFirstDerivative(T v0, T v1, T v2, T v3, float mu);
        T EvaluateSecondDerivative(T v0, T v1, T v2, T v3, float mu);
    }
}
