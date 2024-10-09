using UnityEngine;
using System.Collections;
using System.Collections.Generic;

namespace Obi
{
    public class ObiColorSmoothBrushMode : ObiBrushMode
    {
        public ObiColorSmoothBrushMode(ObiBlueprintColorProperty property) : base(property) { }

        public override string name
        {
            get { return "Smooth"; }
        }

        public override bool needsInputValue
        {
            get { return false; }
        }

        public override void ApplyStamps(ObiBrushBase brush, bool modified)
        {
            var colorProperty = (ObiBlueprintColorProperty)property;

            Color averageValue = Color.black;
            float totalWeight = 0;

            for (int i = 0; i < brush.weights.Length; ++i)
            {
                if (!property.Masked(i) && brush.weights[i] > 0)
                {
                    averageValue += colorProperty.Get(i) * brush.weights[i];
                    totalWeight += brush.weights[i];
                }

            }
            averageValue /= totalWeight;

            for (int i = 0; i < brush.weights.Length; ++i)
            {
                if (!property.Masked(i) && brush.weights[i] > 0)
                {
                    Color currentValue = colorProperty.Get(i);
                    Color delta = brush.opacity * brush.speed * (Color.Lerp(currentValue, averageValue, brush.weights[i]) - currentValue);

                    colorProperty.Set(i, currentValue + delta * (modified ? -1 : 1));
                }
            }

        }
    }
}