using UnityEngine;
using System.Collections;
using System.Collections.Generic;

namespace Obi
{
    public class ObiColorPaintBrushMode : ObiBrushMode
    {
        public ObiColorPaintBrushMode(ObiBlueprintColorProperty property) : base(property) { }

        public override string name
        {
            get { return "Paint"; }
        }

        public override void ApplyStamps(ObiBrushBase brush, bool modified)
        {
            var colorProperty = (ObiBlueprintColorProperty)property;

            for (int i = 0; i < brush.weights.Length; ++i)
            {
                if (!property.Masked(i) && brush.weights[i] > 0)
                {
                    Color currentValue = colorProperty.Get(i);
                    Color delta = brush.weights[i] * brush.opacity * brush.speed * (colorProperty.GetDefault() - currentValue);

                    colorProperty.Set(i, currentValue + delta * (modified ? -1 : 1));
                }
            }
        }
    }
}