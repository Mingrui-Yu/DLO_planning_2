using UnityEngine;
using System.Collections;
using System.Collections.Generic;

namespace Obi
{
    public class ObiFloatAddBrushMode : ObiBrushMode
    {
        public ObiFloatAddBrushMode(ObiBlueprintFloatProperty property) : base(property) { }

        public override string name
        {
            get { return "Add"; }
        }

        public override void ApplyStamps(ObiBrushBase brush, bool modified)
        {
            var floatProperty = (ObiBlueprintFloatProperty)property;

            for (int i = 0; i < brush.weights.Length; ++i)
            {
                if (!property.Masked(i) && brush.weights[i] > 0)
                {
                    float currentValue = floatProperty.Get(i);
                    float delta = brush.weights[i] * brush.opacity * brush.speed * floatProperty.GetDefault();

                    floatProperty.Set(i, currentValue + delta * (modified ? -1 : 1));
                }
            }
        }
    }
}