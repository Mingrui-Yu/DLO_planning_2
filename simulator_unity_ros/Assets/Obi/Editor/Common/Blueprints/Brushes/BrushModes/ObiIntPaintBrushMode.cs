using UnityEngine;
using System.Collections;
using System.Collections.Generic;

namespace Obi
{
    public class ObiIntPaintBrushMode : ObiBrushMode
    {
        public ObiIntPaintBrushMode(ObiBlueprintIntProperty property) : base(property) { }

        public override string name
        {
            get { return "Paint"; }
        }

        public override void ApplyStamps(ObiBrushBase brush, bool modified)
        {
            var intProperty = (ObiBlueprintIntProperty)property;

            for (int i = 0; i < brush.weights.Length; ++i)
            {
                if (!property.Masked(i) && brush.weights[i] > (1 - brush.opacity))
                {
                    intProperty.Set(i, intProperty.GetDefault());
                }
            }
        }
    }
}