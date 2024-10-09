using UnityEngine;
using System.Collections;
using System.Collections.Generic;

namespace Obi
{
    public class ObiSelectBrushMode : ObiBrushMode
    {
        public ObiSelectBrushMode(ObiBlueprintSelected property) : base(property) { }

        public override string name
        {
            get { return "Select"; }
        }

        public override void ApplyStamps(ObiBrushBase brush, bool modified)
        {
            var selectedProperty = (ObiBlueprintSelected)property;

            for (int i = 0; i < brush.weights.Length; ++i)
            {
                if (brush.weights[i] > 0 && !property.Masked(i))
                    selectedProperty.Set(i,!modified);
            }
        }
    }
}