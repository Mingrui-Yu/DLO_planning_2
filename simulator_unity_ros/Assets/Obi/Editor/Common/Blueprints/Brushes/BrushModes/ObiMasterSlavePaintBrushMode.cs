using UnityEngine;
using System.Collections;
using System.Collections.Generic;

namespace Obi
{
    public class ObiMasterSlavePaintBrushMode : ObiBrushMode
    {
        public ObiMasterSlavePaintBrushMode(ObiBlueprintIntProperty property) : base(property) { }

        public override string name
        {
            get { return "Master/Slave paint"; }
        }

        public override void ApplyStamps(ObiBrushBase brush, bool modified)
        {
            var intProperty = (ObiBlueprintIntProperty)property;

            for (int i = 0; i < brush.weights.Length; ++i)
            {
                if (!property.Masked(i) && brush.weights[i] > (1 - brush.opacity))
                {
                    int currentValue = intProperty.Get(i);

                    if (modified)
                        currentValue &= ~(int)(1 << intProperty.GetDefault());
                    else currentValue |= (int)(1 << intProperty.GetDefault());

                    intProperty.Set(i, currentValue);
                }
            }
        }
    }
}