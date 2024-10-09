using UnityEngine;
using UnityEditor;
using System.Collections;

namespace Obi
{
    public class ObiBlueprintSelected : ObiBlueprintBoolProperty
    {
        public IObiSelectableParticleProvider provider;
        public ObiBlueprintSelected(IObiSelectableParticleProvider provider)
        {
            this.provider = provider;
        }

        public override string name
        {
            get { return "Selected"; }
        }

        public override bool Get(int index)
        {
            return provider.IsSelected(index);
        }
        public override void Set(int index, bool value)
        {
            provider.SetSelected(index,value);
        }
        public override bool Masked(int index)
        {
            return !provider.Editable(index);
        }
    }
}
