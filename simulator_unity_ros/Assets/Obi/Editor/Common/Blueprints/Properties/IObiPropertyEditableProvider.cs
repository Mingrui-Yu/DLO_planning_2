using UnityEngine;
using System.Collections;

namespace Obi
{
    public interface IObiSelectableParticleProvider
    {
        void SetSelected(int particleIndex, bool selected);
        bool IsSelected(int particleIndex);
        bool Editable(int particleIndex);
    }

}