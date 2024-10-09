#if (OBI_BURST && OBI_MATHEMATICS && OBI_COLLECTIONS)
using System;

public interface IConstraint
{
    int GetParticleCount();
    int GetParticle(int index);
}
#endif