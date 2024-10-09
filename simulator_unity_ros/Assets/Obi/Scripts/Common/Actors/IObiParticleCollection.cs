using UnityEngine;
using System.Collections;

namespace Obi
{
    // Interface for classes that hold a collection of particles. Contains method to get common particle properties.
    public interface IObiParticleCollection 
    {
        int particleCount { get; }
        int activeParticleCount { get; }
        bool usesOrientedParticles { get; }

        int GetParticleRuntimeIndex(int index); // returns solver or blueprint index, depending on implementation.
        Vector3 GetParticlePosition(int index);
        Quaternion GetParticleOrientation(int index);
        void GetParticleAnisotropy(int index, ref Vector4 b1, ref Vector4 b2, ref Vector4 b3);
        float GetParticleMaxRadius(int index);
        Color GetParticleColor(int index);
    }
}
