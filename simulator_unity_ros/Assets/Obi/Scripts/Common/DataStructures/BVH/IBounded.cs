using System;
using UnityEngine;

namespace Obi
{
    public interface IBounded
    {
        Aabb GetBounds();
    }
}
