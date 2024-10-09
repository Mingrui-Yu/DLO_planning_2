using UnityEngine;
using System.Collections;

namespace Obi
{
    public class ObiRopeAttach : MonoBehaviour
    {
        public ObiPathSmoother generator;
        [Range(0,1)]
        public float m;

		public void LateUpdate()
		{
            ObiPathFrame section = generator.GetSectionAt(m);
            transform.position = generator.transform.TransformPoint(section.position);
            transform.rotation = generator.transform.rotation * (Quaternion.LookRotation(section.tangent,section.binormal));
		}

	}
}