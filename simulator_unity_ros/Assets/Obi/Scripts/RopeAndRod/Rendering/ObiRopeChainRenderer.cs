using System;
using System.Collections.Generic;
using UnityEngine;
using Unity.Profiling;

namespace Obi
{
    [AddComponentMenu("Physics/Obi/Obi Rope Chain Renderer", 885)]
    [ExecuteInEditMode]
    public class ObiRopeChainRenderer : MonoBehaviour
    {
        static ProfilerMarker m_UpdateChainRopeRendererChunksPerfMarker = new ProfilerMarker("UpdateChainRopeRenderer");

        [HideInInspector] [SerializeField] public List<GameObject> linkInstances = new List<GameObject>();

        [SerializeProperty("RandomizeLinks")]
        [SerializeField] private bool randomizeLinks = false;

        public Vector3 linkScale = Vector3.one;     /**< Scale of chain links.*/
        public List<GameObject> linkPrefabs = new List<GameObject>();

        [Range(0, 1)]
        public float twistAnchor = 0;               /**< Normalized position of twisting origin along rope.*/

        public float sectionTwist = 0;              /**< Amount of twist applied to each section, in degrees.*/

        ObiPathFrame frame = new ObiPathFrame();

        void Awake()
        {
            ClearChainLinkInstances();
        }

        public bool RandomizeLinks
        {
            get { return randomizeLinks; }
            set
            {
                if (value != randomizeLinks)
                {
                    randomizeLinks = value;
                    CreateChainLinkInstances(GetComponent<ObiRopeBase>());
                }
            }
        }

        void OnEnable()
        {
            GetComponent<ObiRopeBase>().OnInterpolate += UpdateRenderer;
        }

        void OnDisable()
        {
            GetComponent<ObiRopeBase>().OnInterpolate -= UpdateRenderer;
            ClearChainLinkInstances();
        }

        /**
         * Destroys all chain link instances. Used when the chain must be re-created from scratch, and when the actor is disabled/destroyed.
         */
        public void ClearChainLinkInstances()
        {

            if (linkInstances == null)
                return;

            for (int i = 0; i < linkInstances.Count; ++i)
            {
                if (linkInstances[i] != null)
                    GameObject.DestroyImmediate(linkInstances[i]);
            }
            linkInstances.Clear();
        }

        public void CreateChainLinkInstances(ObiRopeBase rope)
        {

            ClearChainLinkInstances();

            if (linkPrefabs.Count > 0)
            {

                for (int i = 0; i < rope.particleCount; ++i)
                {

                    int index = randomizeLinks ? UnityEngine.Random.Range(0, linkPrefabs.Count) : i % linkPrefabs.Count;

                    GameObject linkInstance = null;

                    if (linkPrefabs[index] != null)
                    {
                        linkInstance = GameObject.Instantiate(linkPrefabs[index]);
                        linkInstance.transform.SetParent(rope.transform, false);
                        linkInstance.hideFlags = HideFlags.HideAndDontSave;
                        linkInstance.SetActive(false);
                    }

                    linkInstances.Add(linkInstance);
                }
            }
        }

        public void UpdateRenderer(ObiActor actor)
        {
            using (m_UpdateChainRopeRendererChunksPerfMarker.Auto())
            {
                var rope = actor as ObiRopeBase;

                // In case there are no link prefabs to instantiate:
                if (linkPrefabs.Count == 0)
                    return;

                // Regenerate instances if needed:
                if (linkInstances == null || linkInstances.Count < rope.particleCount)
                {
                    CreateChainLinkInstances(rope);
                }

                var blueprint = rope.sourceBlueprint;
                int elementCount = rope.elements.Count;

                float twist = -sectionTwist * elementCount * twistAnchor;

                //we will define and transport a reference frame along the curve using parallel transport method:
                frame.Reset();
                frame.SetTwist(twist);

                int lastParticle = -1;

                for (int i = 0; i < elementCount; ++i)
                {
                    ObiStructuralElement elm = rope.elements[i];

                    Vector3 pos = rope.GetParticlePosition(elm.particle1);
                    Vector3 nextPos = rope.GetParticlePosition(elm.particle2);
                    Vector3 linkVector = nextPos - pos;
                    Vector3 tangent = linkVector.normalized;

                    if (rope.sourceBlueprint.usesOrientedParticles)
                    {
                        frame.Transport(nextPos, tangent, rope.GetParticleOrientation(elm.particle1) * Vector3.up, twist);
                        twist += sectionTwist;
                    }
                    else
                    {
                        frame.Transport(nextPos, tangent, sectionTwist);
                    }

                    if (linkInstances[i] != null)
                    {
                        linkInstances[i].SetActive(true);
                        Transform linkTransform = linkInstances[i].transform;
                        linkTransform.position = pos + linkVector * 0.5f;
                        linkTransform.localScale = rope.GetParticleMaxRadius(elm.particle1) * 2 * linkScale;
                        linkTransform.rotation = Quaternion.LookRotation(tangent, frame.normal);
                    }

                    lastParticle = elm.particle2;

                }

                for (int i = elementCount; i < linkInstances.Count; ++i)
                {
                    if (linkInstances[i] != null)
                        linkInstances[i].SetActive(false);
                }
            }
        }
    }
}

