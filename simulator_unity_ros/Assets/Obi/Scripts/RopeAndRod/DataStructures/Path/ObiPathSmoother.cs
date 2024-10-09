using UnityEngine;
using Unity.Profiling;
using System;
using System.Collections;
using System.Collections.Generic;

namespace Obi
{
    [ExecuteInEditMode]
    [RequireComponent(typeof(ObiRopeBase))]
    public class ObiPathSmoother : MonoBehaviour
    {
        static ProfilerMarker m_AllocateRawChunksPerfMarker = new ProfilerMarker("AllocateRawChunks");
        static ProfilerMarker m_GenerateSmoothChunksPerfMarker = new ProfilerMarker("GenerateSmoothChunks");

        private Matrix4x4 w2l;
        private Quaternion w2lRotation;

        [Range(0, 1)]
        [Tooltip("Curvature threshold below which the path will be decimated. A value of 0 won't apply any decimation. As you increase the value, decimation will become more aggresive.")]
        public float decimation = 0;

        [Range(0, 3)]
        [Tooltip("Smoothing iterations applied to the path. A smoothing value of 0 won't perform any smoothing at all. Note that smoothing is applied after decimation.")]
        public uint smoothing = 0;

        [Tooltip("Twist in degrees applied to each sucessive path section.")]
        public float twist = 0;

        public event ObiActor.ActorCallback OnCurveGenerated;

        protected float smoothLength = 0;
        protected int smoothSections = 0;

        [HideInInspector] public ObiList<ObiList<ObiPathFrame>> rawChunks = new ObiList<ObiList<ObiPathFrame>>();
        [HideInInspector] public ObiList<ObiList<ObiPathFrame>> smoothChunks = new ObiList<ObiList<ObiPathFrame>>();
        private Stack<Vector2Int> stack = new Stack<Vector2Int>();

        public float SmoothLength
        {
            get { return smoothLength; }
        }

        public float SmoothSections
        {
            get { return smoothSections; }
        }

        private void OnEnable()
        {
            GetComponent<ObiRopeBase>().OnInterpolate += Actor_OnInterpolate;
        }

        private void OnDisable()
        {
            GetComponent<ObiRopeBase>().OnInterpolate -= Actor_OnInterpolate;
        }

        void Actor_OnInterpolate(ObiActor actor)
        {
            GenerateSmoothChunks(((ObiRopeBase)actor), smoothing);

            if (OnCurveGenerated != null)
                OnCurveGenerated(actor);
        }

        private void AllocateChunk(int sections)
        {
            if (sections > 1)
            {

                if (rawChunks.Data[rawChunks.Count] == null)
                {
                    rawChunks.Data[rawChunks.Count] = new ObiList<ObiPathFrame>();
                    smoothChunks.Data[smoothChunks.Count] = new ObiList<ObiPathFrame>();
                }

                rawChunks.Data[rawChunks.Count].SetCount(sections);

                rawChunks.SetCount(rawChunks.Count + 1);
                smoothChunks.SetCount(smoothChunks.Count + 1);
            }
        }

        private float CalculateChunkLength(ObiList<ObiPathFrame> chunk)
        {
            float length = 0;
            for (int i = 1; i < chunk.Count; ++i)
                length += Vector3.Distance(chunk[i].position, chunk[i - 1].position);
            return length;
        }

        /**
         * Generates raw curve chunks from the rope description.
         */
        private void AllocateRawChunks(ObiRopeBase actor)
        {
            using (m_AllocateRawChunksPerfMarker.Auto())
            {
                rawChunks.Clear();

                if (actor.path == null)
                    return;

                // Count particles for each chunk.
                int particles = 0;
                for (int i = 0; i < actor.elements.Count; ++i)
                {
                    particles++;
                    // At discontinuities, start a new chunk.
                    if (i < actor.elements.Count - 1 && actor.elements[i].particle2 != actor.elements[i + 1].particle1)
                    {
                        AllocateChunk(++particles);
                        particles = 0;
                    }
                }
                AllocateChunk(++particles);
            }
        }

        private void PathFrameFromParticle(ObiRopeBase actor, ref ObiPathFrame frame, int particleIndex)
        {
            // Update current frame values from particles:
            frame.position = w2l.MultiplyPoint3x4(actor.GetParticlePosition(particleIndex));
            frame.thickness = actor.GetParticleMaxRadius(particleIndex);
            frame.color = actor.GetParticleColor(particleIndex);

            // Use particle orientation if possible:
            if (actor.usesOrientedParticles)
            {
                Quaternion orientation = w2lRotation * Quaternion.SlerpUnclamped(actor.GetParticleOrientation(particleIndex), actor.GetParticleOrientation(Mathf.Max(0, particleIndex - 1)), 0.5f);
                frame.normal = orientation * Vector3.up;
                frame.binormal = orientation * Vector3.right;
                frame.tangent = orientation * Vector3.forward;
            }
        }

        /**
         * Generates smooth curve chunks.
         */
        public void GenerateSmoothChunks(ObiRopeBase actor, uint smoothingLevels)
        {
            using (m_GenerateSmoothChunksPerfMarker.Auto())
            {
                smoothChunks.Clear();
                smoothSections = 0;
                smoothLength = 0;

                if (!Application.isPlaying)
                    actor.RebuildElementsFromConstraints();

                AllocateRawChunks(actor);

                w2l = actor.transform.worldToLocalMatrix;
                w2lRotation = w2l.rotation;

                // keep track of the first element of each chunk
                int chunkStart = 0;

                ObiPathFrame frame_0 = new ObiPathFrame(); // "next" frame
                ObiPathFrame frame_1 = new ObiPathFrame(); // current frame
                ObiPathFrame frame_2 = new ObiPathFrame(); // previous frame

                // generate curve for each rope chunk:
                for (int i = 0; i < rawChunks.Count; ++i)
                {
                    int elementCount = rawChunks[i].Count - 1;

                    // Initialize frames:
                    frame_0.Reset();
                    frame_1.Reset();
                    frame_2.Reset();

                    PathFrameFromParticle(actor, ref frame_1, actor.elements[chunkStart].particle1);

                    frame_2 = frame_1;

                    for (int m = 1; m <= rawChunks[i].Count; ++m)
                    {

                        int index;
                        if (m >= elementCount)
                            // second particle of last element in the chunk.
                            index = actor.elements[chunkStart + elementCount - 1].particle2;
                        else
                            //first particle of current element.
                            index = actor.elements[chunkStart + m].particle1;

                        // generate curve frame from particle:
                        PathFrameFromParticle(actor, ref frame_0, index);

                        if (actor.usesOrientedParticles)
                        {
                            // copy frame directly.
                            frame_2 = frame_1;
                        }
                        else
                        {
                            // perform parallel transport, using forward / backward average to calculate tangent.
                            frame_1.tangent = ((frame_1.position - frame_2.position) + (frame_0.position - frame_1.position)).normalized;
                            frame_2.Transport(frame_1, twist);
                        }

                        // in case we wrapped around the rope, average first and last frames:
                        if (chunkStart + m > actor.activeParticleCount)
                        {
                            frame_2 = rawChunks[0][0] = 0.5f * frame_2 + 0.5f * rawChunks[0][0];
                        }

                        frame_1 = frame_0;

                        rawChunks[i][m - 1] = frame_2;
                    }

                    // increment chunkStart by the amount of elements in this chunk:
                    chunkStart += elementCount;

                    // adaptive curvature-based decimation:
                    if (Decimate(rawChunks[i], smoothChunks[i], decimation))
                    {
                        // if any decimation took place, swap raw and smooth chunks:
                        var aux = rawChunks[i];
                        rawChunks[i] = smoothChunks[i];
                        smoothChunks[i] = aux;
                    }

                    // get smooth curve points:
                    Chaikin(rawChunks[i], smoothChunks[i], smoothingLevels);

                    // count total curve sections and total curve length:
                    smoothSections += smoothChunks[i].Count - 1;
                    smoothLength += CalculateChunkLength(smoothChunks[i]);
                }
            }
        }

        public ObiPathFrame GetSectionAt(float mu)
        {
            float edgeMu = smoothSections * Mathf.Clamp01(mu);
            int index = (int)edgeMu;
            float sectionMu = edgeMu - index;

            int counter = 0;
            int chunkIndex = -1;
            int indexInChunk = -1;
            for (int i = 0; i < smoothChunks.Count; ++i)
            {
                if (counter + smoothChunks[i].Count > index)
                {
                    chunkIndex = i;
                    indexInChunk = index - counter;
                }
                counter += smoothChunks[i].Count;
            }

            ObiList<ObiPathFrame> chunk = smoothChunks[chunkIndex];
            ObiPathFrame s1 = chunk[indexInChunk];
            ObiPathFrame s2 = chunk[Mathf.Min(indexInChunk + 1, chunk.Count - 1)];

            return (1 - sectionMu) * s1 + sectionMu * s2;
        }

        /**
         * Iterative version of the Ramer-Douglas-Peucker path decimation algorithm. 
         */
        private bool Decimate(ObiList<ObiPathFrame> input, ObiList<ObiPathFrame> output, float threshold)
        {
            // no decimation, no work to do, just return:
            if (threshold < 0.00001f || input.Count < 3)
                return false;

            float scaledThreshold = threshold * threshold * 0.01f;

            stack.Push(new Vector2Int(0, input.Count - 1));
            var bitArray = new BitArray(input.Count, true);

            while (stack.Count > 0)
            {
                var range = stack.Pop();

                float dmax = 0;
                int index = range.x;
                float mu = 0;

                for (int i = index + 1; i < range.y; ++i)
                {
                    if (bitArray[i])
                    {
                        float d = Vector3.SqrMagnitude(ObiUtils.ProjectPointLine(input[i].position, input[range.x].position, input[range.y].position, out mu) - input[i].position);

                        if (d > dmax)
                        {
                            index = i;
                            dmax = d;
                        }
                    }
                }

                if (dmax > scaledThreshold)
                {
                    stack.Push(new Vector2Int(range.x, index));
                    stack.Push(new Vector2Int(index, range.y));
                }
                else
                {
                    for (int i = range.x + 1; i < range.y; ++i)
                        bitArray[i] = false;
                }
            }

            output.Clear();
            for (int i = 0; i < bitArray.Count; ++i)
                if (bitArray[i])
                    output.Add(input[i]);

            return true;
        }

        /** 
        * This method uses a variant of Chainkin's algorithm to produce a smooth curve from a set of control points. It is specially fast
        * because it directly calculates subdivision level k, instead of recursively calculating levels 1..k.
        */
        private void Chaikin(ObiList<ObiPathFrame> input, ObiList<ObiPathFrame> output, uint k)
        {
            // no subdivision levels, no work to do. just copy the input to the output:
            if (k == 0 || input.Count < 3)
            {
                output.SetCount(input.Count);
                for (int i = 0; i < input.Count; ++i)
                    output[i] = input[i];
                return;
            }

            // calculate amount of new points generated by each inner control point:
            int pCount = (int)Mathf.Pow(2, k);

            // precalculate some quantities:
            int n0 = input.Count - 1;
            float twoRaisedToMinusKPlus1 = Mathf.Pow(2, -(k + 1));
            float twoRaisedToMinusK = Mathf.Pow(2, -k);
            float twoRaisedToMinus2K = Mathf.Pow(2, -2 * k);
            float twoRaisedToMinus2KMinus1 = Mathf.Pow(2, -2 * k - 1);

            // allocate ouput:
            output.SetCount((n0 - 1) * pCount + 2);

            // calculate initial curve points:
            output[0] = (0.5f + twoRaisedToMinusKPlus1) * input[0] + (0.5f - twoRaisedToMinusKPlus1) * input[1];
            output[pCount * n0 - pCount + 1] = (0.5f - twoRaisedToMinusKPlus1) * input[n0 - 1] + (0.5f + twoRaisedToMinusKPlus1) * input[n0];

            // calculate internal points:
            for (int j = 1; j <= pCount; ++j)
            {
                // precalculate coefficients:
                float F = 0.5f - twoRaisedToMinusKPlus1 - (j - 1) * (twoRaisedToMinusK - j * twoRaisedToMinus2KMinus1);
                float G = 0.5f + twoRaisedToMinusKPlus1 + (j - 1) * (twoRaisedToMinusK - j * twoRaisedToMinus2K);
                float H = (j - 1) * j * twoRaisedToMinus2KMinus1;

                for (int i = 1; i < n0; ++i)
                    ObiPathFrame.WeightedSum(F, G, H,
                                             ref input.Data[i - 1],
                                             ref input.Data[i],
                                             ref input.Data[i + 1],
                                             ref output.Data[(i - 1) * pCount + j]);
            }

            // make first and last curve points coincide with original points:
            output[0] = input[0];
            output[output.Count - 1] = input[input.Count - 1];
        }

    }
}