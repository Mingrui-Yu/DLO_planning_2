using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;

namespace Obi{

    [CreateAssetMenu(fileName = "distance field", menuName = "Obi/Distance Field", order = 181)]
	[ExecuteInEditMode]	
	public class ObiDistanceField : ScriptableObject
	{
		[SerializeProperty("InputMesh")]
		[SerializeField] private Mesh input = null;

		[HideInInspector][SerializeField] private float minNodeSize = 0;
		[HideInInspector][SerializeField] private Bounds bounds = new Bounds(); 
		[HideInInspector] public List<DFNode> nodes;		/**< list of distance field nodes*/

		[Range(0.00001f,0.1f)]
		public float maxError = 0.01f;

		[Range(1, 8)]
		public int maxDepth = 5;

		public bool Initialized{
			get{return nodes != null;}
		}

		public Bounds FieldBounds {
			get{return bounds;}
		}

		public float EffectiveSampleSize {
			get{return minNodeSize;}
		}

		public Mesh InputMesh{
			set{
				if (value != input){
					Reset();
					input = value;
				}
			}
			get{return input;}
		}

		public void Reset(){
			nodes = null;
			if (input != null)
				bounds = input.bounds;
		}

		public IEnumerator Generate(){

			Reset();

			if (input == null)
				yield break;

            int[] tris = input.triangles;
            Vector3[] verts = input.vertices;

            nodes = new List<DFNode>();
            var buildingCoroutine = ASDF.Build(maxError, maxDepth, verts, tris, nodes);

            while (buildingCoroutine.MoveNext())
                yield return new CoroutineJob.ProgressInfo("Processed nodes: " + nodes.Count, 1);

            // calculate min node size;
            minNodeSize = float.PositiveInfinity;
            for (int j = 0; j < nodes.Count; ++j)
                minNodeSize = Mathf.Min(minNodeSize, nodes[j].center[3] * 2);

            // get bounds:
            float max = Mathf.Max(bounds.size[0], Mathf.Max(bounds.size[1], bounds.size[2])) + 0.2f;
            bounds.size = new Vector3(max, max, max);

        }

		/**
		 * Return a volume texture containing a representation of this distance field.
		 */
		public Texture3D GetVolumeTexture(int size){

			if (!Initialized)
				return null;
	
			// upper bound of the distance from any point inside the bounds to the surface.
			float maxDist = Mathf.Max(bounds.size.x,bounds.size.y,bounds.size.z);				
	
			float spacingX = bounds.size.x / (float)size;
			float spacingY = bounds.size.y / (float)size;
			float spacingZ = bounds.size.z / (float)size;
	
			Texture3D tex = new Texture3D (size, size, size, TextureFormat.Alpha8, false);
	
			var cols = new Color[size*size*size];
			int idx = 0;
			Color c = Color.black;
	
			for (int z = 0; z < size; ++z)
			{
				for (int y = 0; y < size; ++y)
				{
					for (int x = 0; x < size; ++x, ++idx)
					{
						Vector3 samplePoint = bounds.min + new Vector3(spacingX * x + spacingX*0.5f,
						                                 			   spacingY * y + spacingY*0.5f,
						                                  			   spacingZ * z + spacingZ*0.5f);

                        float distance = ASDF.Sample(nodes,samplePoint);
	
						if (distance >= 0)
							c.a = distance.Remap(0,maxDist*0.1f,0.5f,1);
						else 
							c.a = distance.Remap(-maxDist*0.1f,0,0,0.5f);
	
						cols[idx] = c;
					}
				}
			}
			tex.SetPixels (cols);
			tex.Apply ();
			return tex;
	
		}
	}
}

