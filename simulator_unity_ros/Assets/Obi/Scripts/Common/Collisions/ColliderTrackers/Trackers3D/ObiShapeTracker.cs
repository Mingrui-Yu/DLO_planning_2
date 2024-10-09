using System;
using UnityEngine;

namespace Obi{

	public abstract class ObiShapeTracker
	{
        protected ObiColliderBase source;
		protected Component collider;

		public virtual void Destroy(){
		}

		public abstract bool UpdateIfNeeded ();

	}

}


