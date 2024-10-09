// Upgrade NOTE: replaced 'mul(UNITY_MATRIX_MVP,*)' with 'UnityObjectToClipPos(*)'

// Upgrade NOTE: replaced '_World2Object' with 'unity_WorldToObject'

 
Shader "Obi/Distance Field Preview" {
	Properties {
	    _Volume ("Texture", 3D) = "" {}
		_AABBMin("AABB Min",Vector) = (-0.5,-0.5,-0.5)
		_AABBMax("AABB Max",Vector) = (0.5,0.5,0.5)
		_InsideColor("Inside color",Color) = (1,1,1,1)
		_OutsideColor("Outside color",Color) = (0,0,0,1)
		_Absorption("Absorption",Float) = 1.5
		_StepSize("Step size",Float) = 0.01
		_MaxSteps("Max steps",Int) = 300
	}
	SubShader {
	Pass {
	 
	CGPROGRAM
	#pragma vertex vert
	#pragma fragment frag
	#pragma exclude_renderers flash gles
	 
	#include "UnityCG.cginc"
	 
	struct vs_input {
	    float4 vertex : POSITION;
	};
	 
	struct ps_input {
	    float4 pos : SV_POSITION;
	    float3 eyeOrigin : TEXCOORD0;
		float3 eyeDir : TEXCOORD1;
	};
	 
	 
	ps_input vert (vs_input v)
	{
	    ps_input o;
	    o.pos = UnityObjectToClipPos (v.vertex);

		o.eyeOrigin = mul((float3x3)unity_WorldToObject, _WorldSpaceCameraPos); // object space eye origin
		o.eyeDir = -ObjSpaceViewDir(v.vertex);	// object space eye direction
	    return o;
	}
	 
	sampler3D _Volume;
	float3 _AABBMin;
	float3 _AABBMax;
	float _Absorption;
	float _StepSize;
	int _MaxSteps;
	half4 _InsideColor;
	half4 _OutsideColor;

	bool IntersectBox(float3 rayOrigin, float3 rayDir, float3 aabbMin, float3 aabbMax, out float t0, out float t1)
	{
	    float3 invR = 1.0 / rayDir;
	    float3 tbot = invR * (aabbMin-rayOrigin);
	    float3 ttop = invR * (aabbMax-rayOrigin);
	    float3 tmin = min(ttop, tbot);
	    float3 tmax = max(ttop, tbot);
	    float2 t = max(tmin.xx, tmin.yz);
	    t0 = max(t.x, t.y);
	    t = min(tmax.xx, tmax.yz);
	    t1 = min(t.x, t.y);
	    return t0 <= t1;
	}

	 
	float4 frag (ps_input input) : COLOR
	{
		float4 dst = float4(0.0, 0.0, 0.0, 0.0);

		// Calculate ray direction
		float3 eyeDirection = normalize(input.eyeDir);

		//Calculate intersection with bounding box:
		float tnear, tfar;
		if (IntersectBox(input.eyeOrigin,eyeDirection,_AABBMin,_AABBMax,tnear,tfar)){
			if (tnear < 0.0) tnear = 0.0;
	
			//Calculate ray start and stop positions:
			float3 rayStart = input.eyeOrigin + eyeDirection * tnear;
	   		float3 rayStop = input.eyeOrigin + eyeDirection * tfar;
	
			// Transform from object space bounds to texture coordinate space:
			float3 aabbSize = _AABBMax-_AABBMin;
			rayStart = (rayStart-_AABBMin) / aabbSize;
	    	rayStop = (rayStop-_AABBMin) / aabbSize;
	
			// Raytrace:
			float3 pos = rayStart;
	    	float3 step = normalize(rayStop-rayStart) * _StepSize;
	    	float travel = distance(rayStop,rayStart);
	
	    	for (int i=0; i < _MaxSteps && travel > 0.0; ++i, pos += step, travel -= _StepSize) {
	
				float value = tex3Dlod(_Volume, float4(pos,0)).a;
				float4 color;
				if (value > 0.5){ //outside the surface.
					color = _OutsideColor * (1 - value) * 2;
				}else{			  //inside the surface.
					color = _InsideColor * value * 2;
				}
				dst += color * _StepSize * _Absorption;
	
	    	}
		}

        return dst;
	}
	 
	ENDCG
	 
	}
	}
	 
	Fallback "VertexLit"
}