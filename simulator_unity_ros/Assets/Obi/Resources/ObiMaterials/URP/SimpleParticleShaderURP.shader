Shader "Obi/URP/Simple Particles" {

Properties {
	_Color 		("Particle color", Color) = (1,1,1,1)
}

	SubShader { 

        Tags{"RenderPipeline" = "UniversalRenderPipeline"}

		Pass { 

			Name "ParticleFwdBase"
			Tags {"Queue"="Geometry" "IgnoreProjector"="True" "RenderType"="Opaque" "LightMode" = "UniversalForward"}
			Blend SrcAlpha OneMinusSrcAlpha  
			
			HLSLPROGRAM

            #pragma prefer_hlslcc gles
            #pragma exclude_renderers d3d11_9x
            #pragma target 2.0

			#pragma vertex vert
			#pragma fragment frag
			#pragma fragmentoption ARB_precision_hint_fastest

			#pragma multi_compile _ _MAIN_LIGHT_SHADOWS
            #pragma multi_compile _ _MAIN_LIGHT_SHADOWS_CASCADE
            #pragma multi_compile _ _SHADOWS_SOFT

            #include "../ObiUtils.cginc"
            #include "ObiLightingURP.cginc"
            #include "../ObiParticles.cginc"

			float4 _Color;
			float4 _LightColor0; 

			struct vin{
				float4 vertex   : POSITION;
				float3 corner   : NORMAL;
				float4 color    : COLOR;
				float4 t0 : TEXCOORD0; // ellipsoid t1 vector
			};

			struct v2f
			{
				float4 pos   : POSITION;
				float4 color    : COLOR;
				float2 texcoord  : TEXCOORD0;
				float3 lightDir : TEXCOORD1;
			};

			v2f vert(vin v)
			{ 
				v2f o;
		
				// particle positions are passed in world space, no need to use modelview matrix, just view.
				float radius = v.t0.w * _RadiusScale;
				float4 viewpos = mul(UNITY_MATRIX_V, v.vertex) + float4(v.corner.x, v.corner.y, 0, 0) * radius; // multiply by size.
				o.pos = mul(UNITY_MATRIX_P, viewpos);
				o.texcoord = float3(v.corner.x*0.5+0.5, v.corner.y*0.5+0.5, radius);
				o.color = v.color * _Color;

				o.lightDir = mul ((float3x3)UNITY_MATRIX_MV, ObjSpaceLightDir(v.vertex));

				return o;
			} 

			float4 frag(v2f i) : SV_Target
			{
				// generate sphere normals:
				float3 n = BillboardSphereNormals(i.texcoord);

                // simple lighting: ambient
                half3 amb = SampleSphereAmbient(n);

				// simple lighting: diffuse
		   	    float ndotl = saturate( dot( n, normalize(i.lightDir) ) );

				// final lit color:
				return float4(i.color.rgb * (_LightColor0 * ndotl + amb),i.color.a);
			}
			 
			ENDHLSL

		} 

		Pass {
        	Name "ShadowCaster"
		        Tags { "LightMode" = "ShadowCaster" }
		        Offset 1, 1
		       
		        Fog {Mode Off}
		        ZWrite On ZTest LEqual
		 
				HLSLPROGRAM
                #pragma prefer_hlslcc gles
                #pragma exclude_renderers d3d11_9x
                #pragma target 2.0

				#pragma vertex vert
				#pragma fragment frag
				#pragma fragmentoption ARB_precision_hint_fastest

				#pragma multi_compile_shadowcaster

				#include "../ObiUtils.cginc"
                #include "ObiLightingURP.cginc"
                #include "../ObiParticles.cginc"
				 
				struct vin{
					float4 vertex   : POSITION;
					float3 corner   : NORMAL;
					float4 t0 : TEXCOORD0; // ellipsoid t1 vector
				};

				struct v2f {
					float4 pos   : POSITION;
				    float3 texcoord : TEXCOORD0;
				};
				 
				v2f vert( vin v )
				{
				    v2f o;

					float radius = v.t0.w * _RadiusScale;
					float4 viewpos = mul(UNITY_MATRIX_V, v.vertex) + float4(v.corner.x, v.corner.y, 0, 0) * radius;
					o.pos = mul(UNITY_MATRIX_P, viewpos);
					o.texcoord = float3(v.corner.x*0.5+0.5, v.corner.y*0.5+0.5, radius);
				    return o;
				}
				 
				float4 frag( v2f i ) : SV_Target
				{
					float3 n = BillboardSphereNormals(i.texcoord);

					return float4(0,0,0,0);
				}
				ENDHLSL
		 
		    }

	} 
}

