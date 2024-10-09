// Upgrade NOTE: replaced 'mul(UNITY_MATRIX_MVP,*)' with 'UnityObjectToClipPos(*)'

Shader "Particles/Alpha Blended Premultiply Lit" {
Properties {
	_MainTex ("Particle Texture", 2D) = "white" {}
	_InvFade ("Soft Particles Factor", Range(0.01,3.0)) = 1.0
}

Category {
	Tags {"Queue"="Transparent" "IgnoreProjector"="True" "RenderType"="Transparent" "LightMode"="ForwardBase"}
	Blend One OneMinusSrcAlpha 
	ColorMask RGB
	Cull Off ZWrite Off

	SubShader {
		Pass {
		
			CGPROGRAM
			#pragma vertex vert
			#pragma fragment frag
			#pragma multi_compile_particles
			#pragma target 3.0

			#include "UnityCG.cginc"
			#include "Lighting.cginc"

            // compile shader into multiple variants, with and without shadows
            // (we don't care about any lightmaps yet, so skip these variants)
            #pragma multi_compile_fwdbase nolightmap nodirlightmap nodynlightmap novertexlight

			sampler2D _MainTex;
			sampler2D _ShadowMapTexture;
			fixed4 _TintColor;
			
			struct appdata_t {
				float4 pos : POSITION;
				float4 normal: NORMAL;
				fixed4 color : COLOR;
				float2 texcoord : TEXCOORD0;
			};

			struct v2f {
				float4 pos : SV_POSITION;
				fixed4 color : COLOR;
				fixed3 diff : COLOR1;
				float2 texcoord : TEXCOORD0;
				//#ifdef SOFTPARTICLES_ON
				float4 projPos : TEXCOORD1;
				//#endif
			};
			
			float4 _MainTex_ST;

			v2f vert (appdata_t v)
			{
				v2f o;
				o.pos = UnityObjectToClipPos(v.pos);
				o.projPos = ComputeScreenPos (o.pos);
				#ifdef SOFTPARTICLES_ON
				//o.projPos = ComputeScreenPos (o.pos);
				COMPUTE_EYEDEPTH(o.projPos.z);
				#endif

				o.color = v.color;
				o.texcoord = TRANSFORM_TEX(v.texcoord,_MainTex);

				// do shadowmapping here:

				float4 f = UNITY_PROJ_COORD(o.projPos);

				half shadow = tex2Dlod(_ShadowMapTexture, float4(f.xy/f.w,0,0));
				//shadow = f.z/f.w > shadow?0:1;//_LightShadowData.r + shadow * (1-_LightShadowData.r);

				half3 worldNormal = UnityObjectToWorldNormal(v.normal);
                half nl = max(0, dot(worldNormal, _WorldSpaceLightPos0.xyz));
                o.diff = nl * _LightColor0 * shadow + ShadeSH9(half4(worldNormal,1));

				return o;
			}

			sampler2D_float _CameraDepthTexture;
			float _InvFade;
			
			fixed4 frag (v2f i) : SV_Target
			{
				#ifdef SOFTPARTICLES_ON
				float sceneZ = LinearEyeDepth (SAMPLE_DEPTH_TEXTURE_PROJ(_CameraDepthTexture, UNITY_PROJ_COORD(i.projPos)));
				float partZ = i.projPos.z;
				float fade = saturate (_InvFade * (sceneZ-partZ));
				i.color.a *= fade;
				#endif

				i.color.rgb *= i.diff;
				
				return i.color * tex2D(_MainTex, i.texcoord) * i.color.a;
			}
			ENDCG 
		}

	UsePass "Legacy Shaders/VertexLit/SHADOWCASTER"
	}

}
}
