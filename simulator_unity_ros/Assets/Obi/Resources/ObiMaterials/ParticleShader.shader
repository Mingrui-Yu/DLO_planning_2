Shader "Obi/Particles" {

Properties {
	_Color 		("Particle color", Color) = (1,1,1,1)
	_RadiusScale("Radius scale",float) = 1
}

	SubShader { 

		Pass { 

			Name "ParticleFwdBase"
			Tags {"Queue"="Geometry" "IgnoreProjector"="True" "RenderType"="Opaque"}
			Blend SrcAlpha OneMinusSrcAlpha  
			
			CGPROGRAM
			#pragma vertex vert
			#pragma fragment frag
			#pragma fragmentoption ARB_precision_hint_fastest

			#pragma multi_compile_fwdbase nolightmap

		    #include "ObiEllipsoids.cginc"
            #include "ObiUtils.cginc"
            #include "ObiLightingBuiltIn.cginc"

			fixed4 _Color;
			fixed4 _LightColor0; 

			struct vin{
				float4 vertex   : POSITION;
				float3 corner   : NORMAL;
				fixed4 color    : COLOR;
			
				float4 t0 : TEXCOORD0; // ellipsoid t1 vector
			    float4 t1 : TEXCOORD1; // ellipsoid t2 vector
			    float4 t2 : TEXCOORD2; // ellipsoid t3 vector
			};
			
			struct v2f
			{
				float4 pos   : SV_POSITION;
				fixed4 color    : COLOR;
				float4 mapping  : TEXCOORD0;
				float3 viewRay : TEXCOORD1;
				float3 lightDir : TEXCOORD2;
				float3 a2 : TEXCOORD3;
				float3 a3 : TEXCOORD4;
				LIGHTING_COORDS(5,6)
			};

            struct fout 
            {
                half4 color : SV_Target;
                float depth : SV_Depth;
            };

			v2f vert(vin v)
			{ 
				float3x3 P, IP;
				BuildParameterSpaceMatrices(v.t0,v.t1,v.t2,P,IP);
			
				float3 worldPos;
				float3 view;
				float3 eye;
				float radius = BuildEllipsoidBillboard(v.vertex,v.corner,P,IP,worldPos,view,eye);
			
				v2f o;
				o.pos = mul(UNITY_MATRIX_VP, float4(worldPos,v.vertex.w));
				o.mapping = float4(v.corner.xy,1/length(eye),radius); // A[1]
				o.viewRay = mul((float3x3)UNITY_MATRIX_V,view); 	  // A[0]
				o.color = v.color * _Color;
			
				BuildAuxiliaryNormalVectors(v.vertex,worldPos,view,P,IP,o.a2,o.a3);
						
				o.lightDir = mul((float3x3)UNITY_MATRIX_MV, ObjSpaceLightDir(v.vertex));
				TRANSFER_VERTEX_TO_FRAGMENT(o);
			
				return o;
			}

			fout frag(v2f i) 
			{
				fout fo;

				fo.color =  half4(0,0,0,i.color.a); 

				// generate sphere normals:
				float3 p,n;
				IntersectEllipsoid(i.viewRay,i.mapping, i.a2,i.a3,p, n);

				// clip space position:
				float4 pos = mul(UNITY_MATRIX_P,float4(p,1.0));

				// simple lighting: ambient
				half3 amb = SampleSphereAmbient(n,p);

				// simple lighting: diffuse
		   	 	float ndotl = saturate( dot( n, normalize(i.lightDir) ) );
				UNITY_LIGHT_ATTENUATION(atten,i,0);

				// final lit color:
				fo.color.rgb = i.color * (_LightColor0 * ndotl * atten + amb);

				// normalized device coordinates:
				fo.depth = pos.z/pos.w;

				// in openGL calculated depth range is <-1,1> map it to <0,1>
				#if SHADER_API_OPENGL || SHADER_API_GLCORE || SHADER_API_GLES || SHADER_API_GLES3	 
					fo.depth = 0.5*fo.depth + 0.5;
				#endif

				return fo;
			}
			 
			ENDCG

		} 

		Pass {
        	Name "ShadowCaster"
		        Tags { "LightMode" = "ShadowCaster" }
		        Offset 1, 1
		       
		        Fog {Mode Off}
		        ZWrite On ZTest LEqual
		 
				CGPROGRAM

				#pragma vertex ellipsoidShadowVS
				#pragma fragment frag
				#pragma fragmentoption ARB_precision_hint_fastest

				#pragma multi_compile_shadowcaster nolightmap

				#include "ObiEllipsoids.cginc"
                #include "ObiUtils.cginc"
                #include "ObiLightingBuiltIn.cginc"
				 
				sampler3D  _DitherMaskLOD;
				fixed4 _Color;

				struct vin{
					float4 vertex   : POSITION;
					float3 corner   : NORMAL;
					fixed4 color    : COLOR;
				
					float4 t0 : TEXCOORD0; // ellipsoid t1 vector
				    float4 t1 : TEXCOORD1; // ellipsoid t2 vector
				    float4 t2 : TEXCOORD2; // ellipsoid t3 vector
				};
				
				struct v2f {
					fixed4 color    : COLOR;
					float4 mapping  : TEXCOORD0;
					float3 viewRay : TEXCOORD1;
				};

                struct fout 
                {
                    half4 color : SV_Target;
                    float depth : SV_Depth;
                };

				v2f ellipsoidShadowVS( vin v , out float4 outpos : SV_POSITION )// clip space position output
				{
					float3x3 P, IP;
					BuildParameterSpaceMatrices(v.t0,v.t1,v.t2,P,IP);
				
					float3 worldPos;
					float3 view;
					float3 eye;
					float radius = BuildEllipsoidBillboard(v.vertex,v.corner,P,IP,worldPos,view,eye);
				
					v2f o;
					outpos = mul(UNITY_MATRIX_VP, float4(worldPos,v.vertex.w));
					o.mapping = float4(v.corner.xy,1/length(eye),radius); // A[1]
					o.viewRay = mul((float3x3)UNITY_MATRIX_V,view); 	  // A[0]
					o.color = v.color * _Color;
					return o;
				}
								 
				fout frag( v2f i , UNITY_VPOS_TYPE vpos : VPOS) 
				{
					fout fo;

					float3 p,n;
					IntersectEllipsoid(i.viewRay,i.mapping, float3(0,0,0),float3(0,0,0),p, n);
	
					// project camera space position.
					float4 pos = UnityApplyLinearShadowBias( mul(UNITY_MATRIX_P,float4(p,1.0)) );

					fo.color = pos.z/pos.w; //similar to what SHADOW_CASTER_FRAGMENT does in case there's no depth buffer.
					fo.depth = pos.z/pos.w; 

					// in openGL calculated depth range is <-1,1> map it to <0,1>
					#if SHADER_API_OPENGL || SHADER_API_GLCORE || SHADER_API_GLES || SHADER_API_GLES3
						fo.depth = fo.depth*0.5+0.5;
					#endif

	                // Use dither mask for alpha blended shadows, based on pixel position xy
	                // and alpha level. Our dither texture is 4x4x16.
	                half alphaRef = tex3D(_DitherMaskLOD, float3(vpos.xy*0.25,i.color.a*0.9375)).a;
	                clip (alphaRef - 0.01);
           
					return fo;
				}
				ENDCG
		 
		    }

	} 
FallBack "Diffuse"
}

