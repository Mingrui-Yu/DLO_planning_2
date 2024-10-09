Shader "Obi/PropertyGradientMaterial" {

	Properties {
        _MainTex ("Base (RGB)", 2D) = "white" {}
    }

	SubShader { 

		Pass {
       
			Cull Back 
			Fog { Mode Off }

            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag

            #include "UnityCG.cginc"

			struct vin{
				float4 vertex   : POSITION;
				fixed4 color    : COLOR;
				float2 texcoord  : TEXCOORD0;
			};

            struct v2f {
                float4 pos: POSITION;
                fixed4 color    : COLOR;
				float2 texcoord  : TEXCOORD0;
            };

			sampler2D _MainTex;

            v2f vert(vin v) {
                v2f o;
                o.pos = UnityObjectToClipPos (v.vertex);
                o.texcoord = v.texcoord;
				o.color = v.color;
                return o;
            }

            fixed4 frag(v2f i) : SV_Target {
                return i.color;
            }

            ENDCG
        }

        Pass {

            Offset 0, -100
            Cull Back 
            ZWrite Off
            Blend SrcAlpha OneMinusSrcAlpha
            Fog { Mode Off }

            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag

            #include "UnityCG.cginc"

            struct vin{
                float4 vertex   : POSITION;
                fixed4 color    : COLOR;
                float2 texcoord  : TEXCOORD0;
            };

            struct v2f {
                float4 pos: POSITION;
                fixed4 color    : COLOR;
                float2 texcoord  : TEXCOORD0;
            };

            sampler2D _MainTex;

            v2f vert(vin v) {
                v2f o;
                o.pos = UnityObjectToClipPos (v.vertex);
                o.texcoord = v.texcoord;
                o.color = v.color;
                return o;
            }

            fixed4 frag(v2f i) : SV_Target {
                return i.color;
            }

            ENDCG
        }
 
	} 
}

