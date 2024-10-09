Shader "Obi/PaddingShader"
{
	Properties
	{
		_MainTex ("Texture", 2D) = "white" {}
        _Padding ("Paddding", Int) = 32
	}
	SubShader
	{
		// No culling or depth
		Cull Off ZWrite Off ZTest Always

		Pass
		{
			CGPROGRAM
			#pragma vertex vert
			#pragma fragment frag
			
			#include "UnityCG.cginc"

			struct appdata
			{
				float4 vertex : POSITION;
				float2 uv : TEXCOORD0;
			};

			struct v2f
			{
				float2 uv : TEXCOORD0;
				float4 vertex : SV_POSITION;
			};

			v2f vert (appdata v)
			{
				v2f o;
				o.vertex = UnityObjectToClipPos(v.vertex);
				o.uv = v.uv;
				return o;
			}
			
			sampler2D _MainTex;
            float4 _MainTex_TexelSize;
            int _Padding;
            static const float2 offsets[8] = {float2(-1,0), float2(1,0), float2(0,1), float2(0,-1), float2(-1,1), float2(1,1), float2(1,-1), float2(-1,-1)};

			fixed4 frag (v2f i) : SV_Target
			{
            
                fixed4 sample = tex2D(_MainTex, i.uv);

                if (sample.a > 0) 
                    return sample;

                fixed4 minSample = sample;
                float minDist = 99999999;

                for  (int k = 1; k < _Padding; ++k)
                {
                    for  (int j = 0; j < 8; ++j)
                    {
                        float2 offsetUV = i.uv + offsets[j] * _MainTex_TexelSize.xy * k;
                        fixed4 offsetSample = tex2Dlod(_MainTex, float4(offsetUV.xy,0,0));

                        if (offsetSample.a > 0) 
                        {
                            float dist = length(i.uv - offsetUV);
                            if (dist < minDist)
                            {
                                minDist = dist;
                                minSample = offsetSample;
                            }
                        }
                    }
                }

                return minSample;
			}
			ENDCG
		}
	}
}
