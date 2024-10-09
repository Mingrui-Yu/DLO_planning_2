Shader "Obi/EditorParticles"
{
    SubShader
    {
        Blend One OneMinusSrcAlpha
        ZWrite Off
        ZTest always 
        Cull Back 
 
        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
         
            #include "UnityCG.cginc"
 
            struct appdata
            {
                float4 vertex : POSITION;
                float3 corner   : NORMAL;
                fixed4 color    : COLOR;
                float4 t0 : TEXCOORD0; // ellipsoid t1 vector
            };
 
            struct v2f
            {
                float4 pos : SV_POSITION;
                fixed4 color    : COLOR;
                float2 texcoord : TEXCOORD0;
            };
         
            v2f vert (appdata v)
            {
                v2f o;
                
                // particle positions are passed in world space, no need to use modelview matrix, just view.
                float radius = v.t0.w * distance(mul(unity_ObjectToWorld, v.vertex), _WorldSpaceCameraPos);
                float4 viewpos = mul(UNITY_MATRIX_V, v.vertex) + float4(v.corner.x, v.corner.y, 0, 0) * radius;
                o.pos = mul(UNITY_MATRIX_P, viewpos); 

                o.texcoord = float3(v.corner.x*0.5, v.corner.y*0.5, radius);
                o.color = v.color;

                return o;
            }
 
            fixed4 frag (v2f i) : SV_Target
            {
                // antialiased circle:
                float dist = length(i.texcoord);
                float pwidth = fwidth(dist);
                float alpha = i.color.a * saturate((0.5 - dist) / pwidth);
                
                return fixed4(i.color.rgb * alpha, alpha);
            }
            ENDCG
        }
    }
}
