#ifndef OBILIGHTINGBUILTURP_INCLUDED
#define OBILIGHTINGBUILTURP_INCLUDED

#include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"
#include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Lighting.hlsl"
#include "Packages/com.unity.render-pipelines.universal/Shaders/LitInput.hlsl"

half3 SampleSphereAmbient(float3 eyeNormal)
{
    half3 worldNormal = mul(UNITY_MATRIX_I_V,half4(eyeNormal,0.0));
    return SampleSH(worldNormal);
}

float3 ObjSpaceLightDir(in float4 modelPos)
{
    float3 lightPos = mul(unity_WorldToObject,_MainLightPosition).xyz;
    float3 lightVector = lightPos.xyz - modelPos * _MainLightPosition.w;
    return lightVector;
}

float3 WorldSpaceLightDir(in float4 modelPos)
{
    float3 vertexPos = mul(unity_ObjectToWorld, modelPos).xyz;
    float3 lightVector = _MainLightPosition.xyz - vertexPos * _MainLightPosition.w;
    return lightVector;
}

half Attenuation(float3 eyePos)
{
    half3 worldPos = mul(UNITY_MATRIX_I_V,half4(eyePos,1.0));
    float4 shadowCoord = TransformWorldToShadowCoord(worldPos);
    Light mainLight = GetMainLight(shadowCoord);
    return mainLight.shadowAttenuation * mainLight.distanceAttenuation;
}

#endif
