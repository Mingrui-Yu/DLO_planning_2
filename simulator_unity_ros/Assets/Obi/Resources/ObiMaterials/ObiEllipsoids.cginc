#ifndef OBIELLIPSOIDS_INCLUDED
#define OBIELLIPSOIDS_INCLUDED

float _RadiusScale;

// Eye ray origin in world space.
// Works both for orthographic and perspective cameras.
float3 WorldEye(float3 worldPos){
	if ((UNITY_MATRIX_P[3].x == 0.0) && (UNITY_MATRIX_P[3].y == 0.0) && (UNITY_MATRIX_P[3].z == 0.0)){
		return mul(UNITY_MATRIX_I_V,float4(mul(UNITY_MATRIX_V, float4(worldPos,1)).xy,0,1)).xyz;
	}else
		return UNITY_MATRIX_I_V._m03_m13_m23;
}

// Returns visible ellipsoid radius and offset from center, given the eye position in parameter space. 
// Works both for orthographic and perspective cameras.
float VisibleEllipsoidCircleRadius(float3 eye, out float3 m){
	if ((UNITY_MATRIX_P[3].x == 0.0) && (UNITY_MATRIX_P[3].y == 0.0) && (UNITY_MATRIX_P[3].z == 0.0)){
		m = float3(0,0,0);
		return 1;
	}else{
		float t = 1/dot(eye,eye);
		m = t * eye;
		return sqrt(1-t);
	}
}

// Performs accurate raycasting of a spherical impostor.
// Works both for orthographic and perspective cameras.
float IntersectEllipsoid(float3 v, float4 mapping, float3 a2, float3 a3, out float3 eyePos, out float3 eyeNormal)
{
	float r2 = dot(mapping.xy, mapping.xy);
	float iq = 1 - r2/mapping.w;
	clip(iq); // the ray does not intersect the sphere.

	float sqrtiq = sqrt(iq);
	float lambda = 1/(1 + mapping.z * sqrtiq);

	eyePos = lambda * v;
	eyeNormal = normalize(a2 + lambda * a3);

	// return gaussian-falloff thickness.
	return 2 * sqrtiq * exp(-r2*2.0f);
}

void BuildParameterSpaceMatrices(float4 t0, float4 t1, float4 t2, out float3x3 P, out float3x3 IP)
{
	// build 3x3 orientation matrix and its inverse;
	float3x3 IO = float3x3(t0.xyz,t1.xyz,t2.xyz);
	float3x3 O = transpose(IO);

	// build 3x3 scaling matrix and its inverse:
	float3x3 S = float3x3(_RadiusScale*t0.w,0,0,0,_RadiusScale*t1.w,0,0,0,_RadiusScale*t2.w);
	float3x3 IS = float3x3(1/(_RadiusScale*t0.w),0,0,0,1/(_RadiusScale*t1.w),0,0,0,1/(_RadiusScale*t2.w));

	// build 3x3 transformation matrix and its inverse:
	P = mul(mul(O,S),IO);
	IP = mul(mul(O,IS),IO);
}

float BuildEllipsoidBillboard(float3 center, float3 corner, float3x3 P, float3x3 IP, out float3 worldPos, out float3 view, out float3 eye)
{
	// eye position and quad vectors in parameter space:
	eye = mul(IP,WorldEye(center) - center);
	float3 u = normalize(cross(-eye,UNITY_MATRIX_V[1].xyz));
	float3 k = normalize(cross(-eye,u));

	// visible circle radius and offset from center in the direction of the view ray:
	float3 m;
	float radius = VisibleEllipsoidCircleRadius(eye,m);

	// world position of the billboard corner, and view vector to it:
	worldPos = center + mul(P, m) + radius * (mul(P,u)* corner.x + mul(P,k)* corner.y);
	view = worldPos - WorldEye(worldPos);

	return radius;
}

void BuildAuxiliaryNormalVectors(float3 center, float3 worldPos, float3 view, float3x3 P, float3x3 IP, out float3 a2, out float3 a3)
{
	a2 = mul((float3x3)UNITY_MATRIX_V,mul(IP,mul(IP,WorldEye(worldPos) - center))); //T^-2 * (eye - center)
	a3 = mul((float3x3)UNITY_MATRIX_V,mul(IP,mul(IP,view))); 						//T^-2 * A[0]
}

#endif
