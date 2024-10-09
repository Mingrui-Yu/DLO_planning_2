#ifndef OBIPARTICLES_INCLUDED
#define OBIPARTICLES_INCLUDED

float _RadiusScale;

float3 BillboardSphereNormals(float2 texcoords)
{
	float3 n;
	n.xy = texcoords*2.0-1.0;
	float r2 = dot(n.xy, n.xy);
	clip (1 - r2);   // clip pixels outside circle
	n.z = sqrt(1.0 - r2);
	return n;
}

float BillboardSphereThickness(float2 texcoords)
{
	float2 n = texcoords*2.0-1.0;
	float r2 = dot(n.xy, n.xy);
	clip (1 - r2);   // clip pixels outside circle
	return sqrt(1.0 - r2)*2.0f*exp(-r2*2.0f);
}

#endif
