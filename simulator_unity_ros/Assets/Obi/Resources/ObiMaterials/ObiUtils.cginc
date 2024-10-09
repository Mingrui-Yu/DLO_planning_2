#ifndef OBIUTILS_INCLUDED
#define OBIUTILS_INCLUDED

float4x4 _Camera_to_World;

// abstract texture declaration/sampling over built-in and SRPs:
#ifndef TEXTURE2D
#define TEXTURE2D(name) sampler2D name
#endif

#ifndef TEXTURE2D_HALF
#define TEXTURE2D_HALF(name) sampler2D_half name
#endif

#ifndef TEXTURE2D_FLOAT
#define TEXTURE2D_FLOAT(name) sampler2D_float name
#endif

#ifndef SAMPLE_TEXTURE2D
#define SAMPLE_TEXTURE2D(textureName, samplerName, coord2) tex2D(textureName,coord2)
#endif

#ifndef SAMPLER
#define SAMPLER(samplerName)
#endif


#endif
