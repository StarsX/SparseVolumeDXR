//--------------------------------------------------------------------------------------
// By XU, Tianchen
//--------------------------------------------------------------------------------------

#include "SparseRayCast.hlsli"

#define SCREEN_TO_WORLD(xy, z)	ScreenToWorld(xy, z, g_screenToWorld)

//--------------------------------------------------------------------------------------
// Constant buffer
//--------------------------------------------------------------------------------------
cbuffer cbMatrices
{
	matrix	g_screenToWorld;	// View-screen space
	matrix	g_viewProjLS;		// Light space
};

//--------------------------------------------------------------------------------------
// Textures
//--------------------------------------------------------------------------------------
Texture2DArray<uint>	g_txKBufDepth;		// View-screen space
Texture2DArray<uint>	g_txKBufDepthLS;	// Light space

//--------------------------------------------------------------------------------------
// Compute light-path thickness
//--------------------------------------------------------------------------------------
float LightPathThickness(float3 pos)
{
	pos = mul(float4(pos, 1.0), g_viewProjLS).xyz;
	pos.xy = pos.xy * float2(0.5, -0.5) + 0.5;

	const uint2 loc = pos.xy * SHADOW_MAP_SIZE;
	
	float thickness = 0.0;
	for (uint i = 0; i < NUM_K_LAYERS >> 1; ++i)
	{
		// Get light-space depths
		const float depthFront = asfloat(g_txKBufDepthLS[uint3(loc, i * 2)]);
		float depthBack = asfloat(g_txKBufDepthLS[uint3(loc, i * 2 + 1)]);

		// Clip to the current point
		if (depthFront > pos.z || depthBack >= 1.0) break;
		depthBack = min(depthBack, pos.z);

		// Transform to view space
		const float zFront = OrthoToViewZ(depthFront);
		const float zBack = OrthoToViewZ(depthBack);

		thickness += zBack - zFront;
	}

	return thickness;
}

//--------------------------------------------------------------------------------------
// Rendering from sparse volume representation
//--------------------------------------------------------------------------------------
min16float4 main(float4 Pos : SV_POSITION) : SV_TARGET
{
	const float2 xy = Pos.xy;
	const uint2 index = xy;

	float thickness = 0.0;
	min16float scatter = 0.0;
	for (uint i = 0; i < NUM_K_LAYERS >> 1; ++i)
	{
		// Get screen-space depths
		const float depthFront = asfloat(g_txKBufDepth[uint3(index, i * 2)]);
		const float depthBack = asfloat(g_txKBufDepth[uint3(index, i * 2 + 1)]);

		if (depthFront >= 1.0 || depthBack >= 1.0) break;

		// Transform to world space
		const float3 posFront = SCREEN_TO_WORLD(xy, depthFront);
		const float3 posBack = SCREEN_TO_WORLD(xy, depthBack);
		const float3 posFMid = lerp(posFront, posBack, 1.0 / 3.0);
		const float3 posBMid = lerp(posFront, posBack, 2.0 / 3.0);

		// Transform to view space
		const float zFront = PrespectiveToViewZ(depthFront);
		const float zBack = PrespectiveToViewZ(depthBack);

		// Tickness of the current interval (segment)
		const float thicknessSeg = zBack - zFront;
		//const float thicknessSeg = distance(posFront, posBack);

		float4 thicknessnes;	// Front, 1/3, 2/3, and back thicknesses
		thicknessnes.x = LightPathThickness(posFront) + thickness;
		thicknessnes.y = LightPathThickness(posFMid) + thicknessSeg / 3.0 + thickness;
		thicknessnes.z = LightPathThickness(posBMid) + thicknessSeg * (2.0 / 3.0) + thickness;

		// Update the total thickness
		thickness += thicknessSeg;
		thicknessnes.w = LightPathThickness(posBack) + thickness;

		// Compute transmission
		const float4 transmissions = exp(-thicknessnes * g_absorption * g_density);
		
		// Integral
		scatter += g_density * Simpson(transmissions, 0.0, thicknessSeg);
	}

	const min16float transmission = min16float(exp(-thickness * g_absorption * g_density));

	min16float3 result = scatter * g_lightColor + g_ambient;
	result = lerp(result, g_clear * g_clear, transmission);

	return min16float4(sqrt(result), 1.0);
}
