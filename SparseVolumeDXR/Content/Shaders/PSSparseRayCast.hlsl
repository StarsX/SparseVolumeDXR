//--------------------------------------------------------------------------------------
// By XU, Tianchen
//--------------------------------------------------------------------------------------

#include "SharedConst.h"

//--------------------------------------------------------------------------------------
// Constant buffer
//--------------------------------------------------------------------------------------
cbuffer cbMatrices
{
	matrix	g_screenToWorld;	// View-screen space
	matrix	g_viewProjLS;		// Light space
};

static const min16float g_density = 1.0;
static const float g_absorption = 1.0;

//--------------------------------------------------------------------------------------
// Textures
//--------------------------------------------------------------------------------------
Texture2DArray<uint>		g_txKBufDepth;		// View-screen space
Texture2DArray<uint>		g_txKBufDepthLS;	// Light space

//--------------------------------------------------------------------------------------
// Screen space to loacal space
//--------------------------------------------------------------------------------------
float3 ScreenToWorld(float3 loc)
{
	float4 pos = mul(float4(loc, 1.0), g_screenToWorld);
	
	return pos.xyz / pos.w;
}

//--------------------------------------------------------------------------------------
// Perspective clip space to view space
//--------------------------------------------------------------------------------------
float PrespectiveToViewZ(float z)
{
	return g_zNear * g_zFar / (g_zFar - z * (g_zFar - g_zNear));
}

//--------------------------------------------------------------------------------------
// Orthographic clip space to view space
//--------------------------------------------------------------------------------------
float OrthoToViewZ(float z)
{
	return z * (g_zFarLS - g_zNearLS) + g_zNearLS;
}

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
// Simpson rule for integral approximation
//--------------------------------------------------------------------------------------
min16float Simpson(min16float4 vf, float a, float b)
{
	return min16float(b - a) / 8.0 * (vf.x + 3.0 * (vf.y + vf.z) + vf.w);
}

//--------------------------------------------------------------------------------------
// Rendering from sparse volume representation
//--------------------------------------------------------------------------------------
min16float4 main(float4 Pos : SV_POSITION) : SV_TARGET
{
	const uint2 loc = Pos.xy;
	const float2 pos = loc;

	float thickness = 0.0;
	min16float scatter = 0.0;
	for (uint i = 0; i < NUM_K_LAYERS >> 1; ++i)
	{
		// Get screen-space depths
		const float depthFront = asfloat(g_txKBufDepth[uint3(loc, i * 2)]);
		const float depthBack = asfloat(g_txKBufDepth[uint3(loc, i * 2 + 1)]);

		if (depthFront >= 1.0 || depthBack >= 1.0) break;

		// Transform to world space
		const float3 posFront = ScreenToWorld(float3(pos, depthFront));
		const float3 posBack = ScreenToWorld(float3(pos, depthBack));
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
		const min16float4 transmissions = min16float4(exp(-thicknessnes * g_absorption * g_density));
		
		// Integral
		scatter += g_density * Simpson(transmissions, 0.0, thicknessSeg);
	}

	const min16float transmission = min16float(exp(-thickness * g_absorption * g_density));
	const min16float3 result = scatter * 1.0 + 0.3;

	return min16float4(sqrt(result), 1.0 - transmission);
}
