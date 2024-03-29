//--------------------------------------------------------------------------------------
// Copyright (c) XU, Tianchen. All rights reserved.
//--------------------------------------------------------------------------------------

#include "SparseRayCast.hlsli"

#define TRACE_RAY_ONCE	1

#define SCREEN_TO_WORLD(xy, z)	ScreenToWorld(xy, z, l_rayGenCB.ScreenToWorld)

typedef RaytracingAccelerationStructure RaytracingAS;
typedef BuiltInTriangleIntersectionAttributes TriAttributes;

//--------------------------------------------------------------------------------------
// Structs
//--------------------------------------------------------------------------------------
struct RayPayload
{
	float RayLengthSum;
};

struct RayGenConstants
{
	matrix	ScreenToWorld;
	float3	LightDir;
};

//--------------------------------------------------------------------------------------
// Constant buffers
//--------------------------------------------------------------------------------------
ConstantBuffer<RayGenConstants> l_rayGenCB : register(b0);

//--------------------------------------------------------------------------------------
// Texture and buffers
//--------------------------------------------------------------------------------------
RWTexture2D<float4>			RenderTarget	: register(u0);
RaytracingAS				g_scene			: register(t0);
Texture2DArray<uint>		g_txKBufDepth	: register(t1);

//--------------------------------------------------------------------------------------
// Compute light-path thickness
//--------------------------------------------------------------------------------------
float LightPathThickness(RayDesc ray, float3 origin, uint2 index)
{
	// Trace the ray.
	ray.Origin = origin;

#if TRACE_RAY_ONCE
	RayPayload payload = { 0.0 };
	TraceRay(g_scene, RAY_FLAG_NONE, ~0, 0, 1, 0, ray, payload);

	return payload.RayLengthSum;
#else
	RayPayload payloadF = { 0.0 };
	TraceRay(g_scene, RAY_FLAG_CULL_BACK_FACING_TRIANGLES, ~0, 0, 1, 0, ray, payloadF);

	RayPayload payloadB = { 0.0 };
	TraceRay(g_scene, RAY_FLAG_CULL_FRONT_FACING_TRIANGLES, ~0, 0, 1, 0, ray, payloadB);

	return payloadB.RayLengthSum - payloadF.RayLengthSum;
#endif
}

//--------------------------------------------------------------------------------------
// Ray generation
//--------------------------------------------------------------------------------------
[shader("raygeneration")]
void raygenMain()
{
	// Trace the ray.
	RayDesc ray;
	ray.Origin = 0.0;
	ray.Direction = l_rayGenCB.LightDir;
	ray.TMin = 0.0;
	ray.TMax = 10000.0;

	// Fallback layer has no depth
	const uint2 index = DispatchRaysIndex().xy;
	const float2 xy = index + 0.5;	// half pixel offset from the middle of the pixel.
	
	float thickness = 0.0;
	float scatter = 0.0;

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
		
		float4 thicknesses;	// Front, 1/3, 2/3, and back thicknesses
		thicknesses.x = LightPathThickness(ray, posFront, index) + thickness;
		thicknesses.y = LightPathThickness(ray, posFMid, index) + thicknessSeg / 3.0 + thickness;
		thicknesses.z = LightPathThickness(ray, posBMid, index) + thicknessSeg * (2.0 / 3.0) + thickness;

		// Update the total thickness
		thickness += thicknessSeg;
		thicknesses.w = LightPathThickness(ray, posBack, index) + thickness;
		
		// Compute transmission
		const float4 transmissions = float4(exp(-thicknesses * g_absorption * g_density));

		// Integral
		scatter += g_density * Simpson(transmissions, 0.0, thicknessSeg);
	}

	const min16float transmission = min16float(exp(-thickness * g_absorption * g_density));

	min16float3 result = min16float(scatter) * g_lightColor + g_ambient;
	result = lerp(result, g_clear * g_clear, transmission);

	RenderTarget[index] = float4(sqrt(result), 1.0);
}

//--------------------------------------------------------------------------------------
// Ray any hit
//--------------------------------------------------------------------------------------
[shader("anyhit")]
void anyHitMain(inout RayPayload payload, TriAttributes attr)
{
#if TRACE_RAY_ONCE
	const float distance = RayTCurrent();
	payload.RayLengthSum += HitKind() == HIT_KIND_TRIANGLE_BACK_FACE ? distance : -distance;
#else
	payload.RayLengthSum += RayTCurrent();
#endif
	IgnoreHit();
}

//--------------------------------------------------------------------------------------
// Ray miss
//--------------------------------------------------------------------------------------
[shader("miss")]
void missMain(inout RayPayload payload)
{
}
