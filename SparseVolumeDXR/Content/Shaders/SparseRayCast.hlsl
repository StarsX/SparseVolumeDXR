//--------------------------------------------------------------------------------------
// By Stars XU Tianchen
//--------------------------------------------------------------------------------------

#include "SharedConst.h"

//#define MAX_ANY_HIT_COUNT 4

typedef RaytracingAccelerationStructure RaytracingAS;
typedef BuiltInTriangleIntersectionAttributes TriAttributes;

//--------------------------------------------------------------------------------------
// Structs
//--------------------------------------------------------------------------------------
struct RayPayload
{
	uint2 Index;
	uint Sign;
	float RayLengthSum;
};

struct RayGenConstants
{
	matrix	ScreenToWorld;
	float3	LightDir;
};

static const float g_density = 1.0;
static const float g_absorption = 1.0;

//static const float3 g_cornflowerBlue = { 0.392156899, 0.584313750, 0.929411829 };
static const min16float3 g_clear = { 0.0, 0.2, 0.4 };

//--------------------------------------------------------------------------------------
// Constant buffers
//--------------------------------------------------------------------------------------
ConstantBuffer<RayGenConstants> l_rayGenCB : register(b0);

//--------------------------------------------------------------------------------------
// Texture and buffers
//--------------------------------------------------------------------------------------
RWTexture2D<float4>			RenderTarget	: register(u0);
RWTexture2D<float>			ThicknessBuf	: register(u1);
RaytracingAS				g_scene			: register(t0);
Texture2DArray<uint>		g_txKBufDepth	: register(t1);

//--------------------------------------------------------------------------------------
// Screen space to loacal space
//--------------------------------------------------------------------------------------
float3 ScreenToWorld(float2 xy, float depth)
{
	float4 pos = mul(float4(xy, depth, 1.0), l_rayGenCB.ScreenToWorld);

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
// Simpson rule for integral approximation
//--------------------------------------------------------------------------------------
float Simpson(float4 f, float a, float b)
{
	return (b - a) / 8.0 * (f.x + 3.0 * (f.y + f.z) + f.w);
}

//--------------------------------------------------------------------------------------
// Compute light-path thickness
//--------------------------------------------------------------------------------------
float LightPathThickness(RayDesc ray, float3 origin, uint2 index)
{
	// Trace the ray.
	ray.Origin = origin;

	RayPayload payloadF = { index, 0, 0.0 };
	TraceRay(g_scene, RAY_FLAG_CULL_BACK_FACING_TRIANGLES, ~0, 0, 1, 0, ray, payloadF);

	RayPayload payloadB = { index, 1, 0.0 };
	TraceRay(g_scene, RAY_FLAG_CULL_FRONT_FACING_TRIANGLES, ~0, 0, 1, 0, ray, payloadB);

	//return ThicknessBuf[index];
	return payloadB.RayLengthSum - payloadF.RayLengthSum;
	//RayPayload payload = { index, 0, 0.0 };
	//TraceRay(g_scene, RAY_FLAG_NONE, ~0, 0, 1, 0, ray, payload);

	//return payload.RayLengthSum;
}

//--------------------------------------------------------------------------------------
// Ray generation
//--------------------------------------------------------------------------------------
[shader("raygeneration")]
void raygenMain()
{
	// Trace the ray.
	RayDesc ray;
	ray.Direction = l_rayGenCB.LightDir;
	ray.TMin = 0.125;
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
		const float3 posFront = ScreenToWorld(xy, depthFront);
		const float3 posBack = ScreenToWorld(xy, depthBack);
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

	const float transmission = exp(-thickness * g_absorption * g_density);

	float3 result = scatter * 1.0 + 0.3;
	result = lerp(result, g_clear * g_clear, transmission);

	RenderTarget[index] = float4(sqrt(result), 1.0);
}

//--------------------------------------------------------------------------------------
// Ray closest hit
//--------------------------------------------------------------------------------------
[shader("closesthit")]
void closestHitMain(inout RayPayload payload, TriAttributes attr)
{
}

//--------------------------------------------------------------------------------------
// Ray any hit
//--------------------------------------------------------------------------------------
[shader("anyhit")]
void anyHitMain(inout RayPayload payload, TriAttributes attr)
{
	//if (payload.AnyHitCount < MAX_ANY_HIT_COUNT)
	{
		//payload.RayLengthSum += RayTCurrent();
		//++payload.AnyHitCount;
		//IgnoreHit();
	}

	payload.RayLengthSum += RayTCurrent();
	//payload.RayLengthSum = max(RayTCurrent(), payload.RayLengthSum);
	//AcceptHitAndEndSearch();

	//const float distance = RayTCurrent();
	//ThicknessBuf[payload.Index] += payload.Sign ? distance : -distance;
	//ThicknessBuf[payload.Index] = 10.0f;
}

//--------------------------------------------------------------------------------------
// Ray miss
//--------------------------------------------------------------------------------------
[shader("miss")]
void missMain(inout RayPayload payload)
{
}
