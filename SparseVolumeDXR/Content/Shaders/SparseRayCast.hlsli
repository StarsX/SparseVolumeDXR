//--------------------------------------------------------------------------------------
// By Stars XU Tianchen
//--------------------------------------------------------------------------------------

#include "SharedConst.h"

static const min16float3 g_lightColor = 0.8;
static const min16float3 g_ambient = 0.2;

static const min16float g_density = 1.0;
static const float g_absorption = 1.0;

static const min16float3 g_clear = min16float3(CLEAR_COLOR);

//--------------------------------------------------------------------------------------
// Screen space to loacal space
//--------------------------------------------------------------------------------------
float3 ScreenToWorld(float2 xy, float z, matrix screenToWorld)
{
	float4 pos = mul(float4(xy, z, 1.0), screenToWorld);

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
// Simpson rule for integral approximation
//--------------------------------------------------------------------------------------
min16float Simpson(float4 f, float a, float b)
{
	return min16float((b - a) / 8.0 * (f.x + 3.0 * (f.y + f.z) + f.w));
}
