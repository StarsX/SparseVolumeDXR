//--------------------------------------------------------------------------------------
// Copyright (c) XU, Tianchen. All rights reserved.
//--------------------------------------------------------------------------------------

#include "SharedConst.h"

//--------------------------------------------------------------------------------------
// Unordered access textures
//--------------------------------------------------------------------------------------
RWTexture2DArray<uint> g_rwKBufDepth;

//--------------------------------------------------------------------------------------
// Depth peeling
//--------------------------------------------------------------------------------------
[earlydepthstencil]
void main(float4 Pos : SV_POSITION)
{
	uint2 loc = Pos.xy;
	uint depth = asuint(Pos.z);
	uint depthPrev;

	for (uint i = 0; i < NUM_K_LAYERS; ++i)
	{
		const uint3 idx = { loc, i };
		InterlockedMin(g_rwKBufDepth[idx], depth, depthPrev);
		depth = max(depth, depthPrev);
	}
}
