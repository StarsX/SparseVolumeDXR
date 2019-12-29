//--------------------------------------------------------------------------------------
// Copyright (c) XU, Tianchen. All rights reserved.
//--------------------------------------------------------------------------------------

#include "SharedConst.h"
#include "Optional/XUSGObjLoader.h"
#include "SparseVolume.h"

using namespace std;
using namespace DirectX;
using namespace XUSG;
using namespace XUSG::RayTracing;

const wchar_t* SparseVolume::HitGroupName = L"hitGroup";
const wchar_t* SparseVolume::RaygenShaderName = L"raygenMain";
const wchar_t* SparseVolume::ClosestHitShaderName = L"closestHitMain";
const wchar_t* SparseVolume::AnyHitShaderName = L"anyHitMain";
const wchar_t* SparseVolume::MissShaderName = L"missMain";

SparseVolume::SparseVolume(const RayTracing::Device& device) :
	m_device(device),
	m_instances()
{
	m_rayTracingPipelineCache.SetDevice(device);
	m_graphicsPipelineCache.SetDevice(device.Common);
	m_computePipelineCache.SetDevice(device.Common);
	m_descriptorTableCache.SetDevice(device.Common);
	m_pipelineLayoutCache.SetDevice(device.Common);

	m_descriptorTableCache.SetName(L"RayTracerDescriptorTableCache");
}

SparseVolume::~SparseVolume()
{
}

bool SparseVolume::Init(const RayTracing::CommandList& commandList, uint32_t width, uint32_t height, Format rtFormat,
	Format dsFormat, vector<Resource>& uploaders, Geometry& geometry, const char* fileName, const XMFLOAT4& posScale)
{
	m_viewport.x = static_cast<float>(width);
	m_viewport.y = static_cast<float>(height);
	m_posScale = posScale;

	// Load inputs
	ObjLoader objLoader;
	if (!objLoader.Import(fileName, true, true)) return false;
	N_RETURN(createVB(commandList, objLoader.GetNumVertices(), objLoader.GetVertexStride(), objLoader.GetVertices(), uploaders), false);
	N_RETURN(createIB(commandList, objLoader.GetNumIndices(), objLoader.GetIndices(), uploaders), false);

	// Create pipelines
	N_RETURN(createInputLayout(), false);
	N_RETURN(createPipelineLayouts(), false);
	N_RETURN(createPipelines(rtFormat, dsFormat), false);

	// Extract boundary
	const auto center = objLoader.GetCenter();
	m_bound = XMFLOAT4(center.x, center.y, center.z, objLoader.GetRadius());

	// Create output grids and build acceleration structures
	for (auto& kBuffer : m_depthKBuffers)
		N_RETURN(kBuffer.Create(m_device.Common, width, height, Format::R32_UINT, NUM_K_LAYERS,
			ResourceFlag::ALLOW_UNORDERED_ACCESS), false);
	for (auto& kBuffer : m_lsDepthKBuffers)
		N_RETURN(kBuffer.Create(m_device.Common, SHADOW_MAP_SIZE, SHADOW_MAP_SIZE, Format::R32_UINT, NUM_K_LAYERS,
			ResourceFlag::ALLOW_UNORDERED_ACCESS), false);
	for (auto& outView : m_outputViews)
		N_RETURN(outView.Create(m_device.Common, width, height, rtFormat, 1,
			ResourceFlag::ALLOW_UNORDERED_ACCESS), false);
	for (auto& thickness : m_thicknesses)
		N_RETURN(thickness.Create(m_device.Common, width, height, Format::R32_FLOAT, 1,
			ResourceFlag::ALLOW_UNORDERED_ACCESS), false);

	// Initialize world transform
	const auto world = XMMatrixIdentity();
	XMStoreFloat4x4(&m_world, XMMatrixTranspose(world));

	N_RETURN(buildAccelerationStructures(commandList, &geometry), false);
	N_RETURN(buildShaderTables(), false);

	return true;
}

void SparseVolume::UpdateFrame(uint32_t frameIndex, CXMMATRIX viewProj)
{
	// General matrices
	//const auto world = XMMatrixScaling(m_bound.w, m_bound.w, m_bound.w) *
		//XMMatrixTranslation(m_bound.x, m_bound.y, m_bound.z);
	const auto world = XMMatrixScaling(m_posScale.w, m_posScale.w, m_posScale.w) *
		XMMatrixTranslation(m_posScale.x, m_posScale.y, m_posScale.z);
	const auto worldViewProj = world * viewProj;
	XMStoreFloat4x4(&m_world, XMMatrixTranspose(world));
	XMStoreFloat4x4(&m_worldViewProj, XMMatrixTranspose(worldViewProj));

	// Light-space matrices
	const auto focusPt = XMLoadFloat4(&m_bound);
	const auto lightPt = XMVectorSet(-10.0f, 45.0f, -75.0f, 0.0f) + focusPt;
	const auto viewLS = XMMatrixLookAtLH(lightPt, focusPt, XMVectorSet(0.0f, 1.0f, 0.0f, 0.0f));
	const auto projLS = XMMatrixOrthographicLH(m_bound.w * 3.0f, m_bound.w * 3.0f, g_zNearLS, g_zFarLS);
	const auto viewProjLS = viewLS * projLS;
	const auto worldViewProjLS = world * viewProjLS;
	XMStoreFloat4x4(&m_cbPerObject.ViewProjLS, XMMatrixTranspose(viewProjLS));
	XMStoreFloat4x4(&m_worldViewProjLS, XMMatrixTranspose(worldViewProjLS));

	// Screen space matrices
	const auto toScreen = XMMATRIX
	(
		0.5f * m_viewport.x, 0.0f, 0.0f, 0.0f,
		0.0f, -0.5f * m_viewport.y, 0.0f, 0.0f,
		0.0f, 0.0f, 1.0f, 0.0f,
		0.5f * m_viewport.x, 0.5f * m_viewport.y, 0.0f, 1.0f
	);
	const auto worldToScreen = viewProj * toScreen;
	const auto screenToWorld = XMMatrixInverse(nullptr, worldToScreen);
	XMStoreFloat4x4(&m_cbPerObject.ScreenToWorld, XMMatrixTranspose(screenToWorld));

	// Ray tracing
	RayGenConstants cbRayGen;
	cbRayGen.ScreenToWorld = m_cbPerObject.ScreenToWorld;
	XMStoreFloat4(&cbRayGen.LightDir, XMVector3Normalize(lightPt - focusPt));

	m_rayGenShaderTables[frameIndex].Reset();
	m_rayGenShaderTables[frameIndex].AddShaderRecord(ShaderRecord(m_device, m_rayTracingPipeline,
		RaygenShaderName, &cbRayGen, sizeof(cbRayGen)));
}

void SparseVolume::Render(const RayTracing::CommandList& commandList, uint32_t frameIndex,
	const Descriptor& rtv, const Descriptor& dsv, const Descriptor& lsDsv)
{
	const DescriptorPool descriptorPools[] = { m_descriptorTableCache.GetDescriptorPool(CBV_SRV_UAV_POOL) };
	commandList.SetDescriptorPools(static_cast<uint32_t>(size(descriptorPools)), descriptorPools);

	depthPeelLightSpace(commandList, frameIndex, lsDsv);
	depthPeel(commandList, frameIndex, dsv, false);

	render(commandList, frameIndex, rtv);
}

void SparseVolume::RenderDXR(const RayTracing::CommandList& commandList,
	uint32_t frameIndex, RenderTarget& dst, const Descriptor& dsv)
{
	const DescriptorPool descriptorPools[] = { m_descriptorTableCache.GetDescriptorPool(CBV_SRV_UAV_POOL) };
	commandList.SetDescriptorPools(static_cast<uint32_t>(size(descriptorPools)), descriptorPools);

	depthPeel(commandList, frameIndex, dsv);
	rayTrace(commandList, frameIndex);

	ResourceBarrier barriers[2];
	auto numBarriers = m_outputViews[frameIndex].SetBarrier(barriers, ResourceState::COPY_SOURCE);
	numBarriers = dst.SetBarrier(barriers, ResourceState::COPY_DEST, numBarriers);

	TextureCopyLocation dstCopyLoc(dst.GetResource().get(), 0);
	TextureCopyLocation srcCopyLoc(m_outputViews[frameIndex].GetResource().get(), 0);
	commandList.Barrier(numBarriers, barriers);
	commandList.CopyTextureRegion(dstCopyLoc, 0, 0, 0, srcCopyLoc);
}

bool SparseVolume::createVB(const RayTracing::CommandList& commandList, uint32_t numVert,
	uint32_t stride, const uint8_t* pData, vector<Resource>& uploaders)
{
	N_RETURN(m_vertexBuffer.Create(m_device.Common, numVert, stride,
		ResourceFlag::NONE, MemoryType::DEFAULT), false);
	uploaders.push_back(nullptr);

	return m_vertexBuffer.Upload(commandList, uploaders.back(),
		ResourceState::NON_PIXEL_SHADER_RESOURCE,
		pData, stride * numVert);
}

bool SparseVolume::createIB(const RayTracing::CommandList& commandList, uint32_t numIndices,
	const uint32_t* pData, vector<Resource>& uploaders)
{
	m_numIndices = numIndices;
	const uint32_t byteWidth = sizeof(uint32_t) * numIndices;

	N_RETURN(m_indexBuffer.Create(m_device.Common, byteWidth, Format::R32_UINT,
		ResourceFlag::NONE, MemoryType::DEFAULT), false);
	uploaders.push_back(nullptr);

	return m_indexBuffer.Upload(commandList, uploaders.back(),
		ResourceState::NON_PIXEL_SHADER_RESOURCE, pData, byteWidth);
}

bool SparseVolume::createInputLayout()
{
	// Define the vertex input layout.
	InputElementTable inputElementDescs =
	{
		{ "POSITION",	0, Format::R32G32B32_FLOAT, 0, 0,						InputClassification::PER_VERTEX_DATA, 0 },
		{ "NORMAL",		0, Format::R32G32B32_FLOAT, 0, APPEND_ALIGNED_ELEMENT,	InputClassification::PER_VERTEX_DATA, 0 }
	};

	X_RETURN(m_inputLayout, m_graphicsPipelineCache.CreateInputLayout(inputElementDescs), false);

	return true;
}

bool SparseVolume::createPipelineLayouts()
{
	// Depth peeling pass
	{
		// Get pipeline layout
		Util::PipelineLayout pipelineLayout;
		pipelineLayout.SetConstants(CONSTANTS, SizeOfInUint32(XMFLOAT4X4), 0);
		pipelineLayout.SetRange(SRV_UAVS, DescriptorType::UAV, 1, 0, 0,
			DescriptorRangeFlag::DATA_STATIC_WHILE_SET_AT_EXECUTE);
		pipelineLayout.SetShaderStage(CONSTANTS, Shader::Stage::VS);
		pipelineLayout.SetShaderStage(SRV_UAVS, Shader::Stage::PS);
		X_RETURN(m_pipelineLayouts[DEPTH_PEEL_LAYOUT], pipelineLayout.GetPipelineLayout(m_pipelineLayoutCache,
			PipelineLayoutFlag::ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT, L"DepthPeelingLayout"), false);
	}

	// Sparse volume rendering pass with shadow mapping
	{
		// Get pipeline layout
		Util::PipelineLayout pipelineLayout;
		pipelineLayout.SetConstants(CONSTANTS, SizeOfInUint32(PerObjConstants), 0);
		pipelineLayout.SetRange(SRV_UAVS, DescriptorType::SRV, 2, 0);
		pipelineLayout.SetShaderStage(CONSTANTS, Shader::Stage::PS);
		pipelineLayout.SetShaderStage(SRV_UAVS, Shader::Stage::PS);
		X_RETURN(m_pipelineLayouts[SPARSE_RAYCAST_LAYOUT], pipelineLayout.GetPipelineLayout(m_pipelineLayoutCache,
			PipelineLayoutFlag::NONE, L"SparseRayCastLayout"), false);
	}

	// Global pipeline layout
	// This is a pipeline layout that is shared across all raytracing shaders invoked during a DispatchRays() call.
	{
		RayTracing::PipelineLayout pipelineLayout;
		pipelineLayout.SetRange(OUTPUT_VIEW, DescriptorType::UAV, 2, 0);
		pipelineLayout.SetRootSRV(ACCELERATION_STRUCTURE, 0);
		pipelineLayout.SetRange(DEPTH_K_BUFFERS, DescriptorType::SRV, 1, 1);
		X_RETURN(m_pipelineLayouts[GLOBAL_LAYOUT], pipelineLayout.GetPipelineLayout(m_device, m_pipelineLayoutCache,
			PipelineLayoutFlag::NONE, NumUAVs, L"RayTracerGlobalPipelineLayout"), false);
	}

	// Local pipeline layout for RayGen shader
	// This is a pipeline layout that enables a shader to have unique arguments that come from shader tables.
	{
		RayTracing::PipelineLayout pipelineLayout;
		pipelineLayout.SetConstants(CONSTANTS, SizeOfInUint32(RayGenConstants), 0);
		X_RETURN(m_pipelineLayouts[RAY_GEN_LAYOUT], pipelineLayout.GetPipelineLayout(m_device, m_pipelineLayoutCache,
			PipelineLayoutFlag::LOCAL_PIPELINE_LAYOUT, NumUAVs, L"RayTracerRayGenPipelineLayout"), false);
	}

	return true;
}

bool SparseVolume::createPipelines(Format rtFormat, Format dsFormat)
{
	{
		N_RETURN(m_shaderPool.CreateShader(Shader::Stage::VS, VS_BASE_PASS, L"VSBasePass.cso"), false);
		N_RETURN(m_shaderPool.CreateShader(Shader::Stage::PS, PS_DEPTH_PEEL, L"PSDepthPeel.cso"), false);

		Graphics::State state;
		state.SetPipelineLayout(m_pipelineLayouts[DEPTH_PEEL_LAYOUT]);
		state.SetShader(Shader::Stage::VS, m_shaderPool.GetShader(Shader::Stage::VS, VS_BASE_PASS));
		state.SetShader(Shader::Stage::PS, m_shaderPool.GetShader(Shader::Stage::PS, PS_DEPTH_PEEL));
		state.RSSetState(Graphics::RasterizerPreset::CULL_NONE, m_graphicsPipelineCache);
		state.DSSetState(Graphics::DepthStencilPreset::DEPTH_READ_LESS, m_graphicsPipelineCache);
		state.IASetInputLayout(m_inputLayout);
		state.IASetPrimitiveTopologyType(PrimitiveTopologyType::TRIANGLE);
		state.OMSetDSVFormat(dsFormat);

		X_RETURN(m_pipelines[DEPTH_PEEL], state.GetPipeline(m_graphicsPipelineCache, L"DepthPeeling"), false);
	}

	{
		N_RETURN(m_shaderPool.CreateShader(Shader::Stage::VS, VS_SCREEN_QUAD, L"VSScreenQuad.cso"), false);
		N_RETURN(m_shaderPool.CreateShader(Shader::Stage::PS, PS_SPARSE_RAYCAST, L"PSSparseRayCast.cso"), false);

		Graphics::State state;
		state.SetPipelineLayout(m_pipelineLayouts[SPARSE_RAYCAST_LAYOUT]);
		state.SetShader(Shader::Stage::VS, m_shaderPool.GetShader(Shader::Stage::VS, VS_SCREEN_QUAD));
		state.SetShader(Shader::Stage::PS, m_shaderPool.GetShader(Shader::Stage::PS, PS_SPARSE_RAYCAST));
		state.DSSetState(Graphics::DepthStencilPreset::DEPTH_STENCIL_NONE, m_graphicsPipelineCache);
		state.IASetPrimitiveTopologyType(PrimitiveTopologyType::TRIANGLE);
		state.OMSetRTVFormats(&rtFormat, 1);

		X_RETURN(m_pipelines[SPARSE_RAYCAST], state.GetPipeline(m_graphicsPipelineCache, L"SparseRayCast"), false);
	}

	{
		Blob shaderLib;
		V_RETURN(D3DReadFileToBlob(L"SparseRayCast.cso", &shaderLib), cerr, false);

		RayTracing::State state;
		state.SetShaderLibrary(shaderLib);
		state.SetHitGroup(0, HitGroupName, ClosestHitShaderName, AnyHitShaderName);
		state.SetShaderConfig(sizeof(XMFLOAT4), sizeof(XMFLOAT2));
		state.SetLocalPipelineLayout(0, m_pipelineLayouts[RAY_GEN_LAYOUT],
			1, reinterpret_cast<const void**>(&RaygenShaderName));
		state.SetGlobalPipelineLayout(m_pipelineLayouts[GLOBAL_LAYOUT]);
		state.SetMaxRecursionDepth(1);
		m_rayTracingPipeline = state.GetPipeline(m_rayTracingPipelineCache, L"SparseRayCastDXR");

		N_RETURN(m_rayTracingPipeline.Native || m_rayTracingPipeline.Fallback, false);
	}

	return true;
}

bool SparseVolume::createDescriptorTables()
{
	// Acceleration structure UAVs
	{
		const Descriptor descriptors[] = { m_bottomLevelAS.GetResult().GetUAV(), m_topLevelAS.GetResult().GetUAV() };
		Util::DescriptorTable descriptorTable;
		descriptorTable.SetDescriptors(0, static_cast<uint32_t>(size(descriptors)), descriptors);
		const auto asTable = descriptorTable.GetCbvSrvUavTable(m_descriptorTableCache);
		N_RETURN(asTable, false);
	}

	// Other UAVs
	for (auto i = 0u; i < FrameCount; ++i)
	{
		{
			// Get UAV
			Util::DescriptorTable uavTable;
			uavTable.SetDescriptors(0, 1, &m_depthKBuffers[i].GetUAV());
			X_RETURN(m_uavTables[UAV_TABLE_KBUFFER][i], uavTable.GetCbvSrvUavTable(m_descriptorTableCache), false);
		}

		{
			// Get UAV
			Util::DescriptorTable uavTable;
			uavTable.SetDescriptors(0, 1, &m_lsDepthKBuffers[i].GetUAV());
			X_RETURN(m_uavTables[UAV_TABLE_LS_KBUFFER][i], uavTable.GetCbvSrvUavTable(m_descriptorTableCache), false);
		}

		{
			// Output UAV
			Util::DescriptorTable uavTable;
			uavTable.SetDescriptors(0, 1, &m_outputViews[i].GetUAV());
			X_RETURN(m_uavTables[UAV_TABLE_OUT_VIEW][i], uavTable.GetCbvSrvUavTable(m_descriptorTableCache), false);
		}

		{
			// Output UAV
			Util::DescriptorTable uavTable;
			uavTable.SetDescriptors(0, 1, &m_thicknesses[i].GetUAV());
			X_RETURN(m_uavTables[UAV_TABLE_THICKNESS][i], uavTable.GetCbvSrvUavTable(m_descriptorTableCache), false);
		}
	}

	// SRVs
	for (auto i = 0ui8; i < FrameCount; ++i)
	{
		// Depth K-buffer SRV
		const Descriptor srvs[] = { m_depthKBuffers[i].GetSRV(), m_lsDepthKBuffers[i].GetSRV() };
		Util::DescriptorTable srvTable;
		srvTable.SetDescriptors(0, static_cast<uint32_t>(size(srvs)), srvs);
		X_RETURN(m_srvTables[i], srvTable.GetCbvSrvUavTable(m_descriptorTableCache), false);
	}

	// Create the sampler table
	/*{
		Util::DescriptorTable samplerTable;
		const auto sampler = LINEAR_CLAMP;
		samplerTable.SetSamplers(0, 1, &sampler, m_descriptorTableCache);
		X_RETURN(m_samplerTable, samplerTable.GetSamplerTable(m_descriptorTableCache), false);
	}*/

	return true;
}

bool SparseVolume::buildAccelerationStructures(const RayTracing::CommandList& commandList, Geometry* geometries)
{
	AccelerationStructure::SetFrameCount(FrameCount);

	// Set geometries
	const auto geometryFlags = D3D12_RAYTRACING_GEOMETRY_FLAG_NONE;
	BottomLevelAS::SetGeometries(geometries, 1, Format::R32G32B32_FLOAT,
		&m_vertexBuffer.GetVBV(), &m_indexBuffer.GetIBV(), &geometryFlags);

	// Descriptor index in descriptor pool
	const uint32_t bottomLevelASIndex = 0;
	const uint32_t topLevelASIndex = bottomLevelASIndex + 1;

	// Prebuild
	N_RETURN(m_bottomLevelAS.PreBuild(m_device, 1, geometries,
		bottomLevelASIndex, NumUAVs), false);
	N_RETURN(m_topLevelAS.PreBuild(m_device, 1, topLevelASIndex, NumUAVs), false);

	// Create scratch buffer
	auto scratchSize = m_topLevelAS.GetScratchDataMaxSize();
	scratchSize = (max)(m_bottomLevelAS.GetScratchDataMaxSize(), scratchSize);
	N_RETURN(AccelerationStructure::AllocateUAVBuffer(m_device, m_scratch, scratchSize), false);

	// Get descriptor pool and create descriptor tables
	N_RETURN(createDescriptorTables(), false);
	const auto& descriptorPool = m_descriptorTableCache.GetDescriptorPool(CBV_SRV_UAV_POOL);

	// Set instance
	float* const pTransform[] = { reinterpret_cast<float*>(&m_world) };
	TopLevelAS::SetInstances(m_device, m_instances, 1, &m_bottomLevelAS, pTransform);

	// Build bottom level ASs
	m_bottomLevelAS.Build(commandList, m_scratch, descriptorPool, NumUAVs);

	// Build top level AS
	m_topLevelAS.Build(commandList, m_scratch, m_instances, descriptorPool, NumUAVs);

	// Set resource barriers
	ResourceBarrier barriers[2];
	auto numBarriers = m_vertexBuffer.SetBarrier(barriers, ResourceState::VERTEX_AND_CONSTANT_BUFFER);
	numBarriers = m_indexBuffer.SetBarrier(barriers, ResourceState::INDEX_BUFFER, numBarriers);
	commandList.Barrier(numBarriers, barriers);

	return true;
}

bool SparseVolume::buildShaderTables()
{
	// Get shader identifiers.
	const auto shaderIDSize = ShaderRecord::GetShaderIDSize(m_device);

	for (auto i = 0ui8; i < FrameCount; ++i)
	{
		// Ray gen shader table
		N_RETURN(m_rayGenShaderTables[i].Create(m_device, 1, shaderIDSize + sizeof(RayGenConstants),
			(L"RayGenShaderTable" + to_wstring(i)).c_str()), false);
		N_RETURN(m_rayGenShaderTables[i].AddShaderRecord(ShaderRecord(m_device, m_rayTracingPipeline,
			RaygenShaderName, &RayGenConstants(), sizeof(RayGenConstants))), false);
	}

	// Hit group shader table
	N_RETURN(m_hitGroupShaderTable.Create(m_device, 1, shaderIDSize, L"HitGroupShaderTable"), false);
	N_RETURN(m_hitGroupShaderTable.AddShaderRecord(ShaderRecord(m_device, m_rayTracingPipeline, HitGroupName)), false);

	// Miss shader table
	N_RETURN(m_missShaderTable.Create(m_device, 1, shaderIDSize, L"MissShaderTable"), false);
	N_RETURN(m_missShaderTable.AddShaderRecord(ShaderRecord(m_device, m_rayTracingPipeline, MissShaderName)), false);

	return true;
}

void SparseVolume::depthPeel(const RayTracing::CommandList& commandList,
	uint32_t frameIndex, const Descriptor& dsv, bool setPipeline)
{
	// Set resource barrier
	ResourceBarrier barrier;
	const auto numBarriers = m_depthKBuffers[frameIndex].SetBarrier(&barrier, ResourceState::UNORDERED_ACCESS);
	commandList.Barrier(numBarriers, &barrier);

	// Set descriptor tables
	commandList.SetGraphicsPipelineLayout(m_pipelineLayouts[DEPTH_PEEL_LAYOUT]);
	commandList.SetGraphics32BitConstants(CONSTANTS, SizeOfInUint32(XMFLOAT4X4), &m_worldViewProj);
	commandList.SetGraphicsDescriptorTable(SRV_UAVS, m_uavTables[UAV_TABLE_KBUFFER][frameIndex]);

	// Set pipeline state
	if (setPipeline) commandList.SetPipelineState(m_pipelines[DEPTH_PEEL]);

	// Set viewport
	Viewport viewport(0.0f, 0.0f, m_viewport.x, m_viewport.y);
	RectRange scissorRect(0, 0, static_cast<long>(m_viewport.x), static_cast<long>(m_viewport.y));
	commandList.RSSetViewports(1, &viewport);
	commandList.RSSetScissorRects(1, &scissorRect);

	const auto maxDepth = 1.0f;
	commandList.OMSetRenderTargets(0, nullptr, &dsv);
	commandList.ClearUnorderedAccessViewUint(m_uavTables[UAV_TABLE_KBUFFER][frameIndex], m_depthKBuffers[frameIndex].GetUAV(),
		m_depthKBuffers[frameIndex].GetResource(), XMVECTORU32{ reinterpret_cast<const uint32_t&>(maxDepth) }.u);

	// Record commands.
	commandList.IASetVertexBuffers(0, 1, &m_vertexBuffer.GetVBV());
	commandList.IASetIndexBuffer(m_indexBuffer.GetIBV());
	commandList.IASetPrimitiveTopology(PrimitiveTopology::TRIANGLELIST);
	commandList.DrawIndexed(m_numIndices, 1, 0, 0, 0);
}

void SparseVolume::depthPeelLightSpace(const RayTracing::CommandList& commandList,
	uint32_t frameIndex, const Descriptor& dsv)
{
	// Set resource barrier
	ResourceBarrier barrier;
	const auto numBarriers = m_lsDepthKBuffers[frameIndex].SetBarrier(&barrier, ResourceState::UNORDERED_ACCESS);
	commandList.Barrier(numBarriers, &barrier);

	// Set descriptor tables
	commandList.SetGraphicsPipelineLayout(m_pipelineLayouts[DEPTH_PEEL_LAYOUT]);

	commandList.SetGraphics32BitConstants(CONSTANTS, SizeOfInUint32(XMFLOAT4X4), &m_worldViewProjLS);
	commandList.SetGraphicsDescriptorTable(SRV_UAVS, m_uavTables[UAV_TABLE_LS_KBUFFER][frameIndex]);

	// Set pipeline state
	commandList.SetPipelineState(m_pipelines[DEPTH_PEEL]);

	// Set viewport
	Viewport viewport(0.0f, 0.0f, static_cast<float>(SHADOW_MAP_SIZE), static_cast<float>(SHADOW_MAP_SIZE));
	RectRange scissorRect(0, 0, SHADOW_MAP_SIZE, SHADOW_MAP_SIZE);
	commandList.RSSetViewports(1, &viewport);
	commandList.RSSetScissorRects(1, &scissorRect);

	const auto maxDepth = 1.0f;
	commandList.OMSetRenderTargets(0, nullptr, &dsv);
	commandList.ClearUnorderedAccessViewUint(m_uavTables[UAV_TABLE_LS_KBUFFER][frameIndex], m_lsDepthKBuffers[frameIndex].GetUAV(),
		m_lsDepthKBuffers[frameIndex].GetResource(), XMVECTORU32{ reinterpret_cast<const uint32_t&>(maxDepth) }.u);

	// Record commands.
	commandList.IASetVertexBuffers(0, 1, &m_vertexBuffer.GetVBV());
	commandList.IASetIndexBuffer(m_indexBuffer.GetIBV());
	commandList.IASetPrimitiveTopology(PrimitiveTopology::TRIANGLELIST);
	commandList.DrawIndexed(m_numIndices, 1, 0, 0, 0);
}

void SparseVolume::render(const RayTracing::CommandList& commandList,
	uint32_t frameIndex, const Descriptor& rtv)
{
	// Set resource barriers
	ResourceBarrier barriers[2];
	auto numBarriers = m_depthKBuffers[frameIndex].SetBarrier(barriers, ResourceState::PIXEL_SHADER_RESOURCE);
	numBarriers = m_lsDepthKBuffers[frameIndex].SetBarrier(barriers, ResourceState::PIXEL_SHADER_RESOURCE, numBarriers);
	commandList.Barrier(numBarriers, barriers);

	// Set descriptor tables
	commandList.SetGraphicsPipelineLayout(m_pipelineLayouts[SPARSE_RAYCAST_LAYOUT]);

	commandList.SetGraphics32BitConstants(CONSTANTS, SizeOfInUint32(PerObjConstants), &m_cbPerObject);
	commandList.SetGraphicsDescriptorTable(SRV_UAVS, m_srvTables[frameIndex]);

	// Set pipeline state
	commandList.SetPipelineState(m_pipelines[SPARSE_RAYCAST]);

	// Set viewport
	Viewport viewport(0.0f, 0.0f, m_viewport.x, m_viewport.y);
	RectRange scissorRect(0, 0, static_cast<long>(m_viewport.x), static_cast<long>(m_viewport.y));
	commandList.RSSetViewports(1, &viewport);
	commandList.RSSetScissorRects(1, &scissorRect);

	commandList.OMSetRenderTargets(1, &rtv);

	// Record commands.
	commandList.IASetPrimitiveTopology(PrimitiveTopology::TRIANGLESTRIP);
	commandList.Draw(3, 1, 0, 0);
}

void SparseVolume::rayTrace(const RayTracing::CommandList& commandList, uint32_t frameIndex)
{
	// Set resource barrier
	ResourceBarrier barrier;
	const auto numBarriers = m_outputViews[frameIndex].SetBarrier(&barrier, ResourceState::UNORDERED_ACCESS);
	commandList.Barrier(numBarriers, &barrier);

	// Set descriptor tables
	commandList.SetComputePipelineLayout(m_pipelineLayouts[GLOBAL_LAYOUT]);
	commandList.SetComputeDescriptorTable(OUTPUT_VIEW, m_uavTables[UAV_TABLE_OUT_VIEW][frameIndex]);
	commandList.SetTopLevelAccelerationStructure(ACCELERATION_STRUCTURE, m_topLevelAS);
	commandList.SetComputeDescriptorTable(DEPTH_K_BUFFERS, m_srvTables[frameIndex]);

	commandList.ClearUnorderedAccessViewFloat(m_uavTables[UAV_TABLE_THICKNESS][frameIndex], m_thicknesses[frameIndex].GetUAV(),
		m_thicknesses[frameIndex].GetResource(), XMVECTORF32{ 0.0f });

	// Fallback layer has no depth
	commandList.DispatchRays(m_rayTracingPipeline, (uint32_t)m_viewport.x, (uint32_t)m_viewport.y, 1,
		m_hitGroupShaderTable, m_missShaderTable, m_rayGenShaderTables[frameIndex]);
}
