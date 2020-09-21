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
const wchar_t* SparseVolume::AnyHitShaderName = L"anyHitMain";
const wchar_t* SparseVolume::MissShaderName = L"missMain";

SparseVolume::SparseVolume(const RayTracing::Device& device) :
	m_device(device),
	m_instances()
{
	m_shaderPool = ShaderPool::MakeUnique();
	m_rayTracingPipelineCache = RayTracing::PipelineCache::MakeUnique(device);
	m_graphicsPipelineCache = Graphics::PipelineCache::MakeUnique(device.Common);
	m_computePipelineCache = Compute::PipelineCache::MakeUnique(device.Common);
	m_pipelineLayoutCache = PipelineLayoutCache::MakeUnique(device.Common);
	m_descriptorTableCache = DescriptorTableCache::MakeUnique(device.Common, L"RayTracerDescriptorTableCache");
}

SparseVolume::~SparseVolume()
{
}

bool SparseVolume::Init(RayTracing::CommandList* pCommandList, uint32_t width, uint32_t height, Format rtFormat,
	Format dsFormat, vector<Resource>& uploaders, Geometry* pGeometries, const char* fileName, const XMFLOAT4& posScale)
{
	m_viewport.x = static_cast<float>(width);
	m_viewport.y = static_cast<float>(height);
	m_posScale = posScale;

	m_useRayTracing = pGeometries;

	// Load inputs
	ObjLoader objLoader;
	if (!objLoader.Import(fileName, true, true)) return false;
	N_RETURN(createVB(pCommandList, objLoader.GetNumVertices(), objLoader.GetVertexStride(), objLoader.GetVertices(), uploaders), false);
	N_RETURN(createIB(pCommandList, objLoader.GetNumIndices(), objLoader.GetIndices(), uploaders), false);

	// Create pipelines
	N_RETURN(createInputLayout(), false);
	N_RETURN(createPipelineLayouts(), false);
	N_RETURN(createPipelines(rtFormat, dsFormat), false);

	// Extract boundary
	const auto center = objLoader.GetCenter();
	m_bound = XMFLOAT4(center.x, center.y, center.z, objLoader.GetRadius());

	// Create output grids and build acceleration structures
	m_depthKBuffer = Texture2D::MakeUnique();
	N_RETURN(m_depthKBuffer->Create(m_device.Common, width, height, Format::R32_UINT, NUM_K_LAYERS,
		ResourceFlag::ALLOW_UNORDERED_ACCESS | ResourceFlag::ALLOW_SIMULTANEOUS_ACCESS), false);

	m_lsDepthKBuffer = Texture2D::MakeUnique();
	N_RETURN(m_lsDepthKBuffer->Create(m_device.Common, SHADOW_MAP_SIZE, SHADOW_MAP_SIZE, Format::R32_UINT,
		NUM_K_LAYERS, ResourceFlag::ALLOW_UNORDERED_ACCESS | ResourceFlag::ALLOW_SIMULTANEOUS_ACCESS), false);

	m_outputView = Texture2D::MakeUnique();
	N_RETURN(m_outputView->Create(m_device.Common, width, height, rtFormat, 1,
		ResourceFlag::ALLOW_UNORDERED_ACCESS), false);

	// Initialize world transform
	const auto world = XMMatrixIdentity();
	XMStoreFloat4x4(&m_world, XMMatrixTranspose(world));

	if (m_useRayTracing)
	{
		N_RETURN(buildAccelerationStructures(pCommandList, pGeometries), false);
		N_RETURN(buildShaderTables(), false);
	}
	else createDescriptorTables();

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
	if (m_useRayTracing)
	{
		RayGenConstants cbRayGen;
		cbRayGen.ScreenToWorld = m_cbPerObject.ScreenToWorld;
		XMStoreFloat4(&cbRayGen.LightDir, XMVector3Normalize(lightPt - focusPt));

		m_rayGenShaderTables[frameIndex]->Reset();
		m_rayGenShaderTables[frameIndex]->AddShaderRecord(*ShaderRecord::MakeUnique(m_device,
			m_rayTracingPipeline, RaygenShaderName, &cbRayGen, sizeof(cbRayGen)));
	}
}

void SparseVolume::Render(const RayTracing::CommandList* pCommandList, const Descriptor& rtv,
	const Descriptor& dsv, const Descriptor& lsDsv)
{
	const DescriptorPool descriptorPools[] = { m_descriptorTableCache->GetDescriptorPool(CBV_SRV_UAV_POOL) };
	pCommandList->SetDescriptorPools(static_cast<uint32_t>(size(descriptorPools)), descriptorPools);

	depthPeelLightSpace(pCommandList, lsDsv);
	depthPeel(pCommandList, dsv, false);

	render(pCommandList, rtv);
}

void SparseVolume::RenderDXR(const RayTracing::CommandList* pCommandList,
	uint32_t frameIndex, RenderTarget& dst, const Descriptor& dsv)
{
	const DescriptorPool descriptorPools[] = { m_descriptorTableCache->GetDescriptorPool(CBV_SRV_UAV_POOL) };
	pCommandList->SetDescriptorPools(static_cast<uint32_t>(size(descriptorPools)), descriptorPools);

	depthPeel(pCommandList, dsv);
	rayTrace(pCommandList, frameIndex);

	ResourceBarrier barriers[2];
	auto numBarriers = m_outputView->SetBarrier(barriers, ResourceState::COPY_SOURCE);
	numBarriers = dst.SetBarrier(barriers, ResourceState::COPY_DEST, numBarriers);

	TextureCopyLocation dstCopyLoc(dst.GetResource().get(), 0);
	TextureCopyLocation srcCopyLoc(m_outputView->GetResource().get(), 0);
	pCommandList->Barrier(numBarriers, barriers);
	pCommandList->CopyTextureRegion(dstCopyLoc, 0, 0, 0, srcCopyLoc);
}

bool SparseVolume::createVB(XUSG::CommandList* pCommandList, uint32_t numVert,
	uint32_t stride, const uint8_t* pData, vector<Resource>& uploaders)
{
	m_vertexBuffer = VertexBuffer::MakeUnique();
	N_RETURN(m_vertexBuffer->Create(m_device.Common, numVert, stride,
		ResourceFlag::NONE, MemoryType::DEFAULT), false);
	uploaders.push_back(nullptr);

	return m_vertexBuffer->Upload(pCommandList, uploaders.back(), pData,
		stride * numVert, 0, ResourceState::NON_PIXEL_SHADER_RESOURCE);
}

bool SparseVolume::createIB(XUSG::CommandList* pCommandList, uint32_t numIndices,
	const uint32_t* pData, vector<Resource>& uploaders)
{
	m_numIndices = numIndices;
	const uint32_t byteWidth = sizeof(uint32_t) * numIndices;

	m_indexBuffer = IndexBuffer::MakeUnique();
	N_RETURN(m_indexBuffer->Create(m_device.Common, byteWidth, Format::R32_UINT,
		ResourceFlag::NONE, MemoryType::DEFAULT), false);
	uploaders.push_back(nullptr);

	return m_indexBuffer->Upload(pCommandList, uploaders.back(), pData,
		byteWidth, 0, ResourceState::NON_PIXEL_SHADER_RESOURCE);
}

bool SparseVolume::createInputLayout()
{
	// Define the vertex input layout.
	InputElementTable inputElementDescs =
	{
		{ "POSITION",	0, Format::R32G32B32_FLOAT, 0, 0,						InputClassification::PER_VERTEX_DATA, 0 },
		{ "NORMAL",		0, Format::R32G32B32_FLOAT, 0, APPEND_ALIGNED_ELEMENT,	InputClassification::PER_VERTEX_DATA, 0 }
	};

	X_RETURN(m_inputLayout, m_graphicsPipelineCache->CreateInputLayout(inputElementDescs), false);

	return true;
}

bool SparseVolume::createPipelineLayouts()
{
	// Depth peeling pass
	{
		// Get pipeline layout
		const auto pipelineLayout = Util::PipelineLayout::MakeUnique();
		pipelineLayout->SetConstants(CONSTANTS, SizeOfInUint32(XMFLOAT4X4), 0);
		pipelineLayout->SetRange(SRV_UAVS, DescriptorType::UAV, 1, 0, 0,
			DescriptorFlag::DATA_STATIC_WHILE_SET_AT_EXECUTE);
		pipelineLayout->SetShaderStage(CONSTANTS, Shader::Stage::VS);
		pipelineLayout->SetShaderStage(SRV_UAVS, Shader::Stage::PS);
		X_RETURN(m_pipelineLayouts[DEPTH_PEEL_LAYOUT], pipelineLayout->GetPipelineLayout(*m_pipelineLayoutCache,
			PipelineLayoutFlag::ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT, L"DepthPeelingLayout"), false);
	}

	// Sparse volume rendering pass with shadow mapping
	{
		// Get pipeline layout
		const auto pipelineLayout = Util::PipelineLayout::MakeUnique();
		pipelineLayout->SetConstants(CONSTANTS, SizeOfInUint32(PerObjConstants), 0);
		pipelineLayout->SetRange(SRV_UAVS, DescriptorType::SRV, 2, 0);
		pipelineLayout->SetShaderStage(CONSTANTS, Shader::Stage::PS);
		pipelineLayout->SetShaderStage(SRV_UAVS, Shader::Stage::PS);
		X_RETURN(m_pipelineLayouts[SPARSE_RAYCAST_LAYOUT], pipelineLayout->GetPipelineLayout(*m_pipelineLayoutCache,
			PipelineLayoutFlag::NONE, L"SparseRayCastLayout"), false);
	}

	// Global pipeline layout
	// This is a pipeline layout that is shared across all raytracing shaders invoked during a DispatchRays() call.
	if (m_useRayTracing)
	{
		const auto pipelineLayout = RayTracing::PipelineLayout::MakeUnique();
		pipelineLayout->SetRange(OUTPUT_VIEW, DescriptorType::UAV, 1, 0);
		pipelineLayout->SetRootSRV(ACCELERATION_STRUCTURE, 0);
		pipelineLayout->SetRange(DEPTH_K_BUFFERS, DescriptorType::SRV, 1, 1);
		X_RETURN(m_pipelineLayouts[GLOBAL_LAYOUT], pipelineLayout->GetPipelineLayout(m_device, *m_pipelineLayoutCache,
			PipelineLayoutFlag::NONE, L"RayTracerGlobalPipelineLayout"), false);
	}

	// Local pipeline layout for RayGen shader
	// This is a pipeline layout that enables a shader to have unique arguments that come from shader tables.
	if (m_useRayTracing)
	{
		const auto pipelineLayout = RayTracing::PipelineLayout::MakeUnique();
		pipelineLayout->SetConstants(CONSTANTS, SizeOfInUint32(RayGenConstants), 0);
		X_RETURN(m_pipelineLayouts[RAY_GEN_LAYOUT], pipelineLayout->GetPipelineLayout(m_device, *m_pipelineLayoutCache,
			PipelineLayoutFlag::LOCAL_PIPELINE_LAYOUT, L"RayTracerRayGenPipelineLayout"), false);
	}

	return true;
}

bool SparseVolume::createPipelines(Format rtFormat, Format dsFormat)
{
	{
		N_RETURN(m_shaderPool->CreateShader(Shader::Stage::VS, VS_BASE_PASS, L"VSBasePass.cso"), false);
		N_RETURN(m_shaderPool->CreateShader(Shader::Stage::PS, PS_DEPTH_PEEL, L"PSDepthPeel.cso"), false);

		const auto state = Graphics::State::MakeUnique();
		state->SetPipelineLayout(m_pipelineLayouts[DEPTH_PEEL_LAYOUT]);
		state->SetShader(Shader::Stage::VS, m_shaderPool->GetShader(Shader::Stage::VS, VS_BASE_PASS));
		state->SetShader(Shader::Stage::PS, m_shaderPool->GetShader(Shader::Stage::PS, PS_DEPTH_PEEL));
		state->RSSetState(Graphics::RasterizerPreset::CULL_NONE, *m_graphicsPipelineCache);
		state->DSSetState(Graphics::DepthStencilPreset::DEPTH_READ_LESS, *m_graphicsPipelineCache);
		state->IASetInputLayout(m_inputLayout);
		state->IASetPrimitiveTopologyType(PrimitiveTopologyType::TRIANGLE);
		state->OMSetDSVFormat(dsFormat);

		X_RETURN(m_pipelines[DEPTH_PEEL], state->GetPipeline(*m_graphicsPipelineCache, L"DepthPeeling"), false);
	}

	{
		N_RETURN(m_shaderPool->CreateShader(Shader::Stage::VS, VS_SCREEN_QUAD, L"VSScreenQuad.cso"), false);
		N_RETURN(m_shaderPool->CreateShader(Shader::Stage::PS, PS_SPARSE_RAYCAST, L"PSSparseRayCast.cso"), false);

		const auto state = Graphics::State::MakeUnique();
		state->SetPipelineLayout(m_pipelineLayouts[SPARSE_RAYCAST_LAYOUT]);
		state->SetShader(Shader::Stage::VS, m_shaderPool->GetShader(Shader::Stage::VS, VS_SCREEN_QUAD));
		state->SetShader(Shader::Stage::PS, m_shaderPool->GetShader(Shader::Stage::PS, PS_SPARSE_RAYCAST));
		state->DSSetState(Graphics::DepthStencilPreset::DEPTH_STENCIL_NONE, *m_graphicsPipelineCache);
		state->IASetPrimitiveTopologyType(PrimitiveTopologyType::TRIANGLE);
		state->OMSetRTVFormats(&rtFormat, 1);

		X_RETURN(m_pipelines[SPARSE_RAYCAST], state->GetPipeline(*m_graphicsPipelineCache, L"SparseRayCast"), false);
	}

	if (m_useRayTracing)
	{
		N_RETURN(m_shaderPool->CreateShader(Shader::Stage::CS, 0, L"SparseRayCast.cso"), false);

		const auto state = RayTracing::State::MakeUnique();
		state->SetShaderLibrary(m_shaderPool->GetShader(Shader::Stage::CS, 0));
		state->SetHitGroup(0, HitGroupName, nullptr, AnyHitShaderName);
		state->SetShaderConfig(sizeof(float), sizeof(XMFLOAT2));
		state->SetLocalPipelineLayout(0, m_pipelineLayouts[RAY_GEN_LAYOUT],
			1, reinterpret_cast<const void**>(&RaygenShaderName));
		state->SetGlobalPipelineLayout(m_pipelineLayouts[GLOBAL_LAYOUT]);
		state->SetMaxRecursionDepth(1);
		m_rayTracingPipeline = state->GetPipeline(*m_rayTracingPipelineCache, L"SparseRayCastDXR");

		N_RETURN(m_rayTracingPipeline.Native, false);
	}

	return true;
}

bool SparseVolume::createDescriptorTables()
{
	// Acceleration structure UAVs
	if (m_useRayTracing)
	{
		const Descriptor descriptors[] = { m_bottomLevelAS->GetResult()->GetUAV(), m_topLevelAS->GetResult()->GetUAV() };
		const auto descriptorTable = Util::DescriptorTable::MakeUnique();
		descriptorTable->SetDescriptors(0, static_cast<uint32_t>(size(descriptors)), descriptors);
		const auto asTable = descriptorTable->GetCbvSrvUavTable(*m_descriptorTableCache);
		N_RETURN(asTable, false);
	}

	// Other UAVs
	{
		// Get UAV
		const auto descriptorTable = Util::DescriptorTable::MakeUnique();
		descriptorTable->SetDescriptors(0, 1, &m_depthKBuffer->GetUAV());
		X_RETURN(m_uavTables[UAV_TABLE_KBUFFER], descriptorTable->GetCbvSrvUavTable(*m_descriptorTableCache), false);
	}

	{
		// Get UAV
		const auto descriptorTable = Util::DescriptorTable::MakeUnique();
		descriptorTable->SetDescriptors(0, 1, &m_lsDepthKBuffer->GetUAV());
		X_RETURN(m_uavTables[UAV_TABLE_LS_KBUFFER], descriptorTable->GetCbvSrvUavTable(*m_descriptorTableCache), false);
	}

	{
		// Output UAV
		const auto descriptorTable = Util::DescriptorTable::MakeUnique();
		descriptorTable->SetDescriptors(0, 1, &m_outputView->GetUAV());
		X_RETURN(m_uavTables[UAV_TABLE_OUT_VIEW], descriptorTable->GetCbvSrvUavTable(*m_descriptorTableCache), false);
	}

	// Depth K-buffer SRV
	const Descriptor descriptors[] = { m_depthKBuffer->GetSRV(), m_lsDepthKBuffer->GetSRV() };
	const auto descriptorTable = Util::DescriptorTable::MakeUnique();
	descriptorTable->SetDescriptors(0, static_cast<uint32_t>(size(descriptors)), descriptors);
	X_RETURN(m_srvTable, descriptorTable->GetCbvSrvUavTable(*m_descriptorTableCache), false);

	// Create the sampler table
	/*{
		const auto descriptorTable = Util::DescriptorTable::MakeUnique();
		const auto sampler = LINEAR_CLAMP;
		descriptorTable->SetSamplers(0, 1, &sampler, *m_descriptorTableCache);
		X_RETURN(m_samplerTable, descriptorTable->GetSamplerTable(*m_descriptorTableCache), false);
	}*/

	return true;
}

bool SparseVolume::buildAccelerationStructures(const RayTracing::CommandList* pCommandList, Geometry* geometries)
{
	// Set geometries
	const auto geometryFlags = GeometryFlags::NONE;
	BottomLevelAS::SetTriangleGeometries(geometries, 1, Format::R32G32B32_FLOAT,
		&m_vertexBuffer->GetVBV(), &m_indexBuffer->GetIBV(), &geometryFlags);

	// Descriptor index in descriptor pool
	const auto bottomLevelASIndex = 0u;
	const auto topLevelASIndex = bottomLevelASIndex + 1;

	// Prebuild
	m_bottomLevelAS = BottomLevelAS::MakeUnique();
	m_topLevelAS = TopLevelAS::MakeUnique();
	N_RETURN(m_bottomLevelAS->PreBuild(m_device, 1, geometries, bottomLevelASIndex), false);
	N_RETURN(m_topLevelAS->PreBuild(m_device, 1, topLevelASIndex), false);

	// Create scratch buffer
	auto scratchSize = m_topLevelAS->GetScratchDataMaxSize();
	scratchSize = (max)(m_bottomLevelAS->GetScratchDataMaxSize(), scratchSize);
	N_RETURN(AccelerationStructure::AllocateUAVBuffer(m_device, m_scratch, scratchSize), false);

	// Get descriptor pool and create descriptor tables
	N_RETURN(createDescriptorTables(), false);
	const auto& descriptorPool = m_descriptorTableCache->GetDescriptorPool(CBV_SRV_UAV_POOL);

	// Set instance
	float* const pTransform[] = { reinterpret_cast<float*>(&m_world) };
	const BottomLevelAS* ppBottomLevelAS[] = { m_bottomLevelAS.get() };
	TopLevelAS::SetInstances(m_device, m_instances, 1, ppBottomLevelAS, pTransform);

	// Build bottom level ASs
	m_bottomLevelAS->Build(pCommandList, m_scratch, descriptorPool);

	// Build top level AS
	m_topLevelAS->Build(pCommandList, m_scratch, m_instances, descriptorPool);

	// Set resource barriers
	ResourceBarrier barriers[2];
	auto numBarriers = m_vertexBuffer->SetBarrier(barriers, ResourceState::VERTEX_AND_CONSTANT_BUFFER);
	numBarriers = m_indexBuffer->SetBarrier(barriers, ResourceState::INDEX_BUFFER, numBarriers);
	pCommandList->Barrier(numBarriers, barriers);

	return true;
}

bool SparseVolume::buildShaderTables()
{
	// Get shader identifiers.
	const auto shaderIDSize = ShaderRecord::GetShaderIDSize(m_device);

	for (auto i = 0ui8; i < FrameCount; ++i)
	{
		// Ray gen shader table
		m_rayGenShaderTables[i] = ShaderTable::MakeUnique();
		N_RETURN(m_rayGenShaderTables[i]->Create(m_device, 1, shaderIDSize + sizeof(RayGenConstants),
			(L"RayGenShaderTable" + to_wstring(i)).c_str()), false);
		N_RETURN(m_rayGenShaderTables[i]->AddShaderRecord(*ShaderRecord::MakeUnique(m_device,
			m_rayTracingPipeline, RaygenShaderName, &RayGenConstants(), sizeof(RayGenConstants))), false);
	}

	// Hit group shader table
	m_hitGroupShaderTable = ShaderTable::MakeUnique();
	N_RETURN(m_hitGroupShaderTable->Create(m_device, 1, shaderIDSize, L"HitGroupShaderTable"), false);
	N_RETURN(m_hitGroupShaderTable->AddShaderRecord(*ShaderRecord::MakeUnique(m_device, m_rayTracingPipeline, HitGroupName)), false);

	// Miss shader table
	m_missShaderTable = ShaderTable::MakeUnique();
	N_RETURN(m_missShaderTable->Create(m_device, 1, shaderIDSize, L"MissShaderTable"), false);
	N_RETURN(m_missShaderTable->AddShaderRecord(*ShaderRecord::MakeUnique(m_device, m_rayTracingPipeline, MissShaderName)), false);

	return true;
}

void SparseVolume::depthPeel(const RayTracing::CommandList* pCommandList,
	const Descriptor& dsv, bool setPipeline)
{
	// Set resource barrier
	ResourceBarrier barrier;
	m_depthKBuffer->SetBarrier(&barrier, ResourceState::UNORDERED_ACCESS); // Auto promotion

	// Set descriptor tables
	pCommandList->SetGraphicsPipelineLayout(m_pipelineLayouts[DEPTH_PEEL_LAYOUT]);
	pCommandList->SetGraphics32BitConstants(CONSTANTS, SizeOfInUint32(XMFLOAT4X4), &m_worldViewProj);
	pCommandList->SetGraphicsDescriptorTable(SRV_UAVS, m_uavTables[UAV_TABLE_KBUFFER]);

	// Set pipeline state
	if (setPipeline) pCommandList->SetPipelineState(m_pipelines[DEPTH_PEEL]);

	// Set viewport
	Viewport viewport(0.0f, 0.0f, m_viewport.x, m_viewport.y);
	RectRange scissorRect(0, 0, static_cast<long>(m_viewport.x), static_cast<long>(m_viewport.y));
	pCommandList->RSSetViewports(1, &viewport);
	pCommandList->RSSetScissorRects(1, &scissorRect);

	const auto maxDepth = 1.0f;
	pCommandList->OMSetRenderTargets(0, nullptr, &dsv);
	pCommandList->ClearUnorderedAccessViewUint(m_uavTables[UAV_TABLE_KBUFFER], m_depthKBuffer->GetUAV(),
		m_depthKBuffer->GetResource(), XMVECTORU32{ reinterpret_cast<const uint32_t&>(maxDepth) }.u);

	// Record commands.
	pCommandList->IASetVertexBuffers(0, 1, &m_vertexBuffer->GetVBV());
	pCommandList->IASetIndexBuffer(m_indexBuffer->GetIBV());
	pCommandList->IASetPrimitiveTopology(PrimitiveTopology::TRIANGLELIST);
	pCommandList->DrawIndexed(m_numIndices, 1, 0, 0, 0);
}

void SparseVolume::depthPeelLightSpace(const RayTracing::CommandList* pCommandList, const Descriptor& dsv)
{
	// Set resource barrier
	ResourceBarrier barrier;
	m_lsDepthKBuffer->SetBarrier(&barrier, ResourceState::UNORDERED_ACCESS); // Auto promotion

	// Set descriptor tables
	pCommandList->SetGraphicsPipelineLayout(m_pipelineLayouts[DEPTH_PEEL_LAYOUT]);

	pCommandList->SetGraphics32BitConstants(CONSTANTS, SizeOfInUint32(XMFLOAT4X4), &m_worldViewProjLS);
	pCommandList->SetGraphicsDescriptorTable(SRV_UAVS, m_uavTables[UAV_TABLE_LS_KBUFFER]);

	// Set pipeline state
	pCommandList->SetPipelineState(m_pipelines[DEPTH_PEEL]);

	// Set viewport
	Viewport viewport(0.0f, 0.0f, static_cast<float>(SHADOW_MAP_SIZE), static_cast<float>(SHADOW_MAP_SIZE));
	RectRange scissorRect(0, 0, SHADOW_MAP_SIZE, SHADOW_MAP_SIZE);
	pCommandList->RSSetViewports(1, &viewport);
	pCommandList->RSSetScissorRects(1, &scissorRect);

	const auto maxDepth = 1.0f;
	pCommandList->OMSetRenderTargets(0, nullptr, &dsv);
	pCommandList->ClearUnorderedAccessViewUint(m_uavTables[UAV_TABLE_LS_KBUFFER], m_lsDepthKBuffer->GetUAV(),
		m_lsDepthKBuffer->GetResource(), XMVECTORU32{ reinterpret_cast<const uint32_t&>(maxDepth) }.u);

	// Record commands.
	pCommandList->IASetVertexBuffers(0, 1, &m_vertexBuffer->GetVBV());
	pCommandList->IASetIndexBuffer(m_indexBuffer->GetIBV());
	pCommandList->IASetPrimitiveTopology(PrimitiveTopology::TRIANGLELIST);
	pCommandList->DrawIndexed(m_numIndices, 1, 0, 0, 0);
}

void SparseVolume::render(const RayTracing::CommandList* pCommandList, const Descriptor& rtv)
{
	// Set resource barriers
	ResourceBarrier barriers[2];
	auto numBarriers = m_depthKBuffer->SetBarrier(barriers, ResourceState::PIXEL_SHADER_RESOURCE);
	numBarriers = m_lsDepthKBuffer->SetBarrier(barriers, ResourceState::PIXEL_SHADER_RESOURCE, numBarriers);
	pCommandList->Barrier(numBarriers, barriers);

	// Set descriptor tables
	pCommandList->SetGraphicsPipelineLayout(m_pipelineLayouts[SPARSE_RAYCAST_LAYOUT]);

	pCommandList->SetGraphics32BitConstants(CONSTANTS, SizeOfInUint32(PerObjConstants), &m_cbPerObject);
	pCommandList->SetGraphicsDescriptorTable(SRV_UAVS, m_srvTable);

	// Set pipeline state
	pCommandList->SetPipelineState(m_pipelines[SPARSE_RAYCAST]);

	// Set viewport
	Viewport viewport(0.0f, 0.0f, m_viewport.x, m_viewport.y);
	RectRange scissorRect(0, 0, static_cast<long>(m_viewport.x), static_cast<long>(m_viewport.y));
	pCommandList->RSSetViewports(1, &viewport);
	pCommandList->RSSetScissorRects(1, &scissorRect);

	pCommandList->OMSetRenderTargets(1, &rtv);

	// Record commands.
	pCommandList->IASetPrimitiveTopology(PrimitiveTopology::TRIANGLESTRIP);
	pCommandList->Draw(3, 1, 0, 0);
}

void SparseVolume::rayTrace(const RayTracing::CommandList* pCommandList, uint32_t frameIndex)
{
	// Set resource barrier
	ResourceBarrier barrier;
	const auto numBarriers = m_outputView->SetBarrier(&barrier, ResourceState::UNORDERED_ACCESS);
	pCommandList->Barrier(numBarriers, &barrier);

	// Set descriptor tables
	pCommandList->SetComputePipelineLayout(m_pipelineLayouts[GLOBAL_LAYOUT]);
	pCommandList->SetComputeDescriptorTable(OUTPUT_VIEW, m_uavTables[UAV_TABLE_OUT_VIEW]);
	pCommandList->SetTopLevelAccelerationStructure(ACCELERATION_STRUCTURE, *m_topLevelAS);
	pCommandList->SetComputeDescriptorTable(DEPTH_K_BUFFERS, m_srvTable);

	pCommandList->ClearUnorderedAccessViewFloat(m_uavTables[UAV_TABLE_OUT_VIEW], m_outputView->GetUAV(),
		m_outputView->GetResource(), XMVECTORF32{ 0.0f });

	// Fallback layer has no depth
	pCommandList->DispatchRays(m_rayTracingPipeline, (uint32_t)m_viewport.x, (uint32_t)m_viewport.y, 1,
		*m_hitGroupShaderTable, *m_missShaderTable, *m_rayGenShaderTables[frameIndex]);
}
