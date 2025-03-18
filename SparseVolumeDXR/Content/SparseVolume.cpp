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

struct CBPerFrame
{
	DirectX::XMFLOAT4X4	ScreenToWorld;
	DirectX::XMFLOAT4X4	ViewProjLS;
};

struct RayGenConstants
{
	DirectX::XMFLOAT4X4	ScreenToWorld;
	DirectX::XMFLOAT4	LightDir;
};

const wchar_t* SparseVolume::HitGroupName = L"hitGroup";
const wchar_t* SparseVolume::RaygenShaderName = L"raygenMain";
const wchar_t* SparseVolume::AnyHitShaderName = L"anyHitMain";
const wchar_t* SparseVolume::MissShaderName = L"missMain";

SparseVolume::SparseVolume() :
	m_instances()
{
	m_shaderLib = ShaderLib::MakeUnique();
}

SparseVolume::~SparseVolume()
{
}

bool SparseVolume::Init(RayTracing::CommandList* pCommandList, const DescriptorTableLib::sptr& descriptorTableLib,
	uint32_t width, uint32_t height, Format rtFormat, Format dsFormat, vector<Resource::uptr>& uploaders,
	GeometryBuffer* pGeometry, const char* fileName, const XMFLOAT4& posScale)
{
	const auto pDevice = pCommandList->GetRTDevice();
	m_rayTracingPipelineLib = RayTracing::PipelineLib::MakeUnique(pDevice);
	m_graphicsPipelineLib = Graphics::PipelineLib::MakeUnique(pDevice);
	m_computePipelineLib = Compute::PipelineLib::MakeUnique(pDevice);
	m_pipelineLayoutLib = PipelineLayoutLib::MakeUnique(pDevice);
	m_descriptorTableLib = descriptorTableLib;

	m_viewport.x = static_cast<float>(width);
	m_viewport.y = static_cast<float>(height);
	m_posScale = posScale;

	m_useRayTracing = pGeometry;

	// Load inputs
	ObjLoader objLoader;
	if (!objLoader.Import(fileName, true, true)) return false;
	XUSG_N_RETURN(createVB(pCommandList, objLoader.GetNumVertices(), objLoader.GetVertexStride(), objLoader.GetVertices(), uploaders), false);
	XUSG_N_RETURN(createIB(pCommandList, objLoader.GetNumIndices(), objLoader.GetIndices(), uploaders), false);

	// Extract boundary
	const auto& aabb = objLoader.GetAABB();
	const XMFLOAT3 ext(aabb.Max.x - aabb.Min.x, aabb.Max.y - aabb.Min.y, aabb.Max.z - aabb.Min.z);
	m_bound.x = (aabb.Max.x + aabb.Min.x) / 2.0f;
	m_bound.y = (aabb.Max.y + aabb.Min.y) / 2.0f;
	m_bound.z = (aabb.Max.z + aabb.Min.z) / 2.0f;
	m_bound.w = (max)(ext.x, (max)(ext.y, ext.z)) / 2.0f;

	// Create output grids and build acceleration structures
	m_depthKBuffer = Texture2D::MakeUnique();
	XUSG_N_RETURN(m_depthKBuffer->Create(pDevice, width, height, Format::R32_UINT, NUM_K_LAYERS,
		ResourceFlag::ALLOW_UNORDERED_ACCESS | ResourceFlag::ALLOW_SIMULTANEOUS_ACCESS), false);

	m_lsDepthKBuffer = Texture2D::MakeUnique();
	XUSG_N_RETURN(m_lsDepthKBuffer->Create(pDevice, SHADOW_MAP_SIZE, SHADOW_MAP_SIZE, Format::R32_UINT,
		NUM_K_LAYERS, ResourceFlag::ALLOW_UNORDERED_ACCESS | ResourceFlag::ALLOW_SIMULTANEOUS_ACCESS), false);

	m_outputView = Texture2D::MakeUnique();
	XUSG_N_RETURN(m_outputView->Create(pDevice, width, height, rtFormat, 1,
		ResourceFlag::ALLOW_UNORDERED_ACCESS), false);

	// Create constant buffers
	m_cbDepthPeel = ConstantBuffer::MakeUnique();
	XUSG_N_RETURN(m_cbDepthPeel->Create(pDevice, sizeof(XMFLOAT4X4[FrameCount]), FrameCount,
		nullptr, MemoryType::UPLOAD, MemoryFlag::NONE, L"CBDepthPeel"), false);

	m_cbDepthPeelLS = ConstantBuffer::MakeUnique();
	XUSG_N_RETURN(m_cbDepthPeelLS->Create(pDevice, sizeof(XMFLOAT4X4[FrameCount]), FrameCount,
		nullptr, MemoryType::UPLOAD, MemoryFlag::NONE, L"CBDepthPeelLS"), false);

	m_cbPerFrame = ConstantBuffer::MakeUnique();
	XUSG_N_RETURN(m_cbPerFrame->Create(pDevice, sizeof(CBPerFrame[FrameCount]), FrameCount,
		nullptr, MemoryType::UPLOAD, MemoryFlag::NONE, L"CBPerFrame"), false);

	// Initialize world transform
	XMStoreFloat3x4(&m_world, XMMatrixIdentity());

	// Create input layout and descriptor tables
	XUSG_N_RETURN(createInputLayout(), false);
	XUSG_N_RETURN(createDescriptorTables(), false);
	if (m_useRayTracing)
	{
		// Build ASes, create pipelines, and build shader tables
		XUSG_N_RETURN(buildAccelerationStructures(pCommandList, pGeometry), false);
		XUSG_N_RETURN(createPipelineLayouts(pDevice), false);
		XUSG_N_RETURN(createPipelines(rtFormat, dsFormat), false);
		XUSG_N_RETURN(buildShaderTables(pDevice), false);
	}
	else
	{
		// Create pipelines
		XUSG_N_RETURN(createPipelineLayouts(pDevice), false);
		XUSG_N_RETURN(createPipelines(rtFormat, dsFormat), false);
	}

	return true;
}

void SparseVolume::UpdateFrame(const RayTracing::Device* pDevice, uint8_t frameIndex, CXMMATRIX viewProj)
{
	// General matrices
	//const auto world = XMMatrixScaling(m_bound.w, m_bound.w, m_bound.w) *
		//XMMatrixTranslation(m_bound.x, m_bound.y, m_bound.z);
	const auto world = XMMatrixScaling(m_posScale.w, m_posScale.w, m_posScale.w) *
		XMMatrixTranslation(m_posScale.x, m_posScale.y, m_posScale.z);
	XMStoreFloat3x4(&m_world, world);
	{
		const auto pCbData = reinterpret_cast<XMFLOAT4X4*>(m_cbDepthPeel->Map(frameIndex));
		XMStoreFloat4x4(pCbData, XMMatrixTranspose(world * viewProj));
	}

	// Light-space matrices
	const auto focusPt = XMLoadFloat4(&m_bound);
	const auto lightPt = XMVectorSet(-10.0f, 45.0f, -75.0f, 0.0f) + focusPt;
	const auto viewLS = XMMatrixLookAtLH(lightPt, focusPt, XMVectorSet(0.0f, 1.0f, 0.0f, 0.0f));
	const auto projLS = XMMatrixOrthographicLH(m_bound.w * 3.0f, m_bound.w * 3.0f, g_zNearLS, g_zFarLS);
	const auto viewProjLS = viewLS * projLS;
	const auto pCbData = reinterpret_cast<CBPerFrame*>(m_cbPerFrame->Map(frameIndex));
	XMStoreFloat4x4(&pCbData->ViewProjLS, XMMatrixTranspose(viewProjLS));
	{
		const auto pCbData = reinterpret_cast<XMFLOAT4X4*>(m_cbDepthPeelLS->Map(frameIndex));
		XMStoreFloat4x4(pCbData, XMMatrixTranspose(world * viewProjLS));
	}

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
	XMStoreFloat4x4(&pCbData->ScreenToWorld, XMMatrixTranspose(screenToWorld));

	// Ray tracing
	if (m_useRayTracing)
	{
		RayGenConstants cbRayGen;
		cbRayGen.ScreenToWorld = pCbData->ScreenToWorld;
		XMStoreFloat4(&cbRayGen.LightDir, XMVector3Normalize(lightPt - focusPt));

		m_rayGenShaderTables[frameIndex]->Reset();
		m_rayGenShaderTables[frameIndex]->AddShaderRecord(ShaderRecord::MakeUnique(pDevice,
			m_pipelines[RAY_TRACING], RaygenShaderName, &cbRayGen, sizeof(cbRayGen)).get());
	}
}

void SparseVolume::Render(RayTracing::CommandList* pCommandList, uint8_t frameIndex,
	const Descriptor& rtv, const Descriptor& dsv, const Descriptor& lsDsv)
{
	depthPeelLightSpace(pCommandList, frameIndex, lsDsv);
	depthPeel(pCommandList, frameIndex, dsv, false);

	render(pCommandList, frameIndex, rtv);
}

void SparseVolume::RenderDXR(RayTracing::CommandList* pCommandList,
	uint8_t frameIndex, RenderTarget* pDst, const Descriptor& dsv)
{
	depthPeel(pCommandList, frameIndex, dsv);
	rayTrace(pCommandList, frameIndex);

	ResourceBarrier barriers[2];
	auto numBarriers = m_outputView->SetBarrier(barriers, ResourceState::COPY_SOURCE);
	numBarriers = pDst->SetBarrier(barriers, ResourceState::COPY_DEST, numBarriers);

	TextureCopyLocation dstCopyLoc(pDst, 0);
	TextureCopyLocation srcCopyLoc(m_outputView.get(), 0);
	pCommandList->Barrier(numBarriers, barriers);
	pCommandList->CopyTextureRegion(dstCopyLoc, 0, 0, 0, srcCopyLoc);
}

bool SparseVolume::createVB(XUSG::CommandList* pCommandList, uint32_t numVert,
	uint32_t stride, const uint8_t* pData, vector<Resource::uptr>& uploaders)
{
	m_vertexBuffer = VertexBuffer::MakeUnique();
	XUSG_N_RETURN(m_vertexBuffer->Create(pCommandList->GetDevice(), numVert, stride,
		ResourceFlag::NONE, MemoryType::DEFAULT), false);
	uploaders.emplace_back(Resource::MakeUnique());

	return m_vertexBuffer->Upload(pCommandList, uploaders.back().get(), pData,
		stride * numVert, 0, ResourceState::NON_PIXEL_SHADER_RESOURCE);
}

bool SparseVolume::createIB(XUSG::CommandList* pCommandList, uint32_t numIndices,
	const uint32_t* pData, vector<Resource::uptr>& uploaders)
{
	m_numIndices = numIndices;
	const uint32_t byteWidth = sizeof(uint32_t) * numIndices;

	m_indexBuffer = IndexBuffer::MakeUnique();
	XUSG_N_RETURN(m_indexBuffer->Create(pCommandList->GetDevice(), byteWidth, Format::R32_UINT,
		ResourceFlag::NONE, MemoryType::DEFAULT), false);
	uploaders.emplace_back(Resource::MakeUnique());

	return m_indexBuffer->Upload(pCommandList, uploaders.back().get(), pData,
		byteWidth, 0, ResourceState::NON_PIXEL_SHADER_RESOURCE);
}

bool SparseVolume::createInputLayout()
{
	// Define the vertex input layout.
	const InputElement inputElements[] =
	{
		{ "POSITION",	0, Format::R32G32B32_FLOAT, 0, 0,							InputClassification::PER_VERTEX_DATA, 0 },
		{ "NORMAL",		0, Format::R32G32B32_FLOAT, 0, XUSG_APPEND_ALIGNED_ELEMENT,	InputClassification::PER_VERTEX_DATA, 0 }
	};

	XUSG_X_RETURN(m_pInputLayout, m_graphicsPipelineLib->CreateInputLayout(inputElements, static_cast<uint32_t>(size(inputElements))), false);

	return true;
}

bool SparseVolume::createPipelineLayouts(const RayTracing::Device* pDevice)
{
	// Depth peeling pass
	{
		// Get pipeline layout
		const auto pipelineLayout = Util::PipelineLayout::MakeUnique();
		pipelineLayout->SetRootCBV(CONSTANTS, 0, 0, Shader::Stage::VS);
		pipelineLayout->SetRange(SRV_UAVS, DescriptorType::UAV, 1, 0, 0, DescriptorFlag::DATA_STATIC_WHILE_SET_AT_EXECUTE);
		pipelineLayout->SetShaderStage(SRV_UAVS, Shader::Stage::PS);
		XUSG_X_RETURN(m_pipelineLayouts[DEPTH_PEEL_LAYOUT], pipelineLayout->GetPipelineLayout(m_pipelineLayoutLib.get(),
			PipelineLayoutFlag::ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT, L"DepthPeelingLayout"), false);
	}

	// Sparse volume rendering pass with shadow mapping
	{
		// Get pipeline layout
		const auto pipelineLayout = Util::PipelineLayout::MakeUnique();
		pipelineLayout->SetRootCBV(CONSTANTS, 0, 0, Shader::Stage::PS);
		pipelineLayout->SetRange(SRV_UAVS, DescriptorType::SRV, 2, 0);
		pipelineLayout->SetShaderStage(SRV_UAVS, Shader::Stage::PS);
		XUSG_X_RETURN(m_pipelineLayouts[SPARSE_RAYCAST_LAYOUT], pipelineLayout->GetPipelineLayout(m_pipelineLayoutLib.get(),
			PipelineLayoutFlag::NONE, L"SparseRayCastLayout"), false);
	}

	// Global pipeline layout
	// This is a pipeline layout that is shared across all raytracing shaders invoked during a DispatchRays() call.
	if (m_useRayTracing)
	{
		const auto pipelineLayout = RayTracing::PipelineLayout::MakeUnique();
		pipelineLayout->SetRange(OUTPUT_VIEW, DescriptorType::UAV, 1, 0);
		pipelineLayout->SetRootSRV(ACCELERATION_STRUCTURE, 0, 0, DescriptorFlag::DATA_STATIC);
		pipelineLayout->SetRange(DEPTH_K_BUFFERS, DescriptorType::SRV, 1, 1);
		XUSG_X_RETURN(m_pipelineLayouts[GLOBAL_LAYOUT], pipelineLayout->GetPipelineLayout(pDevice, m_pipelineLayoutLib.get(),
			PipelineLayoutFlag::NONE, L"RayTracerGlobalPipelineLayout"), false);
	}

	// Local pipeline layout for RayGen shader
	// This is a pipeline layout that enables a shader to have unique arguments that come from shader tables.
	if (m_useRayTracing)
	{
		const auto pipelineLayout = RayTracing::PipelineLayout::MakeUnique();
		pipelineLayout->SetConstants(CONSTANTS, XUSG_UINT32_SIZE_OF(RayGenConstants), 0);
		XUSG_X_RETURN(m_pipelineLayouts[RAY_GEN_LAYOUT], pipelineLayout->GetPipelineLayout(pDevice, m_pipelineLayoutLib.get(),
			PipelineLayoutFlag::LOCAL_PIPELINE_LAYOUT, L"RayTracerRayGenPipelineLayout"), false);
	}

	return true;
}

bool SparseVolume::createPipelines(Format rtFormat, Format dsFormat)
{
	{
		XUSG_N_RETURN(m_shaderLib->CreateShader(Shader::Stage::VS, VS_BASE_PASS, L"VSBasePass.cso"), false);
		XUSG_N_RETURN(m_shaderLib->CreateShader(Shader::Stage::PS, PS_DEPTH_PEEL, L"PSDepthPeel.cso"), false);

		const auto state = Graphics::State::MakeUnique();
		state->SetPipelineLayout(m_pipelineLayouts[DEPTH_PEEL_LAYOUT]);
		state->SetShader(Shader::Stage::VS, m_shaderLib->GetShader(Shader::Stage::VS, VS_BASE_PASS));
		state->SetShader(Shader::Stage::PS, m_shaderLib->GetShader(Shader::Stage::PS, PS_DEPTH_PEEL));
		state->RSSetState(Graphics::RasterizerPreset::CULL_NONE, m_graphicsPipelineLib.get());
		state->DSSetState(Graphics::DepthStencilPreset::DEPTH_READ_LESS, m_graphicsPipelineLib.get());
		state->IASetInputLayout(m_pInputLayout);
		state->IASetPrimitiveTopologyType(PrimitiveTopologyType::TRIANGLE);
		state->OMSetDSVFormat(dsFormat);

		XUSG_X_RETURN(m_pipelines[DEPTH_PEEL], state->GetPipeline(m_graphicsPipelineLib.get(), L"DepthPeeling"), false);
	}

	{
		XUSG_N_RETURN(m_shaderLib->CreateShader(Shader::Stage::VS, VS_SCREEN_QUAD, L"VSScreenQuad.cso"), false);
		XUSG_N_RETURN(m_shaderLib->CreateShader(Shader::Stage::PS, PS_SPARSE_RAYCAST, L"PSSparseRayCast.cso"), false);

		const auto state = Graphics::State::MakeUnique();
		state->SetPipelineLayout(m_pipelineLayouts[SPARSE_RAYCAST_LAYOUT]);
		state->SetShader(Shader::Stage::VS, m_shaderLib->GetShader(Shader::Stage::VS, VS_SCREEN_QUAD));
		state->SetShader(Shader::Stage::PS, m_shaderLib->GetShader(Shader::Stage::PS, PS_SPARSE_RAYCAST));
		state->DSSetState(Graphics::DepthStencilPreset::DEPTH_STENCIL_NONE, m_graphicsPipelineLib.get());
		state->IASetPrimitiveTopologyType(PrimitiveTopologyType::TRIANGLE);
		state->OMSetRTVFormats(&rtFormat, 1);

		XUSG_X_RETURN(m_pipelines[SPARSE_RAYCAST], state->GetPipeline(m_graphicsPipelineLib.get(), L"SparseRayCast"), false);
	}

	if (m_useRayTracing)
	{
		XUSG_N_RETURN(m_shaderLib->CreateShader(Shader::Stage::CS, 0, L"SparseRayCast.cso"), false);
		const wchar_t* shaderNames[] = { RaygenShaderName, AnyHitShaderName, MissShaderName };

		const auto state = RayTracing::State::MakeUnique();
		state->SetShaderLibrary(0, m_shaderLib->GetShader(Shader::Stage::CS, 0),
			static_cast<uint32_t>(size(shaderNames)), shaderNames);
		state->SetHitGroup(0, HitGroupName, nullptr, AnyHitShaderName);
		state->SetShaderConfig(sizeof(float), sizeof(XMFLOAT2));
		state->SetLocalPipelineLayout(0, m_pipelineLayouts[RAY_GEN_LAYOUT],	1, &RaygenShaderName);
		state->SetGlobalPipelineLayout(m_pipelineLayouts[GLOBAL_LAYOUT]);
		state->SetMaxRecursionDepth(1);
		XUSG_X_RETURN(m_pipelines[RAY_TRACING], state->GetPipeline(m_rayTracingPipelineLib.get(), L"SparseRayCastDXR"), false);
	}

	return true;
}

bool SparseVolume::createDescriptorTables()
{
	// K-buffer and output UAVs
	{
		// Get UAV
		const auto descriptorTable = Util::DescriptorTable::MakeUnique();
		descriptorTable->SetDescriptors(0, 1, &m_depthKBuffer->GetUAV());
		XUSG_X_RETURN(m_uavTables[UAV_TABLE_KBUFFER], descriptorTable->GetCbvSrvUavTable(m_descriptorTableLib.get()), false);
	}

	{
		// Get UAV
		const auto descriptorTable = Util::DescriptorTable::MakeUnique();
		descriptorTable->SetDescriptors(0, 1, &m_lsDepthKBuffer->GetUAV());
		XUSG_X_RETURN(m_uavTables[UAV_TABLE_LS_KBUFFER], descriptorTable->GetCbvSrvUavTable(m_descriptorTableLib.get()), false);
	}

	{
		// Output UAV
		const auto descriptorTable = Util::DescriptorTable::MakeUnique();
		descriptorTable->SetDescriptors(0, 1, &m_outputView->GetUAV());
		XUSG_X_RETURN(m_uavTables[UAV_TABLE_OUT_VIEW], descriptorTable->GetCbvSrvUavTable(m_descriptorTableLib.get()), false);
	}

	// Depth K-buffer SRV
	const Descriptor descriptors[] = { m_depthKBuffer->GetSRV(), m_lsDepthKBuffer->GetSRV() };
	const auto descriptorTable = Util::DescriptorTable::MakeUnique();
	descriptorTable->SetDescriptors(0, static_cast<uint32_t>(size(descriptors)), descriptors);
	XUSG_X_RETURN(m_srvTable, descriptorTable->GetCbvSrvUavTable(m_descriptorTableLib.get()), false);

	// Create the sampler table
	/*{
		const auto descriptorTable = Util::DescriptorTable::MakeUnique();
		const auto sampler = LINEAR_CLAMP;
		descriptorTable->SetSamplers(0, 1, &sampler, m_descriptorTableCache.get());
		XUSG_X_RETURN(m_samplerTable, descriptorTable->GetSamplerTable(m_descriptorTableCache.get()), false);
	}*/

	return true;
}

bool SparseVolume::buildAccelerationStructures(RayTracing::CommandList* pCommandList, GeometryBuffer* pGeometry)
{
	const auto pDevice = pCommandList->GetRTDevice();

	// Set geometries
	const auto geometryFlags = GeometryFlag::NONE;
	BottomLevelAS::SetTriangleGeometries(*pGeometry, 1, Format::R32G32B32_FLOAT,
		&m_vertexBuffer->GetVBV(), &m_indexBuffer->GetIBV(), &geometryFlags);

	// Prebuild
	m_bottomLevelAS = BottomLevelAS::MakeUnique();
	m_topLevelAS = TopLevelAS::MakeUnique();
	XUSG_N_RETURN(m_bottomLevelAS->Prebuild(pDevice, 1, *pGeometry), false);
	XUSG_N_RETURN(m_topLevelAS->Prebuild(pDevice, 1), false);

	// Allocate AS buffers
	XUSG_N_RETURN(m_bottomLevelAS->Allocate(pDevice, m_descriptorTableLib.get()), false);
	XUSG_N_RETURN(m_topLevelAS->Allocate(pDevice, m_descriptorTableLib.get()), false);

	// Create scratch buffer
	auto scratchSize = m_topLevelAS->GetScratchDataByteSize();
	scratchSize = (max)(m_bottomLevelAS->GetScratchDataByteSize(), scratchSize);
	m_scratch = Buffer::MakeUnique();
	XUSG_N_RETURN(AccelerationStructure::AllocateUAVBuffer(pDevice, m_scratch.get(), scratchSize), false);

	// Set instance
	float* const pTransform[] = { reinterpret_cast<float*>(&m_world) };
	m_instances = Buffer::MakeUnique();
	const BottomLevelAS* ppBottomLevelAS[] = { m_bottomLevelAS.get() };
	TopLevelAS::SetInstances(pDevice, m_instances.get(), 1, ppBottomLevelAS, pTransform);

	// Build bottom level ASs
	m_bottomLevelAS->Build(pCommandList, m_scratch.get());

	const ResourceBarrier barrier = { nullptr, ResourceState::UNORDERED_ACCESS };
	pCommandList->Barrier(1, &barrier);

	// Build top level AS
	m_topLevelAS->Build(pCommandList, m_scratch.get(), m_instances.get(),
		m_descriptorTableLib->GetDescriptorHeap(CBV_SRV_UAV_HEAP));

	// Set resource barriers
	ResourceBarrier barriers[2];
	auto numBarriers = m_vertexBuffer->SetBarrier(barriers, ResourceState::VERTEX_AND_CONSTANT_BUFFER);
	numBarriers = m_indexBuffer->SetBarrier(barriers, ResourceState::INDEX_BUFFER, numBarriers);
	pCommandList->Barrier(numBarriers, barriers);

	return true;
}

bool SparseVolume::buildShaderTables(const RayTracing::Device* pDevice)
{
	// Get shader identifiers.
	const auto shaderIDSize = ShaderRecord::GetShaderIdentifierSize(pDevice);
	const RayGenConstants rayGenConsts = {};

	for (uint8_t i = 0; i < FrameCount; ++i)
	{
		// Ray gen shader table
		m_rayGenShaderTables[i] = ShaderTable::MakeUnique();
		XUSG_N_RETURN(m_rayGenShaderTables[i]->Create(pDevice, 1, shaderIDSize + sizeof(RayGenConstants),
			MemoryFlag::NONE, (L"RayGenShaderTable" + to_wstring(i)).c_str()), false);
		m_rayGenShaderTables[i]->AddShaderRecord(ShaderRecord::MakeUnique(pDevice,
			m_pipelines[RAY_TRACING], RaygenShaderName, &rayGenConsts, sizeof(RayGenConstants)).get());
	}

	// Hit group shader table
	m_hitGroupShaderTable = ShaderTable::MakeUnique();
	XUSG_N_RETURN(m_hitGroupShaderTable->Create(pDevice, 1, shaderIDSize, MemoryFlag::NONE, L"HitGroupShaderTable"), false);
	m_hitGroupShaderTable->AddShaderRecord(ShaderRecord::MakeUnique(pDevice, m_pipelines[RAY_TRACING], HitGroupName).get());

	// Miss shader table
	m_missShaderTable = ShaderTable::MakeUnique();
	XUSG_N_RETURN(m_missShaderTable->Create(pDevice, 1, shaderIDSize, MemoryFlag::NONE, L"MissShaderTable"), false);
	m_missShaderTable->AddShaderRecord(ShaderRecord::MakeUnique(pDevice, m_pipelines[RAY_TRACING], MissShaderName).get());

	return true;
}

void SparseVolume::depthPeel(RayTracing::CommandList* pCommandList,
	uint8_t frameIndex, const Descriptor& dsv, bool setPipeline)
{
	// Set resource barrier
	ResourceBarrier barrier;
	m_depthKBuffer->SetBarrier(&barrier, ResourceState::UNORDERED_ACCESS); // Auto promotion

	// Set descriptor tables
	pCommandList->SetGraphicsPipelineLayout(m_pipelineLayouts[DEPTH_PEEL_LAYOUT]);
	pCommandList->SetGraphicsRootConstantBufferView(CONSTANTS, m_cbDepthPeel.get(), m_cbDepthPeel->GetCBVOffset(frameIndex));
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
		m_depthKBuffer.get(), XMVECTORU32{ reinterpret_cast<const uint32_t&>(maxDepth) }.u);

	// Record commands.
	pCommandList->IASetVertexBuffers(0, 1, &m_vertexBuffer->GetVBV());
	pCommandList->IASetIndexBuffer(m_indexBuffer->GetIBV());
	pCommandList->IASetPrimitiveTopology(PrimitiveTopology::TRIANGLELIST);
	pCommandList->DrawIndexed(m_numIndices, 1, 0, 0, 0);
}

void SparseVolume::depthPeelLightSpace(RayTracing::CommandList* pCommandList,
	uint8_t frameIndex, const Descriptor& dsv)
{
	// Set resource barrier
	ResourceBarrier barrier;
	m_lsDepthKBuffer->SetBarrier(&barrier, ResourceState::UNORDERED_ACCESS); // Auto promotion

	// Set descriptor tables
	pCommandList->SetGraphicsPipelineLayout(m_pipelineLayouts[DEPTH_PEEL_LAYOUT]);
	pCommandList->SetGraphicsRootConstantBufferView(CONSTANTS, m_cbDepthPeelLS.get(), m_cbDepthPeelLS->GetCBVOffset(frameIndex));
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
		m_lsDepthKBuffer.get(), XMVECTORU32{ reinterpret_cast<const uint32_t&>(maxDepth) }.u);

	// Record commands.
	pCommandList->IASetVertexBuffers(0, 1, &m_vertexBuffer->GetVBV());
	pCommandList->IASetIndexBuffer(m_indexBuffer->GetIBV());
	pCommandList->IASetPrimitiveTopology(PrimitiveTopology::TRIANGLELIST);
	pCommandList->DrawIndexed(m_numIndices, 1, 0, 0, 0);
}

void SparseVolume::render(RayTracing::CommandList* pCommandList, uint8_t frameIndex, const Descriptor& rtv)
{
	// Set resource barriers
	ResourceBarrier barriers[2];
	auto numBarriers = m_depthKBuffer->SetBarrier(barriers, ResourceState::PIXEL_SHADER_RESOURCE);
	numBarriers = m_lsDepthKBuffer->SetBarrier(barriers, ResourceState::PIXEL_SHADER_RESOURCE, numBarriers);
	pCommandList->Barrier(numBarriers, barriers);

	// Set descriptor tables
	pCommandList->SetGraphicsPipelineLayout(m_pipelineLayouts[SPARSE_RAYCAST_LAYOUT]);
	pCommandList->SetGraphicsRootConstantBufferView(CONSTANTS, m_cbPerFrame.get(), m_cbPerFrame->GetCBVOffset(frameIndex));
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

void SparseVolume::rayTrace(RayTracing::CommandList* pCommandList, uint8_t frameIndex)
{
	// Set resource barrier
	ResourceBarrier barrier;
	const auto numBarriers = m_outputView->SetBarrier(&barrier, ResourceState::UNORDERED_ACCESS);
	pCommandList->Barrier(numBarriers, &barrier);

	// Set descriptor tables
	pCommandList->SetComputePipelineLayout(m_pipelineLayouts[GLOBAL_LAYOUT]);
	pCommandList->SetComputeDescriptorTable(OUTPUT_VIEW, m_uavTables[UAV_TABLE_OUT_VIEW]);
	pCommandList->SetTopLevelAccelerationStructure(ACCELERATION_STRUCTURE, m_topLevelAS.get());
	pCommandList->SetComputeDescriptorTable(DEPTH_K_BUFFERS, m_srvTable);

	pCommandList->ClearUnorderedAccessViewFloat(m_uavTables[UAV_TABLE_OUT_VIEW], m_outputView->GetUAV(),
		m_outputView.get(), XMVECTORF32{ 0.0f });

	// Fallback layer has no depth
	pCommandList->SetRayTracingPipeline(m_pipelines[RAY_TRACING]);
	pCommandList->DispatchRays((uint32_t)m_viewport.x, (uint32_t)m_viewport.y, 1,
		m_rayGenShaderTables[frameIndex].get(), m_hitGroupShaderTable.get(), m_missShaderTable.get());
}
