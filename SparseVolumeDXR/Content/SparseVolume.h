//--------------------------------------------------------------------------------------
// Copyright (c) XU, Tianchen. All rights reserved.
//--------------------------------------------------------------------------------------

#pragma once

#include "RayTracing/XUSGRayTracing.h"

class SparseVolume
{
public:
	SparseVolume();
	virtual ~SparseVolume();

	bool Init(XUSG::RayTracing::CommandList* pCommandList, const XUSG::DescriptorTableLib::sptr& descriptorTableLib,
		uint32_t width, uint32_t height, XUSG::Format rtFormat, XUSG::Format dsFormat, std::vector<XUSG::Resource::uptr>& uploaders,
		XUSG::RayTracing::GeometryBuffer* pGeometry, const char* fileName, const DirectX::XMFLOAT4& posScale);

	void UpdateFrame(const XUSG::RayTracing::Device* pDevice, uint8_t frameIndex, DirectX::CXMMATRIX viewProj);
	void Render(XUSG::RayTracing::CommandList* pCommandList, uint8_t frameIndex,
		const XUSG::Descriptor& rtv, const XUSG::Descriptor& dsv, const XUSG::Descriptor& lsDsv);
	void RenderDXR(XUSG::RayTracing::CommandList* pCommandList, uint8_t frameIndex,
		XUSG::RenderTarget* pDst, const XUSG::Descriptor& dsv);

	static const uint8_t FrameCount = 3;

protected:
	enum PipelineLayoutIndex : uint8_t
	{
		DEPTH_PEEL_LAYOUT,
		SPARSE_RAYCAST_LAYOUT,
		GLOBAL_LAYOUT,
		RAY_GEN_LAYOUT,

		NUM_PIPELINE_LAYOUT
	};

	enum PipelineLayoutSlot : uint8_t
	{
		CONSTANTS,
		SRV_UAVS
	};

	enum GlobalPipelineLayoutSlot : uint8_t
	{
		OUTPUT_VIEW,
		ACCELERATION_STRUCTURE,
		DEPTH_K_BUFFERS
	};

	enum PipelineIndex : uint8_t
	{
		DEPTH_PEEL,
		SPARSE_RAYCAST,
		RAY_TRACING,

		NUM_PIPELINE
	};

	enum UAVTable : uint8_t
	{
		UAV_TABLE_KBUFFER,
		UAV_TABLE_LS_KBUFFER,
		UAV_TABLE_OUT_VIEW,

		NUM_UAV_TABLE
	};

	enum VertexShaderID : uint8_t
	{
		VS_BASE_PASS,
		VS_SCREEN_QUAD
	};

	enum PixelShaderID : uint8_t
	{
		PS_DEPTH_PEEL,
		PS_SPARSE_RAYCAST
	};

	bool createVB(XUSG::CommandList* pCommandList, uint32_t numVert,
		uint32_t stride, const uint8_t* pData, std::vector<XUSG::Resource::uptr>& uploaders);
	bool createIB(XUSG::CommandList* pCommandList, uint32_t numIndices,
		const uint32_t* pData, std::vector<XUSG::Resource::uptr>& uploaders);
	bool createInputLayout();
	bool createPipelineLayouts(const XUSG::RayTracing::Device* pDevice);
	bool createPipelines(XUSG::Format rtFormat, XUSG::Format dsFormat);
	bool createDescriptorTables();
	bool buildAccelerationStructures(XUSG::RayTracing::CommandList* pCommandList,
		XUSG::RayTracing::GeometryBuffer* pGeometry);
	bool buildShaderTables(const XUSG::RayTracing::Device* pDevice);

	void depthPeel(XUSG::RayTracing::CommandList* pCommandList,
		uint8_t frameIndex, const XUSG::Descriptor& dsv, bool setPipeline = true);
	void depthPeelLightSpace(XUSG::RayTracing::CommandList* pCommandList,
		uint8_t frameIndex, const XUSG::Descriptor& dsv);
	void render(XUSG::RayTracing::CommandList* pCommandList, uint8_t frameIndex, const XUSG::Descriptor& rtv);
	void rayTrace(XUSG::RayTracing::CommandList* pCommandList, uint8_t frameIndex);

	XUSG::RayTracing::BottomLevelAS::uptr m_bottomLevelAS;
	XUSG::RayTracing::TopLevelAS::uptr m_topLevelAS;

	const XUSG::InputLayout*	m_pInputLayout;
	XUSG::PipelineLayout		m_pipelineLayouts[NUM_PIPELINE_LAYOUT];
	XUSG::Pipeline				m_pipelines[NUM_PIPELINE];

	XUSG::DescriptorTable		m_srvTable;
	XUSG::DescriptorTable		m_uavTables[NUM_UAV_TABLE];

	XUSG::VertexBuffer::uptr	m_vertexBuffer;
	XUSG::IndexBuffer::uptr		m_indexBuffer;

	XUSG::Texture2D::uptr		m_depthKBuffer;
	XUSG::Texture2D::uptr		m_lsDepthKBuffer;
	XUSG::Texture2D::uptr		m_outputView;

	XUSG::ConstantBuffer::uptr	m_cbDepthPeel;
	XUSG::ConstantBuffer::uptr	m_cbDepthPeelLS;
	XUSG::ConstantBuffer::uptr	m_cbPerFrame;

	XUSG::Resource::uptr		m_scratch;
	XUSG::Resource::uptr		m_instances;

	DirectX::XMFLOAT3X4			m_world;

	// Shader tables
	static const wchar_t* HitGroupName;
	static const wchar_t* RaygenShaderName;
	static const wchar_t* AnyHitShaderName;
	static const wchar_t* MissShaderName;
	XUSG::RayTracing::ShaderTable::uptr	m_missShaderTable;
	XUSG::RayTracing::ShaderTable::uptr	m_hitGroupShaderTable;
	XUSG::RayTracing::ShaderTable::uptr	m_rayGenShaderTables[FrameCount];

	XUSG::ShaderLib::uptr				m_shaderLib;
	XUSG::RayTracing::PipelineLib::uptr	m_rayTracingPipelineLib;
	XUSG::Graphics::PipelineLib::uptr	m_graphicsPipelineLib;
	XUSG::Compute::PipelineLib::uptr	m_computePipelineLib;
	XUSG::PipelineLayoutLib::uptr		m_pipelineLayoutLib;
	XUSG::DescriptorTableLib::sptr		m_descriptorTableLib;

	DirectX::XMFLOAT2	m_viewport;
	DirectX::XMFLOAT4	m_bound;
	DirectX::XMFLOAT4	m_posScale;
	uint32_t			m_numIndices;

	bool				m_useRayTracing;
};
