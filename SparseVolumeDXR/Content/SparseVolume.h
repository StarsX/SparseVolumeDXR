//--------------------------------------------------------------------------------------
// By Stars XU Tianchen
//--------------------------------------------------------------------------------------

#pragma once

#include "Core/XUSG.h"
#include "RayTracing/XUSGRayTracing.h"

class SparseVolume
{
public:
	SparseVolume(const XUSG::RayTracing::Device &device);
	virtual ~SparseVolume();

	bool Init(const XUSG::RayTracing::CommandList &commandList, uint32_t width, uint32_t height,
		XUSG::Format rtFormat, XUSG::Format dsFormat, std::vector<XUSG::Resource> &uploaders,
		XUSG::RayTracing::Geometry &geometry, const char *fileName);

	void UpdateFrame(uint32_t frameIndex, DirectX::CXMMATRIX viewProj);
	void Render(const XUSG::RayTracing::CommandList &commandList, uint32_t frameIndex,
		const XUSG::RenderTargetTable &rtvs, const XUSG::Descriptor &dsv, const XUSG::Descriptor &lsDsv);
	void RenderDXR(const XUSG::RayTracing::CommandList &commandList, uint32_t frameIndex,
		XUSG::RenderTarget &dst, const XUSG::Descriptor &dsv);

	static const uint32_t FrameCount = 3;

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

		NUM_PIPELINE
	};

	enum UAVTable : uint8_t
	{
		UAV_TABLE_KBUFFER,
		UAV_TABLE_LS_KBUFFER,
		UAV_TABLE_OUT_VIEW,
		UAV_TABLE_THICKNESS,

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

	struct PerObjConstants
	{
		DirectX::XMFLOAT4X4	ScreenToWorld;
		DirectX::XMFLOAT4X4	ViewProjLS;
	};

	struct RayGenConstants
	{
		DirectX::XMFLOAT4X4	ScreenToWorld;
		DirectX::XMFLOAT4	LightDir;
	};

	bool createVB(const XUSG::RayTracing::CommandList &commandList, uint32_t numVert,
		uint32_t stride, const uint8_t *pData, std::vector<XUSG::Resource> &uploaders);
	bool createIB(const XUSG::RayTracing::CommandList &commandList, uint32_t numIndices,
		const uint32_t *pData, std::vector<XUSG::Resource> &uploaders);
	bool createInputLayout();
	bool createPipelineLayouts();
	bool createPipelines(XUSG::Format rtFormat, XUSG::Format dsFormat);
	bool createDescriptorTables();
	bool buildAccelerationStructures(const XUSG::RayTracing::CommandList &commandList,
		XUSG::RayTracing::Geometry *geometries);
	bool buildShaderTables();

	void depthPeel(const XUSG::RayTracing::CommandList &commandList, uint32_t frameIndex,
		const XUSG::Descriptor &dsv, bool setPipeline = true);
	void depthPeelLightSpace(const XUSG::RayTracing::CommandList &commandList,
		uint32_t frameIndex, const XUSG::Descriptor &dsv);
	void render(const XUSG::RayTracing::CommandList &commandList,
		uint32_t frameIndex, const XUSG::RenderTargetTable &rtvs);
	void rayTrace(const XUSG::RayTracing::CommandList &commandList, uint32_t frameIndex);

	XUSG::RayTracing::Device m_device;

	static const uint32_t NumUAVs = 2 + FrameCount * NUM_UAV_TABLE;
	XUSG::RayTracing::BottomLevelAS m_bottomLevelAS;
	XUSG::RayTracing::TopLevelAS m_topLevelAS;

	XUSG::InputLayout			m_inputLayout;
	XUSG::PipelineLayout		m_pipelineLayouts[NUM_PIPELINE_LAYOUT];
	XUSG::RayTracing::Pipeline	m_rayTracingPipeline;
	XUSG::Pipeline				m_pipelines[NUM_PIPELINE];

	XUSG::DescriptorTable		m_srvTables[FrameCount];
	XUSG::DescriptorTable		m_uavTables[NUM_UAV_TABLE][FrameCount];

	XUSG::VertexBuffer			m_vertexBuffer;
	XUSG::IndexBuffer			m_indexBuffer;

	XUSG::Texture2D				m_depthKBuffers[FrameCount];
	XUSG::Texture2D				m_lsDepthKBuffers[FrameCount];
	XUSG::Texture2D				m_outputViews[FrameCount];
	XUSG::Texture2D				m_thicknesses[FrameCount];

	XUSG::Resource				m_scratch;
	XUSG::Resource				m_instances;

	DirectX::XMFLOAT4X4			m_world;
	DirectX::XMFLOAT4X4			m_worldViewProj;
	DirectX::XMFLOAT4X4			m_worldViewProjLS;
	PerObjConstants				m_cbPerObject;

	// Shader tables
	static const wchar_t *HitGroupName;
	static const wchar_t *RaygenShaderName;
	static const wchar_t *ClosestHitShaderName;
	static const wchar_t *AnyHitShaderName;
	static const wchar_t *MissShaderName;
	XUSG::RayTracing::ShaderTable	m_missShaderTable;
	XUSG::RayTracing::ShaderTable	m_hitGroupShaderTable;
	XUSG::RayTracing::ShaderTable	m_rayGenShaderTables[FrameCount];

	XUSG::ShaderPool				m_shaderPool;
	XUSG::RayTracing::PipelineCache	m_rayTracingPipelineCache;
	XUSG::Graphics::PipelineCache	m_graphicsPipelineCache;
	XUSG::Compute::PipelineCache	m_computePipelineCache;
	XUSG::PipelineLayoutCache		m_pipelineLayoutCache;
	XUSG::DescriptorTableCache		m_descriptorTableCache;

	DirectX::XMFLOAT2				m_viewport;
	DirectX::XMFLOAT4				m_bound;
	uint32_t						m_numIndices;
};
