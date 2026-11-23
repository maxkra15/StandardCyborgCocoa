//
//  PointCloudCommandEncoder.swift
//  TrueDepthFusion
//
//  Copyright © 2018 Standard Cyborg. All rights reserved.
//

import AVFoundation
import Metal
import MetalKit
import StandardCyborgFusion
import simd

private struct SharedUniforms {
    let modelViewMatrix: simd_float4x4
    let normalMatrix: simd_float3x3
    let modelViewProjection: simd_float4x4
    let pointSize: Float
    // Match Metal's trailing padding for the 16-byte-aligned uniform block.
    let padding: (Float, Float, Float) = (0, 0, 0)
}

/** Submits Metal commands onto a command buffer to render a point cloud */
public class PointCloudCommandEncoder {
    private let _device: MTLDevice
    private let _pipelineState: MTLRenderPipelineState
    private let _depthStencilState: MTLDepthStencilState
    private var _depthTexture: MTLTexture?
    private let _matcapTexture: MTLTexture

    // MARK: - MetalVisualization

    private class func pointCloudVertexDescriptor() -> MTLVertexDescriptor {
        let descriptor = MTLVertexDescriptor()
        
        descriptor.layouts[0].stride = SCPointCloud.pointStride()
        descriptor.attributes[0].format = SCPointCloud.positionMTLVertexFormat()
        descriptor.attributes[0].offset = SCPointCloud.positionOffset()
        descriptor.attributes[1].format = SCPointCloud.normalMTLVertexFormat()
        descriptor.attributes[1].offset = SCPointCloud.normalOffset()
        descriptor.attributes[2].format = SCPointCloud.colorMTLVertexFormat()
        descriptor.attributes[2].offset = SCPointCloud.colorOffset()
        descriptor.attributes[3].format = SCPointCloud.weightMTLVertexFormat()
        descriptor.attributes[3].offset = SCPointCloud.weightOffset()
        
        return descriptor
    }
    
    /** The designated initializer. Pass the same MTLDevice instance as is used
        for the rest of your rendering and your CAMetalLayer */
    public init(device: MTLDevice, library: MTLLibrary) {
        _device = device
        
        let textureLoader = MTKTextureLoader(device: _device)
        let bundle = Bundle.scuiResourcesBundle
        /*
        guard let matcapURL = bundle.url(forResource: "matcap", withExtension: "png") else {
            fatalError("Couldn't find matcap.png in \(bundle)")
        }
        
        _matcapTexture = try! textureLoader.newTexture(URL: matcapURL, options: [MTKTextureLoader.Option.SRGB: NSNumber(booleanLiteral: false)])
         */
        _matcapTexture = try! textureLoader.newTexture(name: "matcap",
                                                       scaleFactor: 1,
                                                       bundle: bundle,
                                                       options: [MTKTextureLoader.Option.SRGB: NSNumber(booleanLiteral: false)])

        
        let vertexFunction = library.makeFunction(name: "RenderSCPointCloudVertex")
        let fragmentFunction = library.makeFunction(name: "RenderSCPointCloudFragment")
        
        let pipelineDescriptor = MTLRenderPipelineDescriptor()
        pipelineDescriptor.vertexFunction = vertexFunction
        pipelineDescriptor.fragmentFunction = fragmentFunction
        pipelineDescriptor.vertexDescriptor = PointCloudCommandEncoder.pointCloudVertexDescriptor()
        pipelineDescriptor.colorAttachments[0].pixelFormat = MTLPixelFormat.bgra8Unorm
        pipelineDescriptor.depthAttachmentPixelFormat = MTLPixelFormat.depth32Float
        pipelineDescriptor.label = "PointCloudCommandEncoder._pipelineState"
        
        let depthStencilDescriptor = MTLDepthStencilDescriptor()
        depthStencilDescriptor.depthCompareFunction = MTLCompareFunction.less
        depthStencilDescriptor.isDepthWriteEnabled = true
        depthStencilDescriptor.label = "PointCloudCommandEncoder._depthStencilState"
        _depthStencilState = _device.makeDepthStencilState(descriptor: depthStencilDescriptor)!
        
        _pipelineState = try! device.makeRenderPipelineState(descriptor: pipelineDescriptor)
    }

    public func encodeCommands(onto commandBuffer: MTLCommandBuffer,
                               pointCloud: SCPointCloud,
                               depthCameraCalibrationData: AVCameraCalibrationData,
                               viewMatrix: simd_float4x4,
                               pointSize: Float,
                               flipsInputHorizontally: Bool = false,
                               outputTexture: MTLTexture,
                               colorSize: SIMD2<Float>? = nil,
                               focalLengthScale: Float = 1.019)
    {
        guard pointCloud.pointCount > 0 else { return }
        
        if _depthTexture?.width != outputTexture.width || _depthTexture?.height != outputTexture.height {
            let descriptor = MTLTextureDescriptor.texture2DDescriptor(pixelFormat: MTLPixelFormat.depth32Float,
                                                                      width: outputTexture.width,
                                                                      height: outputTexture.height,
                                                                      mipmapped: false)
            descriptor.usage = MTLTextureUsage.renderTarget
            descriptor.storageMode = MTLStorageMode.private
            _depthTexture = _device.makeTexture(descriptor: descriptor)
            _depthTexture?.label = "PointCloudCommandEncoder._depthTexture"
        }
        guard let depthTexture = _depthTexture else { return }
        
        let pointsBuffer = pointCloud.buildPointsMTLBuffer(with: _device)
        pointsBuffer.label = "PointCloudCommandEncoder.pointsBuffer"
        
        let referenceSize = depthCameraCalibrationData.intrinsicMatrixReferenceDimensions
        var intrinsics = depthCameraCalibrationData.intrinsicMatrix
        intrinsics.columns.0.x *= focalLengthScale
        intrinsics.columns.1.y *= focalLengthScale
        let (modelView, projection) = ScanPreviewProjection.matrices(
            intrinsicMatrix: intrinsics, extrinsicMatrix: depthCameraCalibrationData.extrinsicMatrix,
            referenceWidth: Float(referenceSize.width), referenceHeight: Float(referenceSize.height),
            outputWidth: Float(outputTexture.width), outputHeight: Float(outputTexture.height),
            viewMatrix: viewMatrix, mirror: flipsInputHorizontally, colorSize: colorSize
        )
        let viewRotation = simd_float3x3(columns: (
            SIMD3(modelView.columns.0.x, modelView.columns.0.y, modelView.columns.0.z),
            SIMD3(modelView.columns.1.x, modelView.columns.1.y, modelView.columns.1.z),
            SIMD3(modelView.columns.2.x, modelView.columns.2.y, modelView.columns.2.z)
        ))
        var uniforms = SharedUniforms(modelViewMatrix: modelView,
                                      normalMatrix: viewRotation.inverse.transpose,
                                      modelViewProjection: projection * modelView,
                                      pointSize: pointSize)

        let passDescriptor = MTLRenderPassDescriptor()
        passDescriptor.colorAttachments[0].texture = outputTexture
        passDescriptor.colorAttachments[0].storeAction = MTLStoreAction.store
        passDescriptor.colorAttachments[0].loadAction = MTLLoadAction.load
        passDescriptor.depthAttachment.texture = depthTexture
        passDescriptor.depthAttachment.loadAction = MTLLoadAction.clear
        passDescriptor.depthAttachment.storeAction = MTLStoreAction.dontCare
        passDescriptor.depthAttachment.clearDepth = 1
        
        let commandEncoder = commandBuffer.makeRenderCommandEncoder(descriptor: passDescriptor)!
        commandEncoder.label = "PointCloudCommandEncoder.commandEncoder"
        
        commandEncoder.setRenderPipelineState(_pipelineState)
        commandEncoder.setViewport(MTLViewport(originX: 0, originY: 0,
                                               width: Double(outputTexture.width),
                                               height: Double(outputTexture.height),
                                               znear: 0, zfar: 1))
        commandEncoder.setDepthStencilState(_depthStencilState)
        commandEncoder.setFrontFacing(MTLWinding.counterClockwise)
        commandEncoder.setCullMode(MTLCullMode.back)
        commandEncoder.setVertexTexture(_matcapTexture, index: 0)
        commandEncoder.setVertexBuffer(pointsBuffer, offset: 0, index: 0)
        commandEncoder.setVertexBytes(&uniforms, length: MemoryLayout<SharedUniforms>.size, index: 1)
        commandEncoder.drawPrimitives(type: MTLPrimitiveType.point, vertexStart: 0, vertexCount: pointCloud.pointCount)
        
        commandEncoder.endEncoding()
    }
    
}
