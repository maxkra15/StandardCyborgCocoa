//
//  AspectFillTextureCommandEncoder.swift
//  Capture
//
//  Copyright © 2018 Standard Cyborg. All rights reserved.
//

import CoreVideo
import Foundation
import Metal
import simd
import StandardCyborgFusion

public class AspectFillTextureCommandEncoder {
    
    private struct Uniforms {
        var transform = simd_float3x3(0)
        var alpha: Float = 0.3
        var __memoryPadding = simd_float4(repeating: 0)
    }
    
    private let _pipelineState: MTLComputePipelineState
    private let _textureCache: CVMetalTextureCache
    private var _uniforms = Uniforms()
    
    public init(device: MTLDevice, library: MTLLibrary) {
        _pipelineState = AspectFillTextureCommandEncoder._buildColorPipelineState(withDevice: device, library: library)
        
        var cache: CVMetalTextureCache?
        CVMetalTextureCacheCreate(kCFAllocatorDefault, nil, device, nil, &cache)
        _textureCache = cache!
    }
    
    public var alpha: Float = 0.3
    
    public func encodeCommands(onto commandBuffer: MTLCommandBuffer,
                               colorBuffer: CVPixelBuffer,
                               outputTexture: MTLTexture)
    {
        guard let colorTexture = _metalTexture(fromColorBuffer: colorBuffer)
            else { return }
        
        _uniforms.alpha = alpha
        _uniforms.transform = ScanPreviewProjection.colorSamplingTransform(
            sourceWidth: Float(colorTexture.width),
            sourceHeight: Float(colorTexture.height),
            outputWidth: Float(outputTexture.width),
            outputHeight: Float(outputTexture.height))
        
        _encodeColorRenderCommands(onto: commandBuffer,
                                   colorTexture: colorTexture,
                                   outputTexture: outputTexture)
    }
    
    // MARK: - Private
    
    private func _encodeColorRenderCommands(onto commandBuffer: MTLCommandBuffer,
                                            colorTexture: MTLTexture,
                                            outputTexture: MTLTexture)
    {
        let resultWidth = outputTexture.width
        let resultHeight = outputTexture.height
        let threadgroupCounts = MTLSize(width: 8, height: 8, depth: 1)
        let threadgroups = MTLSize(width:  resultWidth  / threadgroupCounts.width  + (resultWidth  % threadgroupCounts.width  == 0 ? 0 : 1),
                                   height: resultHeight / threadgroupCounts.height + (resultHeight % threadgroupCounts.height == 0 ? 0 : 1),
                                   depth: 1)
        
        
        let commandEncoder = commandBuffer.makeComputeCommandEncoder()!
        commandEncoder.label = "DrawColorTexture.commandEncoder"
        commandEncoder.setComputePipelineState(_pipelineState)
        commandEncoder.setBytes(&_uniforms, length: MemoryLayout<Uniforms>.size, index: 0)
        commandEncoder.setTexture(colorTexture, index: 0)
        commandEncoder.setTexture(outputTexture, index: 1)
        commandEncoder.dispatchThreadgroups(threadgroups, threadsPerThreadgroup: threadgroupCounts)
        commandEncoder.endEncoding()
    }
    
    private func _metalTexture(fromColorBuffer colorBuffer: CVPixelBuffer) -> MTLTexture? {
        let textureAttributes = [kCVPixelBufferMetalCompatibilityKey: NSNumber(booleanLiteral: true),
                                 kCVMetalTextureUsage: NSNumber(integerLiteral: Int(MTLTextureUsage.shaderRead.rawValue))]
        
        var texture: CVMetalTexture?
        CVMetalTextureCacheCreateTextureFromImage(kCFAllocatorDefault,
                                                  _textureCache,
                                                  colorBuffer,
                                                  textureAttributes as CFDictionary,
                                                  MTLPixelFormat.bgra8Unorm,
                                                  CVPixelBufferGetWidthOfPlane(colorBuffer, 0),
                                                  CVPixelBufferGetHeightOfPlane(colorBuffer, 0),
                                                  0,
                                                  &texture)
        
        return texture == nil ? nil : CVMetalTextureGetTexture(texture!)
    }
    
    private class func _buildColorPipelineState(withDevice device: MTLDevice, library: MTLLibrary) -> MTLComputePipelineState {
        let function = library.makeFunction(name: "DrawColorTexture")!
        
        let pipelineDescriptor = MTLComputePipelineDescriptor()
        pipelineDescriptor.computeFunction = function
        pipelineDescriptor.label = "DrawColorTexture._depthColorPipelineState"
        
        let pipelineState = try! device.makeComputePipelineState(function: function)
        
        return pipelineState
    }
    
}
