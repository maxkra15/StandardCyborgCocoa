//
//  DefaultScanningViewRenderer.swift
//  StandardCyborgUI
//
//  Copyright © 2019 Standard Cyborg. All rights reserved.
//

import AVFoundation
import Foundation
import Metal
import StandardCyborgFusion

public class DefaultScanningViewRenderer: ScanningViewRenderer {
    
    private let _commandQueue: MTLCommandQueue
    private let _drawTextureCommandEncoder: AspectFillTextureCommandEncoder
    private let _pointCloudRenderer: PointCloudCommandEncoder
    
    public var flipsInputHorizontally: Bool = false
    var cameraFocalLengthScaleFactor: Float = 1.019
    
    required public init(device: MTLDevice, commandQueue: MTLCommandQueue) {
        _commandQueue = commandQueue
        
        let library = try! device.makeDefaultLibrary(bundle: Bundle.scuiResourcesBundle)
        
        _drawTextureCommandEncoder = AspectFillTextureCommandEncoder(device: device, library: library)
        _pointCloudRenderer = PointCloudCommandEncoder(device: device, library: library)
    }
    
    public func draw(colorBuffer: CVPixelBuffer?,
                     pointCloud: SCPointCloud?,
                     depthCameraCalibrationData: AVCameraCalibrationData,
                     viewMatrix: matrix_float4x4,
                     into metalLayer: CAMetalLayer)
    {
        autoreleasepool {
            let commandBuffer = _commandQueue.makeCommandBuffer()!
            commandBuffer.label = "ScanningViewRenderer.commandBuffer"
            
            guard let drawable = metalLayer.nextDrawable() else { return }
            let outputTexture = drawable.texture
            
            if let colorBuffer = colorBuffer {
                _drawTextureCommandEncoder.encodeCommands(onto: commandBuffer,
                                                          colorBuffer: colorBuffer,
                                                          outputTexture: outputTexture)
            }
            
            if let pointCloud = pointCloud {
                let colorSize = colorBuffer.map {
                    SIMD2(Float(CVPixelBufferGetWidth($0)), Float(CVPixelBufferGetHeight($0)))
                }
                _pointCloudRenderer.encodeCommands(onto: commandBuffer,
                                                   pointCloud: pointCloud,
                                                   depthCameraCalibrationData: depthCameraCalibrationData,
                                                   viewMatrix: viewMatrix,
                                                   pointSize: 16,
                                                   flipsInputHorizontally: flipsInputHorizontally,
                                                   outputTexture: outputTexture,
                                                   colorSize: colorSize,
                                                   focalLengthScale: cameraFocalLengthScaleFactor)
            }
            
            commandBuffer.present(drawable)
            commandBuffer.commit()
            commandBuffer.waitUntilScheduled()
        }
    }
}
