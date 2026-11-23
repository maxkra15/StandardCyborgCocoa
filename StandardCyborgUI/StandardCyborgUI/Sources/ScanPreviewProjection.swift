import simd
import StandardCyborgFusion

/// Geometry shared by the portrait TrueDepth image and depth overlay.
public enum ScanPreviewProjection {
    public static func aspectFillScale(sourceWidth: Float, sourceHeight: Float,
                                       outputWidth: Float, outputHeight: Float) -> SIMD2<Float> {
        let portraitAspect = sourceHeight / sourceWidth
        let outputAspect = outputWidth / outputHeight
        return SIMD2(max(1, portraitAspect / outputAspect), max(1, outputAspect / portraitAspect))
    }

    public static func colorSamplingTransform(sourceWidth: Float, sourceHeight: Float,
                                              outputWidth: Float, outputHeight: Float) -> simd_float3x3 {
        let scale = aspectFillScale(sourceWidth: sourceWidth, sourceHeight: sourceHeight,
                                   outputWidth: outputWidth, outputHeight: outputHeight)
        // Invert the centered aspect fill and the quarter turn when looking up each output pixel.
        return simd_float3x3(columns: (
            SIMD3(0, 1 / (scale.x * outputWidth), 0),
            SIMD3(-1 / (scale.y * outputHeight), 0, 0),
            SIMD3(0.5 * (1 + 1 / scale.y), 0.5 * (1 - 1 / scale.x), 1)
        ))
    }

    public static func matrices(intrinsicMatrix: simd_float3x3, extrinsicMatrix: simd_float4x3,
                                referenceWidth: Float, referenceHeight: Float,
                                outputWidth: Float, outputHeight: Float,
                                viewMatrix: simd_float4x4, mirror: Bool,
                                colorSize: SIMD2<Float>? = nil)
        -> (modelView: simd_float4x4, projection: simd_float4x4) {
        let sourceSize = colorSize ?? SIMD2(referenceWidth, referenceHeight)
        let scale = aspectFillScale(sourceWidth: sourceSize.x, sourceHeight: sourceSize.y,
                                   outputWidth: outputWidth, outputHeight: outputHeight)
        // The mirror bracket reverses depth rows before fusion. Reflect the projected dots
        // horizontally to register them with the raw reflection displayed in the camera feed.
        let horizontalScale = mirror ? -scale.x : scale.x

        let near: Float = 0.001
        let far: Float = 10
        let fx = 2 * intrinsicMatrix.columns.0.x / referenceWidth
        let fy = 2 * intrinsicMatrix.columns.1.y / referenceHeight
        let skew = 2 * intrinsicMatrix.columns.1.x / referenceWidth
        let offsetX = 1 - 2 * intrinsicMatrix.columns.2.y / referenceHeight
        let offsetY = 1 - 2 * intrinsicMatrix.columns.2.x / referenceWidth
        let projection = simd_float4x4(columns: (
            SIMD4(fy * horizontalScale, -skew * scale.y, 0, 0),
            SIMD4(0, fx * scale.y, 0, 0),
            SIMD4(-offsetX * horizontalScale, offsetY * scale.y, far / (near - far), -1),
            SIMD4(0, 0, far * near / (near - far), 0)
        ))

        let orientation = SCReconstructionCameraOrientationMatrix()
        let calibrationView = simd_float4x4(columns: (
            SIMD4(extrinsicMatrix.columns.0, 0),
            SIMD4(extrinsicMatrix.columns.1, 0),
            SIMD4(extrinsicMatrix.columns.2, 0),
            SIMD4(extrinsicMatrix.columns.3, 1)
        )) * simd_float4x4(columns: (
            SIMD4(orientation.columns.0, 0),
            SIMD4(orientation.columns.1, 0),
            SIMD4(orientation.columns.2, 0),
            SIMD4(0, 0, 0, 1)
        ))
        // Undo fusion's camera basis change, then rotate into the same portrait coordinates as RGB.
        let portraitRotation = simd_float4x4(columns: (
            SIMD4(0, -1, 0, 0), SIMD4(1, 0, 0, 0),
            SIMD4(0, 0, 1, 0), SIMD4(0, 0, 0, 1)
        ))
        return (portraitRotation * calibrationView * viewMatrix.inverse, projection)
    }
}
