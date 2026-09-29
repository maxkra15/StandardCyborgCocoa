import AVFoundation
import StandardCyborgFusion
import UIKit

/// Opt-in, bounded depth-only capture. The caller controls sharing; nothing is uploaded.
final class ScanDiagnosticsRecorder {
    private let startedAt = ProcessInfo.processInfo.systemUptime
    private var lastSampleTime = -Double.infinity
    private var frames: [[String: Any]] = []
    private var events: [[String: Any]] = []

    private func numbers(_ values: [Float]) -> [Any] {
        values.map { $0.isFinite ? NSNumber(value: $0) : NSNull() }
    }

    func recordDepth(_ buffer: CVPixelBuffer, calibration: AVCameraCalibrationData) {
        let now = ProcessInfo.processInfo.systemUptime
        guard now - lastSampleTime >= 0.5,
              CVPixelBufferGetPixelFormatType(buffer) == kCVPixelFormatType_DepthFloat32 else { return }
        lastSampleTime = now
        CVPixelBufferLockBaseAddress(buffer, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(buffer, .readOnly) }
        guard let base = CVPixelBufferGetBaseAddress(buffer) else { return }
        let width = CVPixelBufferGetWidth(buffer), height = CVPixelBufferGetHeight(buffer)
        let stride = CVPixelBufferGetBytesPerRow(buffer)
        var depth = Data(capacity: width * height * 4)
        for row in 0..<height { depth.append(base.advanced(by: row * stride).assumingMemoryBound(to: UInt8.self), count: width * 4) }
        let k = calibration.intrinsicMatrix.columns
        let e = calibration.extrinsicMatrix.columns
        var frame: [String: Any] = [
            "seconds": now - startedAt, "width": width, "height": height,
            "depthFloat32LittleEndianBase64": depth.base64EncodedString(),
            "intrinsicColumnMajor": numbers([k.0.x,k.0.y,k.0.z,k.1.x,k.1.y,k.1.z,k.2.x,k.2.y,k.2.z]),
            "intrinsicReferenceSize": [calibration.intrinsicMatrixReferenceDimensions.width, calibration.intrinsicMatrixReferenceDimensions.height],
            "extrinsicColumnMajor": numbers([e.0.x,e.0.y,e.0.z,e.1.x,e.1.y,e.1.z,e.2.x,e.2.y,e.2.z,e.3.x,e.3.y,e.3.z]),
            "lensDistortionCenter": [calibration.lensDistortionCenter.x, calibration.lensDistortionCenter.y]
        ]
        frame["lensDistortionFloat32Base64"] = calibration.lensDistortionLookupTable?.base64EncodedString()
        frame["inverseLensDistortionFloat32Base64"] = calibration.inverseLensDistortionLookupTable?.base64EncodedString()
        // Keep the first two observations plus the ten most recent observations.
        if frames.count == 12 { frames.remove(at: 2) }
        frames.append(frame)
    }

    func recordTracking(_ metadata: SCAssimilatedFrameMetadata, statistics: SCReconstructionManagerStatistics) {
        let m = metadata.viewMatrix.columns
        if events.count == 300 { events.remove(at: 0) }
        events.append([
            "seconds": ProcessInfo.processInfo.systemUptime - startedAt,
            "result": metadata.result.rawValue,
            "icpIterations": metadata.icpIterationCount,
            "icpResidualAndQuality": numbers([metadata.icpCorrespondenceError, metadata.icpUnusedIterationFraction]),
            "acceptedFrames": statistics.succeededCount,
            "lostTrackingFrames": statistics.lostTrackingCount,
            "consecutiveLostFrames": statistics.consecutiveLostTrackingCount,
            "viewMatrixColumnMajor": numbers([m.0.x,m.0.y,m.0.z,m.0.w,m.1.x,m.1.y,m.1.z,m.1.w,m.2.x,m.2.y,m.2.z,m.2.w,m.3.x,m.3.y,m.3.z,m.3.w])
        ])
    }

    func write(cloud: SCPointCloud, reason: String, maximumDepth: Float, iterationLimit: Int = 0, completion: @escaping (URL?) -> Void) {
        let gravity = cloud.gravity
        var report: [String: Any] = [
            "format": "eigen-scan-diagnostics", "version": 1,
            "createdAt": ISO8601DateFormatter().string(from: Date()),
            "device": UIDevice.current.model, "systemVersion": UIDevice.current.systemVersion,
            "reason": reason, "maximumDepthMetres": maximumDepth,
            "icpIterationLimit": iterationLimit, "captureImplementation": "eigen-distance-guidance-1",
            "gravity": numbers([gravity.x, gravity.y, gravity.z]),
            "depthFrames": frames, "trackingEvents": events,
            "pointCount": cloud.pointCount,
            "pointStride": SCPointCloud.pointStride(),
            "positionOffset": SCPointCloud.positionOffset(), "normalOffset": SCPointCloud.normalOffset(),
            "pointEncoding": "Interleaved little-endian float32 surfels; offsets and stride are bytes. Coordinates are metres in the initial camera frame.",
            "purpose": "Diagnostic data only. Partial geometry is not a validated foot model. Depth frames are sampled, not a complete replay sequence. No RGB images included."
        ]
        #if DEBUG
        report["buildConfiguration"] = "Debug"
        #else
        report["buildConfiguration"] = "Release"
        #endif
        report["surfelsBase64"] = cloud.pointsData?.base64EncodedString()
        DispatchQueue.global(qos: .utility).async {
            do {
                let directory = FileManager.default.temporaryDirectory.appendingPathComponent("EigenScanDiagnostics", isDirectory: true)
                try FileManager.default.createDirectory(at: directory, withIntermediateDirectories: true)
                // Limit retention without deleting a report another share sheet may be using.
                for file in (try? FileManager.default.contentsOfDirectory(at: directory, includingPropertiesForKeys: [.creationDateKey])) ?? [] {
                    if let created = try? file.resourceValues(forKeys: [.creationDateKey]).creationDate,
                       created.timeIntervalSinceNow < -86400 { try? FileManager.default.removeItem(at: file) }
                }
                let url = directory.appendingPathComponent("scan-\(UUID().uuidString).eigenscan.json")
                try JSONSerialization.data(withJSONObject: report, options: [.sortedKeys]).write(to: url, options: .atomic)
                DispatchQueue.main.async { completion(url) }
            } catch {
                DispatchQueue.main.async { completion(nil) }
            }
        }
    }
}
