import AVFoundation
import Compression
import CoreMotion
import CryptoKit
import Darwin
import StandardCyborgFusion
import UIKit

/// Private, opt-in RGBD journal. Payloads stream to durable storage; finalizing
/// raw capture never depends on reconstruction success or a completed mesh.
final class ScanDiagnosticsRecorder {
    private let queue: DispatchQueue
    private let lock = NSLock()
    private let startedAt = ProcessInfo.processInfo.systemUptime
    private let id = UUID().uuidString
    private let createdAt = ISO8601DateFormatter().string(from: Date())
    private let configuration: [String: Any]
    private let onFailure: (String) -> Void
    private var pendingBytes = 0
    private var nextSequence = 0
    private var captureStartedAt: TimeInterval?
    private var stopped = false
    private var failure: String?
    private var finishing = false
    private var recordedFrames = 0
    private var receivedFrames = 0
    private var totalBytes = 0
    private var issues: [String] = []
    private var finalReason = "Recording has not been finalized"
    private var finalized = false
    private var firstMotion: [String: Any]?
    private var gravity: [Any]?
    private var streams: [String: FileHandle] = [:]
    private var writerFrameCount = 0
    private var writerTotalSeconds: TimeInterval = 0
    private var maximumWriterSeconds: TimeInterval = 0
    private var writerLock: Int32 = -1
    private(set) var directory: URL?
    private let maximumBytes = 4 * 1024 * 1024 * 1024
    private let maximumPendingBytes = 64 * 1024 * 1024
    private let maximumSeconds: TimeInterval = 90
    private let diskReserve: Int64 = 1024 * 1024 * 1024

    /// The injected writer queue must be serial, preserving capture/seal order.
    init(configuration: [String: Any],
         writerQueue: DispatchQueue = DispatchQueue(label: "EigenScanDiagnostics.writer", qos: .utility),
         onFailure: @escaping (String) -> Void) {
        queue = writerQueue
        self.configuration = configuration
        self.onFailure = onFailure
        do {
            let base = try FileManager.default.url(for: .applicationSupportDirectory, in: .userDomainMask,
                                                   appropriateFor: nil, create: true)
                .appendingPathComponent("EigenScanDiagnostics", isDirectory: true)
            try FileManager.default.createDirectory(at: base, withIntermediateDirectories: true,
                attributes: [.protectionKey: FileProtectionType.complete])
            var root = base.appendingPathComponent("scan-\(id).eigenscan", isDirectory: true)
            try FileManager.default.createDirectory(at: root, withIntermediateDirectories: false,
                attributes: [.protectionKey: FileProtectionType.complete])
            var values = URLResourceValues(); values.isExcludedFromBackup = true
            try root.setResourceValues(values)
            directory = root
            let lockURL = root.appendingPathComponent("recording.lock")
            guard FileManager.default.createFile(atPath: lockURL.path, contents: nil,
                attributes: [.protectionKey: FileProtectionType.complete, .posixPermissions: 0o600]) else {
                throw CocoaError(.fileWriteUnknown)
            }
            writerLock = open(lockURL.path, O_RDWR)
            guard writerLock >= 0, flock(writerLock, LOCK_EX | LOCK_NB) == 0 else { throw CocoaError(.fileLocking) }
            try FileManager.default.createDirectory(at: root.appendingPathComponent("frames"), withIntermediateDirectories: false)
            for name in ["frames", "events", "tracking"] {
                let url = root.appendingPathComponent("\(name).jsonl")
                guard FileManager.default.createFile(atPath: url.path, contents: nil,
                    attributes: [.protectionKey: FileProtectionType.complete]) else { throw CocoaError(.fileWriteUnknown) }
                streams[name] = try FileHandle(forWritingTo: url)
            }
            try writeManifest() // durable attempt metadata exists before camera startup
            try requireDiskSpace()
        } catch { fail("Could not start the private recording: \(error.localizedDescription)") }
    }

    deinit {
        for stream in streams.values { try? stream.close() }
        releaseWriterLock()
    }

    private func numbers(_ values: [Float]) -> [Any] {
        values.map { $0.isFinite ? NSNumber(value: $0) : NSNull() }
    }

    static func timestamp(_ time: CMTime) -> [String: Any] {
        ["value": String(time.value), "timescale": time.timescale,
         "flags": time.flags.rawValue, "epoch": String(time.epoch)]
    }

    private func motionData(_ motion: CMDeviceMotion?) -> [String: Any]? {
        guard let motion else { return nil }
        let g = motion.gravity, r = motion.rotationRate, a = motion.userAcceleration, q = motion.attitude.quaternion
        return ["timestamp": motion.timestamp,
                "gravity": [g.x,g.y,g.z], "rotationRate": [r.x,r.y,r.z],
                "userAcceleration": [a.x,a.y,a.z], "attitudeQuaternion": [q.x,q.y,q.z,q.w]]
    }

    private static func calibrationData(_ calibration: AVCameraCalibrationData) -> [String: Any] {
        let k = calibration.intrinsicMatrix.columns, e = calibration.extrinsicMatrix.columns
        func numbers(_ values: [Float]) -> [Any] {
            values.map { $0.isFinite ? NSNumber(value: $0) : NSNull() }
        }
        var result: [String: Any] = [
            "intrinsicColumnMajor": numbers([k.0.x,k.0.y,k.0.z,k.1.x,k.1.y,k.1.z,k.2.x,k.2.y,k.2.z]),
            "intrinsicReferenceSize": [calibration.intrinsicMatrixReferenceDimensions.width, calibration.intrinsicMatrixReferenceDimensions.height],
            "extrinsicColumnMajor": numbers([e.0.x,e.0.y,e.0.z,e.1.x,e.1.y,e.1.z,e.2.x,e.2.y,e.2.z,e.3.x,e.3.y,e.3.z]),
            "lensDistortionCenter": [calibration.lensDistortionCenter.x, calibration.lensDistortionCenter.y],
            "pixelSize": calibration.pixelSize]
        result["lensDistortionFloat32Base64"] = calibration.lensDistortionLookupTable?.base64EncodedString()
        result["inverseLensDistortionFloat32Base64"] = calibration.inverseLensDistortionLookupTable?.base64EncodedString()
        return result
    }

    /// Immutable byte ownership ends dependence on the camera's finite pixel-buffer
    /// pool before slower compression, hashing and disk writes begin.
    struct OwnedPixelBuffer {
        let data: Data
        let width: Int
        let height: Int
        let pixelFormat: OSType
        let planes: [[String: Int]]

        static func byteCount(of buffer: CVPixelBuffer) throws -> Int {
            let count = CVPixelBufferGetPlaneCount(buffer)
            var total = 0
            for index in 0..<max(1, count) {
                let height = count == 0 ? CVPixelBufferGetHeight(buffer) : CVPixelBufferGetHeightOfPlane(buffer, index)
                let row = count == 0 ? CVPixelBufferGetBytesPerRow(buffer) : CVPixelBufferGetBytesPerRowOfPlane(buffer, index)
                let (bytes, overflow) = height.multipliedReportingOverflow(by: row)
                guard height > 0, row > 0, !overflow, bytes <= 96 * 1_024 * 1_024,
                      total <= 96 * 1_024 * 1_024 - bytes else { throw CocoaError(.fileReadTooLarge) }
                total += bytes
            }
            return total
        }

        init(copying buffer: CVPixelBuffer) throws {
            let bytes = try Self.byteCount(of: buffer)
            guard CVPixelBufferLockBaseAddress(buffer, .readOnly) == kCVReturnSuccess else { throw CocoaError(.fileReadUnknown) }
            defer { CVPixelBufferUnlockBaseAddress(buffer, .readOnly) }
            let count = CVPixelBufferGetPlaneCount(buffer)
            var raw = Data(), copiedPlanes: [[String: Int]] = []
            raw.reserveCapacity(bytes)
            for index in 0..<max(1, count) {
                let width = count == 0 ? CVPixelBufferGetWidth(buffer) : CVPixelBufferGetWidthOfPlane(buffer, index)
                let height = count == 0 ? CVPixelBufferGetHeight(buffer) : CVPixelBufferGetHeightOfPlane(buffer, index)
                let row = count == 0 ? CVPixelBufferGetBytesPerRow(buffer) : CVPixelBufferGetBytesPerRowOfPlane(buffer, index)
                guard let base = count == 0 ? CVPixelBufferGetBaseAddress(buffer) : CVPixelBufferGetBaseAddressOfPlane(buffer, index) else { throw CocoaError(.fileReadUnknown) }
                let length = height * row
                copiedPlanes.append(["width": width, "height": height, "rowBytes": row, "dataOffset": raw.count, "byteCount": length])
                raw.append(base.assumingMemoryBound(to: UInt8.self), count: length)
            }
            data = raw
            width = CVPixelBufferGetWidth(buffer)
            height = CVPixelBufferGetHeight(buffer)
            pixelFormat = CVPixelBufferGetPixelFormatType(buffer)
            planes = copiedPlanes
        }
    }

    func beginCapture() {
        let now = ProcessInfo.processInfo.systemUptime
        lock.lock(); captureStartedAt = now; lock.unlock()
        recordEvent("capture_started", fields: ["hostTimeSeconds": now])
    }

    /// Retains at most the bounded pending buffer bytes; IO/compression occurs on
    /// one worker. A rejected admission stops capture rather than silently losing
    /// reconstruction inputs. Caller must not submit a rejected frame to fusion.
    func recordFrame(color: CVPixelBuffer, depth: CVPixelBuffer, calibration: AVCameraCalibrationData,
                     colorTimestamp: CMTime, depthTimestamp: CMTime, motion: CMDeviceMotion?,
                     submission: [String: Any]) -> Int? {
        recordFrame(color: color, depth: depth, calibration: Self.calibrationData(calibration),
            colorTimestamp: colorTimestamp, depthTimestamp: depthTimestamp,
            motion: motionData(motion), submission: submission)
    }

    /// Value metadata boundary also lets replay fixtures exercise the same writer
    /// without constructing an AVFoundation-owned calibration object.
    func recordFrame(color: CVPixelBuffer, depth: CVPixelBuffer, calibration: [String: Any],
                     colorTimestamp: CMTime, depthTimestamp: CMTime, motion: [String: Any]?,
                     submission: [String: Any]) -> Int? {
        let hostTime = ProcessInfo.processInfo.systemUptime
        lock.lock()
        guard !stopped, directory != nil else { lock.unlock(); return nil }
        receivedFrames += 1
        let bytes: Int
        do { bytes = try OwnedPixelBuffer.byteCount(of: color) + OwnedPixelBuffer.byteCount(of: depth) }
        catch {
            lock.unlock()
            fail("Recording could not copy a camera buffer. The captured prefix has been saved.")
            return nil
        }
        let tooLong = ProcessInfo.processInfo.systemUptime - (captureStartedAt ?? startedAt) > maximumSeconds
        guard !tooLong, bytes > 0, pendingBytes + bytes <= maximumPendingBytes,
              totalBytes + pendingBytes + bytes <= maximumBytes else {
            lock.unlock()
            fail(tooLong ? "The 90-second recording limit was reached. The captured prefix has been saved." :
                "Recording reached its storage or IO limit. The captured prefix has been saved.")
            return nil
        }
        // Reserve and enqueue under the same lock as finish. No frame can be
        // admitted after the seal, and no queued writer retains a sensor buffer.
        let copyStarted = ProcessInfo.processInfo.systemUptime
        let ownedColor: OwnedPixelBuffer, ownedDepth: OwnedPixelBuffer
        do {
            ownedColor = try OwnedPixelBuffer(copying: color)
            ownedDepth = try OwnedPixelBuffer(copying: depth)
        } catch {
            lock.unlock()
            fail("Recording could not copy a camera buffer. The captured prefix has been saved.")
            return nil
        }
        let copyFinished = ProcessInfo.processInfo.systemUptime
        let sequence = nextSequence; nextSequence += 1; pendingBytes += bytes
        queue.async { [self] in
            defer { lock.lock(); pendingBytes -= bytes; lock.unlock() }
            autoreleasepool {
                guard failureMessage == nil else { return }
                let workStarted = ProcessInfo.processInfo.systemUptime
                do {
                    if sequence % 16 == 0 { try requireDiskSpace() }
                    let colorInfo = try writeBuffer(ownedColor, name: String(format: "%06d.color", sequence))
                    let depthInfo = try writeBuffer(ownedDepth, name: String(format: "%06d.depth", sequence))
                    var frame: [String: Any] = ["sequence": sequence, "hostTimeSeconds": hostTime,
                        "colorTimestamp": Self.timestamp(colorTimestamp), "depthTimestamp": Self.timestamp(depthTimestamp),
                        "color": colorInfo, "depth": depthInfo, "calibration": calibration, "submission": submission,
                        "recordingTiming": ["copySeconds": copyFinished - copyStarted,
                            "writerQueueDelaySeconds": workStarted - copyFinished,
                            "payloadEncodingAndWriteSeconds": ProcessInfo.processInfo.systemUptime - workStarted]]
                    if let motion { frame["motion"] = motion; if firstMotion == nil { firstMotion = motion } }
                    try append(frame, stream: "frames")
                    lock.lock(); recordedFrames += 1; lock.unlock()
                    try writeManifest()
                    let duration = ProcessInfo.processInfo.systemUptime - workStarted
                    writerFrameCount += 1
                    writerTotalSeconds += duration
                    maximumWriterSeconds = max(maximumWriterSeconds, duration)
                } catch { fail("Recording stopped after a write failed: \(error.localizedDescription)") }
            }
        }
        lock.unlock()
        return sequence
    }

    func recordEvent(_ event: String, fields: [String: Any] = [:]) {
        var data = fields; data["event"] = event
        if data["hostTimeSeconds"] == nil { data["hostTimeSeconds"] = ProcessInfo.processInfo.systemUptime }
        lock.lock()
        guard !finishing else { lock.unlock(); return }
        queue.async { [self] in
            do { try append(data, stream: "events") }
            catch { fail("Recording event write failed: \(error.localizedDescription)") }
        }
        lock.unlock()
    }

    func recordCameraDrop(colorTimestamp: CMTime, depthTimestamp: CMTime, colorDropped: Bool, depthDropped: Bool,
                          details: [String: Any] = [:]) {
        let hostTime = ProcessInfo.processInfo.systemUptime
        lock.lock()
        guard !finishing else { lock.unlock(); return }
        queue.async { [self] in
            if !issues.contains("Camera reported dropped synchronized data") { issues.append("Camera reported dropped synchronized data") }
            do {
                var event: [String: Any] = ["event": "camera_drop", "hostTimeSeconds": hostTime,
                    "colorTimestamp": Self.timestamp(colorTimestamp), "depthTimestamp": Self.timestamp(depthTimestamp),
                    "colorDropped": colorDropped, "depthDropped": depthDropped]
                if !details.isEmpty { event["details"] = details }
                try append(event, stream: "events")
            } catch { fail("Camera-drop journal write failed: \(error.localizedDescription)") }
        }
        lock.unlock()
    }

    func recordTracking(_ metadata: SCAssimilatedFrameMetadata, statistics: SCReconstructionManagerStatistics) {
        let m = metadata.viewMatrix.columns
        let event: [String: Any] = ["event": "tracking", "inputSequence": metadata.inputSequence,
            "hostTimeSeconds": ProcessInfo.processInfo.systemUptime, "result": metadata.result.rawValue,
            "assimilationTime": metadata.assimilationTime, "icpIterations": metadata.icpIterationCount,
            "icpResidualAndQuality": numbers([metadata.icpCorrespondenceError, metadata.icpUnusedIterationFraction]),
            "acceptedFrames": statistics.succeededCount, "lostTrackingFrames": statistics.lostTrackingCount,
            "consecutiveLostFrames": statistics.consecutiveLostTrackingCount,
            "viewMatrixColumnMajor": numbers([m.0.x,m.0.y,m.0.z,m.0.w,m.1.x,m.1.y,m.1.z,m.1.w,m.2.x,m.2.y,m.2.z,m.2.w,m.3.x,m.3.y,m.3.z,m.3.w])]
        lock.lock()
        guard !finishing else { lock.unlock(); return }
        queue.async { [self] in
            do { try append(event, stream: "tracking") }
            catch { fail("Tracking journal write failed: \(error.localizedDescription)") }
        }
        lock.unlock()
    }

    func finish(reason: String, gravity: SIMD3<Float>?, incomplete: Bool = false, completion: @escaping (URL?) -> Void) {
        lock.lock()
        guard !finishing else { lock.unlock(); return }
        finishing = true; stopped = true
        queue.async { [self] in
            finalReason = reason
            if incomplete { issues.append(reason) }
            if let gravity { self.gravity = numbers([gravity.x,gravity.y,gravity.z]) }
            if let failure = failureMessage, !issues.contains(failure) { issues.append(failure) }
            finalized = true
            do {
                try append(["event": "capture_stopped", "reason": reason,
                    "hostTimeSeconds": ProcessInfo.processInfo.systemUptime,
                    "liveTraceComplete": false], stream: "events")
                for stream in streams.values { try stream.synchronize() }
                try writeManifest()
            } catch {
                // The previous valid manifest and JSONL prefix remain discoverable.
                // Never delete raw capture because final metadata failed to write.
                issues.append("Finalization failed: \(error.localizedDescription)")
            }
            // A closed prefix is safe to recover/export even if the final
            // manifest write failed. Kernel locks also release after a crash;
            // process IDs or a stale on-disk marker are not lifetime evidence.
            for stream in streams.values { try? stream.close() }
            streams.removeAll()
            releaseWriterLock()
            DispatchQueue.main.async { completion(self.directory) }
        }
        lock.unlock()
    }

    private func append(_ object: [String: Any], stream: String) throws {
        let data = try JSONSerialization.data(withJSONObject: object, options: [.sortedKeys]) + Data([10])
        guard let handle = streams[stream] else { throw CocoaError(.fileWriteUnknown) }
        try handle.write(contentsOf: data); try handle.synchronize()
        lock.lock(); totalBytes += data.count; lock.unlock()
    }

    private func writeBuffer(_ buffer: OwnedPixelBuffer, name: String) throws -> [String: Any] {
        guard let directory else { throw CocoaError(.fileNoSuchFile) }
        let raw = buffer.data
        let compressed: Data? = (try? (raw as NSData).compressed(using: .lzfse)).map { $0 as Data }
        let useCompressed = compressed.map { $0.count < raw.count } ?? false
        let payload = useCompressed ? compressed! : raw
        let relative = "frames/\(name).\(useCompressed ? "lzfse" : "raw")"
        try payload.write(to: directory.appendingPathComponent(relative), options: [.atomic, .completeFileProtection])
        lock.lock(); totalBytes += payload.count; lock.unlock()
        return ["file": relative, "compression": useCompressed ? "lzfse" : "none",
            "sha256": SHA256.hash(data: raw).map { String(format: "%02x", $0) }.joined(),
            "byteCount": raw.count, "storedByteCount": payload.count,
            "width": buffer.width, "height": buffer.height,
            "pixelFormat": buffer.pixelFormat, "planes": buffer.planes]
    }

    private func requireDiskSpace() throws {
        guard let directory else { throw CocoaError(.fileNoSuchFile) }
        let capacity = try directory.resourceValues(forKeys: [.volumeAvailableCapacityForImportantUsageKey]).volumeAvailableCapacityForImportantUsage
        if let capacity, capacity < diskReserve { throw CocoaError(.fileWriteOutOfSpace) }
    }

    private func writeManifest() throws {
        guard let directory else { throw CocoaError(.fileNoSuchFile) }
        lock.lock()
        let bytes = totalBytes, frames = recordedFrames, received = receivedFrames, captureStart = captureStartedAt
        lock.unlock()
        let failure = failureMessage
        var manifest: [String: Any] = ["format": "eigen-scan-diagnostics", "version": 2, "id": id,
            "createdAt": createdAt, "updatedAt": ISO8601DateFormatter().string(from: Date()),
            "processIdentifier": ProcessInfo.processInfo.processIdentifier,
            "finalized": finalized, "completeSequence": finalized && issues.isEmpty && failure == nil && frames == received,
            "completedLiveFusion": false, "liveTraceComplete": false,
            "state": !finalized ? "recording" : (issues.isEmpty && failure == nil && frames == received ? "complete" : "incomplete"),
            "reason": finalReason, "recordedFrameCount": frames, "receivedFrameCount": received,
            "totalBytes": bytes, "totalBytesMeaning": "Compressed payload bytes plus JSONL bytes; excludes manifest",
            "issues": issues, "configuration": configuration,
            "device": UIDevice.current.model, "systemVersion": UIDevice.current.systemVersion,
            "appVersion": Bundle.main.object(forInfoDictionaryKey: "CFBundleShortVersionString") as? String ?? "unknown",
            "appBuild": Bundle.main.object(forInfoDictionaryKey: "CFBundleVersion") as? String ?? "unknown",
            "captureImplementation": "eigen-rgbd-journal-2", "clockOriginHostSeconds": startedAt,
            "bufferOwnership": "Camera buffers are copied before asynchronous compression and disk writes",
            "writerTiming": ["completedFrameTasks": writerFrameCount, "totalSeconds": writerTotalSeconds,
                "maximumFrameSeconds": maximumWriterSeconds],
            "motionSampling": "Latest CMDeviceMotion at each synchronized camera pair; not a continuous raw IMU log",
            "streams": ["frames": "frames.jsonl", "events": "events.jsonl", "tracking": "tracking.jsonl"],
            "limits": ["maximumStoredBytes": maximumBytes, "maximumPendingBytes": maximumPendingBytes,
                "maximumSeconds": maximumSeconds, "minimumFreeDiskBytes": diskReserve],
            "replayLimitations": "Original synchronized buffers and admission trace are recorded. Fusion may coalesce submitted inputs. Live trace can end with pending work; replay is not guaranteed bit-exact. A non-finalized manifest is an interrupted prefix."]
        if let gravity { manifest["gravity"] = gravity }
        if let firstMotion { manifest["firstMotion"] = firstMotion }
        if let captureStart { manifest["captureStartedHostSeconds"] = captureStart }
        let data = try JSONSerialization.data(withJSONObject: manifest, options: [.sortedKeys])
        try data.write(to: directory.appendingPathComponent("manifest.json"), options: [.atomic, .completeFileProtection])
    }

    private func fail(_ message: String) {
        lock.lock()
        guard failure == nil else { lock.unlock(); return }
        failure = message; stopped = true
        lock.unlock()
        DispatchQueue.main.async { self.onFailure(message) }
    }

    private var failureMessage: String? {
        lock.lock(); defer { lock.unlock() }
        return failure
    }

    private func releaseWriterLock() {
        guard writerLock >= 0 else { return }
        flock(writerLock, LOCK_UN)
        close(writerLock)
        writerLock = -1
    }
}
