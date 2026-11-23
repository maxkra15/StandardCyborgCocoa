import Foundation
import simd

/// Capture effort, not anatomical completeness or measurement validity.
public struct ScanCaptureProgress: Sendable {
    public let acceptedFrameCount: Int
    public let acceptedCaptureSeconds: TimeInterval
    public let isRecovering: Bool
    public let guidance: String?
    public let viewpointSpanRadians: Float
    public let cameraTranslationMetres: Float
}

/// Async finalization/diagnostic work may finish after its screen has left.
struct ScanCaptureLifetime {
    private(set) var generation: UInt = 0
    mutating func invalidate() { generation &+= 1 }
    func accepts(_ token: UInt) -> Bool { generation == token }
}

/// The fused model stays in its first camera frame, even after the phone moves.
struct ScanCaptureState {
    private(set) var gravity: SIMD3<Float>?
    private var trackingLostAt: TimeInterval?
    private var depthPausedAt: TimeInterval?
    private var lastSubmittedInputSequence = -1
    private var prePauseInputSequence: Int?
    private var lastAcceptedAt: TimeInterval?
    private var firstCameraPosition: SIMD3<Float>?
    private var firstCameraDirection: SIMD3<Float>?
    private(set) var acceptedFrameCount = 0
    private(set) var acceptedCaptureSeconds: TimeInterval = 0
    private(set) var viewpointSpanRadians: Float = 0
    private(set) var cameraTranslationMetres: Float = 0
    var isDepthPaused: Bool { depthPausedAt != nil }
    var isRecovering: Bool { trackingLostAt != nil || isDepthPaused || prePauseInputSequence != nil }
    var nextTrackingInputSequence: Int { lastSubmittedInputSequence + 1 }

    mutating func begin() { self = ScanCaptureState() }

    @discardableResult
    mutating func recordInitialGravity(_ sample: SIMD3<Float>) -> Bool {
        if gravity != nil { return true }
        guard sample.x.isFinite, sample.y.isFinite, sample.z.isFinite,
              (0.5...1.5).contains(simd_length(sample)) else { return false }
        gravity = simd_normalize(sample)
        return true
    }

    mutating func recordTrackingSubmission(_ inputSequence: Int) {
        lastSubmittedInputSequence = inputSequence
    }

    func acceptsTrackingResult(_ inputSequence: Int) -> Bool {
        guard !isDepthPaused else { return false }
        guard let prePauseInputSequence else { return true }
        return inputSequence > prePauseInputSequence
    }

    mutating func trackingRecovered() {
        trackingLostAt = nil
        prePauseInputSequence = nil
    }

    /// Hold the existing model while the sensor has no usable nearby depth.
    /// This never restarts fusion with a different reference frame.
    mutating func updateDepthUsable(_ usable: Bool, now: TimeInterval, grace: TimeInterval,
                                    waitForTrackedFrame: Bool = false) -> Bool {
        guard now.isFinite else { return true }
        if usable {
            depthPausedAt = nil
            return false
        }
        lastAcceptedAt = nil
        trackingLostAt = nil
        if depthPausedAt == nil {
            depthPausedAt = now
            if waitForTrackedFrame {
                // Results submitted before losing usable depth cannot end recovery.
                prePauseInputSequence = lastSubmittedInputSequence
            }
        }
        return now - (depthPausedAt ?? now) >= grace
    }

    mutating func shouldStopForTrackingLoss(now: TimeInterval, grace: TimeInterval) -> Bool {
        guard now.isFinite else { return true }
        lastAcceptedAt = nil
        if trackingLostAt == nil { trackingLostAt = now }
        return now - (trackingLostAt ?? now) >= grace
    }

    mutating func recordAccepted(now: TimeInterval, viewMatrix: simd_float4x4) {
        guard !isDepthPaused, now.isFinite else { return }
        let camera = simd_inverse(viewMatrix)
        let position = SIMD3(camera.columns.3.x, camera.columns.3.y, camera.columns.3.z)
        let axis = SIMD3(camera.columns.2.x, camera.columns.2.y, camera.columns.2.z)
        guard position.x.isFinite, position.y.isFinite, position.z.isFinite,
              axis.x.isFinite, axis.y.isFinite, axis.z.isFinite,
              simd_length(axis) > 0.5 else { return }
        let direction = simd_normalize(axis)
        if let lastAcceptedAt, now > lastAcceptedAt, now - lastAcceptedAt <= 0.5 {
            acceptedCaptureSeconds += now - lastAcceptedAt
        }
        lastAcceptedAt = now
        acceptedFrameCount += 1
        if firstCameraPosition == nil { firstCameraPosition = position }
        if firstCameraDirection == nil { firstCameraDirection = direction }
        cameraTranslationMetres = max(cameraTranslationMetres, simd_distance(position, firstCameraPosition!))
        let cosine = min(1, max(-1, simd_dot(direction, firstCameraDirection!)))
        viewpointSpanRadians = max(viewpointSpanRadians, acos(cosine))
    }

    func progress(guidance: String?) -> ScanCaptureProgress {
        .init(acceptedFrameCount: acceptedFrameCount, acceptedCaptureSeconds: acceptedCaptureSeconds,
              isRecovering: isRecovering, guidance: guidance,
              viewpointSpanRadians: viewpointSpanRadians, cameraTranslationMetres: cameraTranslationMetres)
    }
}
