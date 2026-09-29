import Foundation
import simd

/// The fused model stays in its first camera frame, even after the phone moves.
struct ScanCaptureState {
    private(set) var gravity: SIMD3<Float>?
    private var trackingLostAt: TimeInterval?
    var isRecovering: Bool { trackingLostAt != nil }

    mutating func begin() {
        gravity = nil
        trackingLostAt = nil
    }

    @discardableResult
    mutating func recordInitialGravity(_ sample: SIMD3<Float>) -> Bool {
        if gravity != nil { return true }
        guard sample.x.isFinite, sample.y.isFinite, sample.z.isFinite,
              (0.5...1.5).contains(simd_length(sample)) else { return false }
        gravity = simd_normalize(sample)
        return true
    }

    mutating func trackingRecovered() { trackingLostAt = nil }

    mutating func shouldStopForTrackingLoss(now: TimeInterval, grace: TimeInterval) -> Bool {
        if trackingLostAt == nil { trackingLostAt = now }
        return now - (trackingLostAt ?? now) >= grace
    }
}
