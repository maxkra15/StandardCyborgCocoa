import CoreMedia
import Foundation

/// Camera durations are rational seconds, not an integer timescale made from seconds.
enum CaptureTiming {
    static func boundedDuration(seconds: TimeInterval, minimum: CMTime, maximum: CMTime) -> CMTime? {
        guard seconds.isFinite, seconds > 0,
              minimum.isNumeric, maximum.isNumeric,
              minimum.seconds > 0, minimum <= maximum else { return nil }
        // Capture formats expose exact rational limits (for example 1/30 s).
        // Reconstructing that boundary at a nanosecond timescale can round down
        // below the permitted minimum. AVFoundation rejects that value with an
        // Objective-C exception, which Swift's configuration do/catch cannot catch.
        if seconds <= minimum.seconds { return minimum }
        if seconds >= maximum.seconds { return maximum }
        let requested = CMTime(seconds: seconds, preferredTimescale: 1_000_000_000)
        guard requested.isNumeric else { return nil }
        if requested < minimum { return minimum }
        if requested > maximum { return maximum }
        return requested
    }

    /// Preserve approximately the current exposure while shortening the shutter.
    /// Work in seconds: multiplying an existing CMTime's Int32 timescale can
    /// overflow when the camera reports nanosecond precision in low light.
    static func compensatedISO(currentDuration: CMTime, targetDuration: CMTime,
                               currentISO: Float, minimumISO: Float, maximumISO: Float) -> Float? {
        guard currentDuration.isNumeric, targetDuration.isNumeric,
              targetDuration.seconds > 0, currentDuration > targetDuration,
              currentISO.isFinite, currentISO > 0,
              minimumISO.isFinite, maximumISO.isFinite,
              minimumISO > 0, minimumISO <= maximumISO else { return nil }
        let adjusted = Double(currentISO) * (currentDuration.seconds / targetDuration.seconds)
        guard adjusted.isFinite, adjusted >= Double(minimumISO), adjusted <= Double(maximumISO) else { return nil }
        return Float(adjusted)
    }
}
