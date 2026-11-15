import CoreMedia
import Foundation

/// Camera durations are rational seconds, not an integer timescale made from seconds.
enum CaptureTiming {
    static func boundedDuration(seconds: TimeInterval, minimum: CMTime, maximum: CMTime) -> CMTime? {
        guard seconds.isFinite, seconds > 0,
              minimum.isNumeric, maximum.isNumeric,
              minimum.seconds > 0, minimum <= maximum else { return nil }
        return CMTime(seconds: min(max(seconds, minimum.seconds), maximum.seconds), preferredTimescale: 1_000_000_000)
    }
}
