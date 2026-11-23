import Foundation

/// Uses a sparse central depth sample, not RGB or inferred anatomy.
struct ScanDepthGuidance {
    let medianMetres: Float?
    let message: String?
    let interruptionMessage: String?
    var allowsStart: Bool { message == nil }
    var allowsTracking: Bool { interruptionMessage == nil }

    init(samples: [Float], minimumDistance: Float, maximumDistance: Float, minimumTrackingDistance: Float = 0) {
        let valid = samples.filter { $0.isFinite && $0 > 0 && $0 < 2 }.sorted()
        guard !samples.isEmpty, valid.count >= max(12, samples.count / 5) else {
            medianMetres = nil
            message = "Keep your whole foot and some floor in view"
            interruptionMessage = "Depth detail did not recover. Keep the whole foot and some floor visible at about 30–40 cm, then try again."
            return
        }
        let median = valid[valid.count / 2]
        medianMetres = median
        if median < minimumDistance {
            message = "Move back so your heel and toes fit"
        } else if median > maximumDistance {
            message = "Move a little closer"
        } else {
            message = nil
        }
        // An established model can keep tracking closer views. Keep the stricter
        // starting distance and show the framing guidance during both phases.
        let trackingMinimum = minimumTrackingDistance > 0
            ? min(minimumTrackingDistance, minimumDistance)
            : minimumDistance
        if median < trackingMinimum {
            interruptionMessage = "The camera stayed too close. Move back until the whole foot and some floor fit in view, then try again."
        } else if median > maximumDistance {
            interruptionMessage = "The camera stayed too far away. Move closer to about 30–40 cm, then try again."
        } else {
            interruptionMessage = nil
        }
    }
}
