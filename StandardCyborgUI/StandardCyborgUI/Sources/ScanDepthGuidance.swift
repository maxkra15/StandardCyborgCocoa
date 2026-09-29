import Foundation

/// Uses a sparse central depth sample, not RGB or inferred anatomy.
struct ScanDepthGuidance {
    let medianMetres: Float?
    let message: String?
    var allowsStart: Bool { message == nil }

    init(samples: [Float], minimumDistance: Float, maximumDistance: Float) {
        let valid = samples.filter { $0.isFinite && $0 > 0 && $0 < 2 }.sorted()
        guard !samples.isEmpty, valid.count >= max(12, samples.count / 5) else {
            medianMetres = nil
            message = "Aim at the whole foot and some floor. Depth detail is missing."
            return
        }
        let median = valid[valid.count / 2]
        medianMetres = median
        let distance = Int((median * 100).rounded())
        if median < minimumDistance {
            message = "Move farther away · about \(distance) cm now. Aim for 30–40 cm."
        } else if median > maximumDistance {
            message = "Move closer · about \(distance) cm now. Aim for 30–40 cm."
        } else {
            message = nil
        }
    }
}
