import Foundation
import simd

enum ScanState: Equatable {
    case ready
    case selectingScanRegion
    case scanning
    case processing
    case reviewing
    case finished
    case failed(reason: String)

    var title: String {
        switch self {
        case .ready:
            "Ready"
        case .selectingScanRegion:
            "Selecting scan region"
        case .scanning:
            "Scanning"
        case .processing:
            "Processing"
        case .reviewing:
            "Reviewing"
        case .finished:
            "Finished"
        case .failed:
            "Failed"
        }
    }

    var failureReason: String? {
        guard case let .failed(reason) = self else { return nil }
        return reason
    }
}

enum ScanEvent: Equatable {
    case start
    case regionSelected
    case stop
    case processingCompleted
    case reviewCompleted
    case fail(reason: String)
    case retry
    case cancel
}

struct ScanStateMachine {
    private(set) var state: ScanState = .ready

    @discardableResult
    mutating func send(_ event: ScanEvent) -> Bool {
        let nextState: ScanState?

        switch (state, event) {
        case (.ready, .start), (.finished, .start):
            nextState = .selectingScanRegion
        case (.selectingScanRegion, .regionSelected):
            nextState = .scanning
        case (.scanning, .stop):
            nextState = .processing
        case (.processing, .processingCompleted):
            nextState = .reviewing
        case (.reviewing, .reviewCompleted):
            nextState = .finished
        case (.failed(_), .retry), (_, .cancel):
            nextState = .ready
        case (.ready, .fail(_)),
             (.selectingScanRegion, .fail(_)),
             (.scanning, .fail(_)),
             (.processing, .fail(_)),
             (.reviewing, .fail(_)):
            guard case let .fail(reason) = event else { return false }
            nextState = .failed(reason: reason)
        default:
            nextState = nil
        }

        guard let nextState else { return false }
        state = nextState
        return true
    }
}

struct ScanCoverageTracker {
    let sectorCount: Int
    private(set) var visitedSectors: Set<Int> = []

    init(sectorCount: Int = 12) {
        precondition(sectorCount > 0)
        self.sectorCount = sectorCount
    }

    var progress: Float {
        Float(visitedSectors.count) / Float(sectorCount)
    }

    var remainingSectorCount: Int {
        sectorCount - visitedSectors.count
    }

    @discardableResult
    mutating func observe(
        cameraPosition: SIMD3<Float>,
        regionCenter: SIMD3<Float>
    ) -> Bool {
        let offset = cameraPosition - regionCenter
        let horizontalDistance = simd_length(SIMD2<Float>(offset.x, offset.z))
        guard horizontalDistance >= 0.1, horizontalDistance <= 5 else { return false }

        let fullTurn = 2 * Float.pi
        let angle = atan2(offset.x, offset.z)
        let normalizedAngle = (angle + fullTurn).truncatingRemainder(dividingBy: fullTurn)
        let sector = min(
            Int(normalizedAngle / fullTurn * Float(sectorCount)),
            sectorCount - 1
        )
        return visitedSectors.insert(sector).inserted
    }

    mutating func reset() {
        visitedSectors.removeAll(keepingCapacity: true)
    }
}
