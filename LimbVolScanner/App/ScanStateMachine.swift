import Foundation

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
    case beginRegionSelection
    case beginScanning
    case beginProcessing
    case beginReviewing
    case finish
    case fail(reason: String)
    case reset
}

struct ScanStateMachine {
    private(set) var state: ScanState = .ready

    @discardableResult
    mutating func send(_ event: ScanEvent) -> Bool {
        let nextState: ScanState?

        switch (state, event) {
        case (.ready, .beginRegionSelection):
            nextState = .selectingScanRegion
        case (.selectingScanRegion, .beginScanning):
            nextState = .scanning
        case (.scanning, .beginProcessing):
            nextState = .processing
        case (.processing, .beginReviewing):
            nextState = .reviewing
        case (.reviewing, .finish):
            nextState = .finished
        case (_, .reset):
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
