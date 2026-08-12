import Foundation
import simd

enum ScanState: Equatable {
    case ready
    case selectingObject
    case scanning
    case processing
    case reviewing
    case failed(reason: String)

    var title: String {
        switch self {
        case .ready:
            "Ready"
        case .selectingObject:
            "Selecting object"
        case .scanning:
            "Scanning"
        case .processing:
            "Processing"
        case .reviewing:
            "Reviewing"
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
        case (.ready, .start), (.reviewing, .start):
            nextState = .selectingObject
        case (.selectingObject, .regionSelected):
            nextState = .scanning
        case (.scanning, .stop):
            nextState = .processing
        case (.processing, .processingCompleted):
            nextState = .reviewing
        case (.failed(_), .retry), (_, .cancel):
            nextState = .ready
        case (.ready, .fail(_)),
             (.selectingObject, .fail(_)),
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

struct ScanCoverageCell: Hashable {
    let angularSector: Int
    let verticalBand: Int
}

struct ScanCylinderRegion: Equatable {
    static let radiusRange: ClosedRange<Float> = 0.06...0.30

    let lowerBoundary: SIMD3<Float>
    let upperBoundary: SIMD3<Float>
    let radius: Float
    let inwardDirection: SIMD3<Float>
    let axisDirection: SIMD3<Float>
    let referenceRadialDirection: SIMD3<Float>
    let angularTangent: SIMD3<Float>

    init?(
        lowerBoundary: SIMD3<Float>,
        upperBoundary: SIMD3<Float>,
        radius: Float,
        cameraPosition: SIMD3<Float>
    ) {
        let boundaryVector = upperBoundary - lowerBoundary
        let height = simd_length(boundaryVector)
        guard height.isFinite, height >= 0.05 else { return nil }

        let axisDirection = boundaryVector / height
        let boundaryCenter = (lowerBoundary + upperBoundary) / 2
        let cameraToObject = boundaryCenter - cameraPosition
        var projectedInward = cameraToObject
            - axisDirection * simd_dot(cameraToObject, axisDirection)
        if simd_length_squared(projectedInward) < 0.000_001 {
            let fallback = abs(axisDirection.y) < 0.9
                ? SIMD3<Float>(0, 1, 0)
                : SIMD3<Float>(1, 0, 0)
            projectedInward = simd_cross(fallback, axisDirection)
        }
        let inwardDirection = simd_normalize(projectedInward)
        let referenceRadialDirection = -inwardDirection
        let angularTangent = simd_normalize(
            simd_cross(axisDirection, referenceRadialDirection)
        )

        self.lowerBoundary = lowerBoundary
        self.upperBoundary = upperBoundary
        self.radius = min(max(radius, Self.radiusRange.lowerBound), Self.radiusRange.upperBound)
        self.inwardDirection = inwardDirection
        self.axisDirection = axisDirection
        self.referenceRadialDirection = referenceRadialDirection
        self.angularTangent = angularTangent
    }

    var height: Float {
        simd_distance(lowerBoundary, upperBoundary)
    }

    var axisStart: SIMD3<Float> {
        lowerBoundary + inwardDirection * radius
    }

    var axisEnd: SIMD3<Float> {
        upperBoundary + inwardDirection * radius
    }

    var center: SIMD3<Float> {
        (axisStart + axisEnd) / 2
    }

    func updatingRadius(_ newRadius: Float) -> ScanCylinderRegion {
        // The original camera position is no longer needed because the inward
        // direction is preserved explicitly while the cylinder expands.
        let clampedRadius = min(
            max(newRadius, Self.radiusRange.lowerBound),
            Self.radiusRange.upperBound
        )
        return ScanCylinderRegion(
            lowerBoundary: lowerBoundary,
            upperBoundary: upperBoundary,
            radius: clampedRadius,
            inwardDirection: inwardDirection,
            axisDirection: axisDirection,
            referenceRadialDirection: referenceRadialDirection,
            angularTangent: angularTangent
        )
    }

    func contains(_ point: SIMD3<Float>) -> Bool {
        let coordinates = cylindricalCoordinates(of: point)
        let axialPadding = min(max(height * 0.05, 0.015), 0.04)
        return coordinates.axialDistance >= -axialPadding
            && coordinates.axialDistance <= height + axialPadding
            && coordinates.radialDistance <= radius * 1.18
            && coordinates.radialDistance >= 0.015
    }

    func coverageCell(
        for point: SIMD3<Float>,
        angularSectorCount: Int,
        verticalBandCount: Int
    ) -> ScanCoverageCell? {
        precondition(angularSectorCount > 0)
        precondition(verticalBandCount > 0)
        let coordinates = cylindricalCoordinates(of: point)
        guard coordinates.axialDistance >= 0,
              coordinates.axialDistance <= height,
              coordinates.radialDistance >= radius * 0.25,
              coordinates.radialDistance <= radius * 1.18 else {
            return nil
        }

        let verticalFraction = min(max(coordinates.axialDistance / height, 0), 0.999_999)
        return ScanCoverageCell(
            angularSector: angularSector(forAngle: coordinates.angle, count: angularSectorCount),
            verticalBand: min(Int(verticalFraction * Float(verticalBandCount)), verticalBandCount - 1)
        )
    }

    func viewSector(for cameraPosition: SIMD3<Float>, count: Int) -> Int? {
        precondition(count > 0)
        let coordinates = cylindricalCoordinates(of: cameraPosition)
        guard coordinates.radialDistance >= radius + 0.05,
              coordinates.radialDistance <= radius + 3 else {
            return nil
        }
        return angularSector(forAngle: coordinates.angle, count: count)
    }

    func cameraSurfaceDistance(_ cameraPosition: SIMD3<Float>) -> Float {
        cylindricalCoordinates(of: cameraPosition).radialDistance - radius
    }

    private init(
        lowerBoundary: SIMD3<Float>,
        upperBoundary: SIMD3<Float>,
        radius: Float,
        inwardDirection: SIMD3<Float>,
        axisDirection: SIMD3<Float>,
        referenceRadialDirection: SIMD3<Float>,
        angularTangent: SIMD3<Float>
    ) {
        self.lowerBoundary = lowerBoundary
        self.upperBoundary = upperBoundary
        self.radius = radius
        self.inwardDirection = inwardDirection
        self.axisDirection = axisDirection
        self.referenceRadialDirection = referenceRadialDirection
        self.angularTangent = angularTangent
    }

    private func cylindricalCoordinates(
        of point: SIMD3<Float>
    ) -> (axialDistance: Float, radialDistance: Float, angle: Float) {
        let fromStart = point - axisStart
        let axialDistance = simd_dot(fromStart, axisDirection)
        let radial = fromStart - axisDirection * axialDistance
        let radialDistance = simd_length(radial)
        let angle = atan2(
            simd_dot(radial, angularTangent),
            simd_dot(radial, referenceRadialDirection)
        )
        return (axialDistance, radialDistance, angle)
    }

    private func angularSector(forAngle angle: Float, count: Int) -> Int {
        let fullTurn = 2 * Float.pi
        let normalizedAngle = (angle + fullTurn).truncatingRemainder(dividingBy: fullTurn)
        let sectorWidth = fullTurn / Float(count)
        let centeredAngle = (normalizedAngle + sectorWidth / 2)
            .truncatingRemainder(dividingBy: fullTurn)
        return min(Int(centeredAngle / sectorWidth), count - 1)
    }
}

struct ScanCoverageTracker {
    let sectorCount: Int
    let verticalBandCount: Int
    let minimumSamplesPerCell: Int
    private(set) var visitedViewSectors: Set<Int> = []
    private(set) var surfaceSampleCounts: [ScanCoverageCell: Int] = [:]

    init(
        sectorCount: Int = 12,
        verticalBandCount: Int = 3,
        minimumSamplesPerCell: Int = 6
    ) {
        precondition(sectorCount > 0)
        precondition(verticalBandCount > 0)
        precondition(minimumSamplesPerCell > 0)
        self.sectorCount = sectorCount
        self.verticalBandCount = verticalBandCount
        self.minimumSamplesPerCell = minimumSamplesPerCell
    }

    var progress: Float {
        let requiredSurfaceCellCount = sectorCount * verticalBandCount
        let completed = visitedViewSectors.count + completedSurfaceCellCount
        return Float(completed) / Float(sectorCount + requiredSurfaceCellCount)
    }

    var isComplete: Bool {
        visitedViewSectors.count == sectorCount
            && completedSurfaceCellCount == sectorCount * verticalBandCount
    }

    var remainingViewSectorCount: Int {
        sectorCount - visitedViewSectors.count
    }

    var remainingSurfaceCellCount: Int {
        sectorCount * verticalBandCount - completedSurfaceCellCount
    }

    var lowerBandProgress: Float {
        bandProgress(0)
    }

    var backProgress: Float {
        let backCenter = sectorCount / 2
        let backSectors = Set([
            normalizedSector(backCenter - 1),
            normalizedSector(backCenter),
            normalizedSector(backCenter + 1)
        ])
        let requiredSurfaceCells = backSectors.count * verticalBandCount
        let completedSurfaceCells = surfaceSampleCounts.reduce(into: 0) { count, item in
            if backSectors.contains(item.key.angularSector),
               item.value >= minimumSamplesPerCell {
                count += 1
            }
        }
        let visitedViews = visitedViewSectors.intersection(backSectors).count
        return Float(completedSurfaceCells + visitedViews)
            / Float(requiredSurfaceCells + backSectors.count)
    }

    @discardableResult
    mutating func observe(
        cameraPosition: SIMD3<Float>,
        region: ScanCylinderRegion
    ) -> Bool {
        guard let sector = region.viewSector(for: cameraPosition, count: sectorCount) else {
            return false
        }
        return visitedViewSectors.insert(sector).inserted
    }

    @discardableResult
    mutating func observe(surfaceCells: [ScanCoverageCell: Int]) -> Bool {
        var completedNewCell = false
        for (cell, sampleCount) in surfaceCells {
            guard (0..<sectorCount).contains(cell.angularSector),
                  (0..<verticalBandCount).contains(cell.verticalBand),
                  sampleCount > 0 else { continue }
            let previousCount = surfaceSampleCounts[cell, default: 0]
            let nextCount = min(previousCount + sampleCount, minimumSamplesPerCell)
            surfaceSampleCounts[cell] = nextCount
            if previousCount < minimumSamplesPerCell,
               nextCount >= minimumSamplesPerCell {
                completedNewCell = true
            }
        }
        return completedNewCell
    }

    mutating func reset() {
        visitedViewSectors.removeAll(keepingCapacity: true)
        surfaceSampleCounts.removeAll(keepingCapacity: true)
    }

    private var completedSurfaceCellCount: Int {
        surfaceSampleCounts.values.reduce(into: 0) { count, sampleCount in
            if sampleCount >= minimumSamplesPerCell {
                count += 1
            }
        }
    }

    private func bandProgress(_ band: Int) -> Float {
        guard (0..<verticalBandCount).contains(band) else { return 0 }
        let completedCount = surfaceSampleCounts.reduce(into: 0) { count, item in
            if item.key.verticalBand == band,
               item.value >= minimumSamplesPerCell {
                count += 1
            }
        }
        return Float(completedCount) / Float(sectorCount)
    }

    private func normalizedSector(_ sector: Int) -> Int {
        (sector % sectorCount + sectorCount) % sectorCount
    }
}

enum ScanGuidance: Equatable {
    case tooFast
    case increaseDistance
    case moveCloser
    case lowerAreaMissing
    case moveToBack
    case keepCircling
    case complete

    var text: String {
        switch self {
        case .tooFast:
            "Zu schnell"
        case .increaseDistance:
            "Mehr Abstand halten"
        case .moveCloser:
            "Näher an das Bein gehen"
        case .lowerAreaMissing:
            "Unterer Bereich fehlt"
        case .moveToBack:
            "Bewege dich zur Rückseite"
        case .keepCircling:
            "Langsam weiter um das Bein bewegen"
        case .complete:
            "360° vollständig erfasst"
        }
    }
}

enum ScanGuidanceEvaluator {
    static func guidance(
        coverage: ScanCoverageTracker,
        cameraPosition: SIMD3<Float>,
        region: ScanCylinderRegion,
        movingTooFast: Bool
    ) -> ScanGuidance {
        if movingTooFast { return .tooFast }

        let distance = region.cameraSurfaceDistance(cameraPosition)
        if distance < 0.25 { return .increaseDistance }
        if distance > 1.2 { return .moveCloser }
        if coverage.lowerBandProgress < 0.6, coverage.progress >= 0.25 {
            return .lowerAreaMissing
        }
        if coverage.backProgress < 0.7, coverage.progress >= 0.30 {
            return .moveToBack
        }
        return coverage.isComplete ? .complete : .keepCircling
    }
}
