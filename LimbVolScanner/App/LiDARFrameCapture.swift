import ARKit
import CoreVideo
import Foundation
import simd

struct RawDepthMap {
    let width: Int
    let height: Int
    let values: [Float32]

    var validSampleFraction: Float {
        guard !values.isEmpty else { return 0 }
        let validCount = values.reduce(into: 0) { count, depth in
            if depth.isFinite, depth >= 0.2, depth <= 3 {
                count += 1
            }
        }
        return Float(validCount) / Float(values.count)
    }
}

enum LiDARFrameCaptureDecision: Equatable {
    case capture
    case rejectLowQuality
    case rejectTooSoon
    case rejectInsufficientMotion
    case rejectExcessiveMotion
}

struct RawDepthConfidenceMap {
    let width: Int
    let height: Int
    let values: [UInt8]

    var mediumOrHighFraction: Float {
        guard !values.isEmpty else { return 0 }
        let threshold = UInt8(ARConfidenceLevel.medium.rawValue)
        let confidentCount = values.reduce(into: 0) { count, confidence in
            if confidence >= threshold {
                count += 1
            }
        }
        return Float(confidentCount) / Float(values.count)
    }
}

struct RawRGBPlane {
    let width: Int
    let height: Int
    let bytesPerRow: Int
    let bytes: Data
}

struct RawRGBImage {
    let width: Int
    let height: Int
    let pixelFormat: OSType
    let planes: [RawRGBPlane]
}

struct CapturedLiDARFrame {
    let depthMap: RawDepthMap
    let confidenceMap: RawDepthConfidenceMap?
    let cameraImageWidth: Int
    let cameraImageHeight: Int
    let cameraPosition: SIMD3<Float>
    let cameraRotation: simd_quatf
    let cameraTransform: simd_float4x4
    let cameraIntrinsics: simd_float3x3
    let timestamp: TimeInterval
    let rgbImage: RawRGBImage?
}

struct CapturedMeshAnchorGeometry {
    let identifier: UUID
    let transform: simd_float4x4
    let vertices: [SIMD3<Float>]
    let normals: [SIMD3<Float>]
    let triangleIndices: [UInt32]

    var triangleCount: Int {
        triangleIndices.count / 3
    }
}

struct MeshGeometryCaptureSummary: Equatable {
    let anchorCount: Int
    let vertexCount: Int
    let triangleCount: Int
}

final class ARMeshGeometryCollector {
    private let lock = NSLock()
    private var isCapturing = false
    private var geometriesByIdentifier: [UUID: CapturedMeshAnchorGeometry] = [:]

    func start() {
        lock.lock()
        geometriesByIdentifier.removeAll(keepingCapacity: true)
        isCapturing = true
        lock.unlock()
    }

    func stop() {
        lock.lock()
        isCapturing = false
        lock.unlock()
    }

    func reset() {
        lock.lock()
        isCapturing = false
        geometriesByIdentifier.removeAll(keepingCapacity: true)
        lock.unlock()
    }

    @discardableResult
    func upsert(anchors: [ARAnchor]) -> MeshGeometryCaptureSummary? {
        lock.lock()
        defer { lock.unlock() }
        guard isCapturing else { return nil }

        for meshAnchor in anchors.compactMap({ $0 as? ARMeshAnchor }) {
            geometriesByIdentifier[meshAnchor.identifier] = Self.copy(meshAnchor)
        }
        return summaryWithoutLock()
    }

    @discardableResult
    func remove(anchors: [ARAnchor]) -> MeshGeometryCaptureSummary? {
        lock.lock()
        defer { lock.unlock() }
        guard isCapturing else { return nil }

        for meshAnchor in anchors.compactMap({ $0 as? ARMeshAnchor }) {
            geometriesByIdentifier.removeValue(forKey: meshAnchor.identifier)
        }
        return summaryWithoutLock()
    }

    func snapshot() -> [CapturedMeshAnchorGeometry] {
        lock.lock()
        defer { lock.unlock() }
        return Array(geometriesByIdentifier.values)
    }

    func summary() -> MeshGeometryCaptureSummary {
        lock.lock()
        defer { lock.unlock() }
        return summaryWithoutLock()
    }

    private func summaryWithoutLock() -> MeshGeometryCaptureSummary {
        MeshGeometryCaptureSummary(
            anchorCount: geometriesByIdentifier.count,
            vertexCount: geometriesByIdentifier.values.reduce(0) { $0 + $1.vertices.count },
            triangleCount: geometriesByIdentifier.values.reduce(0) { $0 + $1.triangleCount }
        )
    }

    private static func copy(_ anchor: ARMeshAnchor) -> CapturedMeshAnchorGeometry {
        let geometry = anchor.geometry
        return CapturedMeshAnchorGeometry(
            identifier: anchor.identifier,
            transform: anchor.transform,
            vertices: copyVectors(from: geometry.vertices),
            normals: copyVectors(from: geometry.normals),
            triangleIndices: copyTriangleIndices(from: geometry.faces)
        )
    }

    private static func copyVectors(from source: ARGeometrySource) -> [SIMD3<Float>] {
        guard source.count > 0 else { return [] }
        let bufferStart = source.buffer.contents().advanced(by: source.offset)
        return (0..<source.count).map { index in
            bufferStart
                .advanced(by: source.stride * index)
                .assumingMemoryBound(to: SIMD3<Float>.self)
                .pointee
        }
    }

    private static func copyTriangleIndices(from faces: ARGeometryElement) -> [UInt32] {
        let indexCount = faces.count * faces.indexCountPerPrimitive
        guard indexCount > 0 else { return [] }
        let bufferStart = faces.buffer.contents()

        switch faces.bytesPerIndex {
        case MemoryLayout<UInt16>.size:
            return (0..<indexCount).map { index in
                UInt32(
                    bufferStart
                        .advanced(by: index * MemoryLayout<UInt16>.size)
                        .assumingMemoryBound(to: UInt16.self)
                        .pointee
                )
            }
        case MemoryLayout<UInt32>.size:
            return (0..<indexCount).map { index in
                bufferStart
                    .advanced(by: index * MemoryLayout<UInt32>.size)
                    .assumingMemoryBound(to: UInt32.self)
                    .pointee
            }
        default:
            return []
        }
    }
}

struct LiDARFrameCaptureGate {
    let minimumTimeInterval: TimeInterval
    let maximumTimeInterval: TimeInterval
    let minimumTranslation: Float
    let minimumRotationRadians: Float
    let minimumValidDepthFraction: Float
    let minimumConfidentDepthFraction: Float
    let maximumTranslationSpeed: Float
    let maximumRotationSpeedRadians: Float

    private var lastTimestamp: TimeInterval?
    private var lastPosition: SIMD3<Float>?
    private var lastRotation: simd_quatf?
    private var lastObservedTimestamp: TimeInterval?
    private var lastObservedPosition: SIMD3<Float>?
    private var lastObservedRotation: simd_quatf?

    init(
        minimumTimeInterval: TimeInterval = 0.15,
        maximumTimeInterval: TimeInterval = 0.8,
        minimumTranslation: Float = 0.015,
        minimumRotationRadians: Float = 2 * .pi / 180,
        minimumValidDepthFraction: Float = 0.05,
        minimumConfidentDepthFraction: Float = 0.02,
        maximumTranslationSpeed: Float = 0.65,
        maximumRotationSpeedRadians: Float = 1.2
    ) {
        precondition(minimumTimeInterval >= 0)
        precondition(maximumTimeInterval >= minimumTimeInterval)
        precondition(minimumTranslation >= 0)
        precondition(minimumRotationRadians >= 0)
        precondition((0...1).contains(minimumValidDepthFraction))
        precondition((0...1).contains(minimumConfidentDepthFraction))
        precondition(maximumTranslationSpeed > 0)
        precondition(maximumRotationSpeedRadians > 0)
        self.minimumTimeInterval = minimumTimeInterval
        self.maximumTimeInterval = maximumTimeInterval
        self.minimumTranslation = minimumTranslation
        self.minimumRotationRadians = minimumRotationRadians
        self.minimumValidDepthFraction = minimumValidDepthFraction
        self.minimumConfidentDepthFraction = minimumConfidentDepthFraction
        self.maximumTranslationSpeed = maximumTranslationSpeed
        self.maximumRotationSpeedRadians = maximumRotationSpeedRadians
    }

    mutating func evaluate(
        timestamp: TimeInterval,
        position: SIMD3<Float>,
        rotation: simd_quatf,
        validDepthFraction: Float,
        confidentDepthFraction: Float?
    ) -> LiDARFrameCaptureDecision {
        let previousObservedTimestamp = lastObservedTimestamp
        let previousObservedPosition = lastObservedPosition
        let previousObservedRotation = lastObservedRotation
        recordObservation(timestamp: timestamp, position: position, rotation: rotation)

        guard validDepthFraction >= minimumValidDepthFraction,
              let confidentDepthFraction,
              confidentDepthFraction >= minimumConfidentDepthFraction else {
            return .rejectLowQuality
        }

        guard let previousObservedTimestamp,
              let previousObservedPosition,
              let previousObservedRotation else {
            return .rejectTooSoon
        }
        let observedElapsed = timestamp - previousObservedTimestamp
        guard observedElapsed > 0 else { return .rejectTooSoon }

        let translationSpeed = simd_distance(position, previousObservedPosition)
            / Float(observedElapsed)
        let rotationSpeed = rotationAngle(
            from: previousObservedRotation,
            to: rotation
        ) / Float(observedElapsed)
        guard translationSpeed <= maximumTranslationSpeed,
              rotationSpeed <= maximumRotationSpeedRadians else {
            return .rejectExcessiveMotion
        }

        guard
            let lastTimestamp,
            let lastPosition,
            let lastRotation
        else {
            record(timestamp: timestamp, position: position, rotation: rotation)
            return .capture
        }

        let elapsed = timestamp - lastTimestamp
        guard elapsed >= minimumTimeInterval else { return .rejectTooSoon }

        let translation = simd_distance(position, lastPosition)
        let rotationDelta = rotationAngle(from: lastRotation, to: rotation)
        guard
            translation >= minimumTranslation
                || rotationDelta >= minimumRotationRadians
                || elapsed >= maximumTimeInterval
        else {
            return .rejectInsufficientMotion
        }

        record(timestamp: timestamp, position: position, rotation: rotation)
        return .capture
    }

    mutating func shouldCapture(
        timestamp: TimeInterval,
        position: SIMD3<Float>,
        rotation: simd_quatf,
        validDepthFraction: Float,
        confidentDepthFraction: Float?
    ) -> Bool {
        evaluate(
            timestamp: timestamp,
            position: position,
            rotation: rotation,
            validDepthFraction: validDepthFraction,
            confidentDepthFraction: confidentDepthFraction
        ) == .capture
    }

    mutating func reset() {
        lastTimestamp = nil
        lastPosition = nil
        lastRotation = nil
        lastObservedTimestamp = nil
        lastObservedPosition = nil
        lastObservedRotation = nil
    }

    private mutating func record(
        timestamp: TimeInterval,
        position: SIMD3<Float>,
        rotation: simd_quatf
    ) {
        lastTimestamp = timestamp
        lastPosition = position
        lastRotation = rotation
    }

    private mutating func recordObservation(
        timestamp: TimeInterval,
        position: SIMD3<Float>,
        rotation: simd_quatf
    ) {
        lastObservedTimestamp = timestamp
        lastObservedPosition = position
        lastObservedRotation = rotation
    }

    private func rotationAngle(from start: simd_quatf, to end: simd_quatf) -> Float {
        let relativeRotation = start.inverse * end
        let clampedReal = min(max(abs(relativeRotation.real), 0), 1)
        return 2 * acos(clampedReal)
    }
}

final class RawLiDARFrameCollector {
    struct CaptureResult {
        let frameCount: Int
        let reachedCapacity: Bool
        let capturedFrame: CapturedLiDARFrame
    }

    private(set) var frames: [CapturedLiDARFrame] = []
    private var gate = LiDARFrameCaptureGate()
    private let maximumFrameCount: Int
    private let rgbKeyframeInterval: Int
    private(set) var lastCaptureDecision: LiDARFrameCaptureDecision?

    init(maximumFrameCount: Int = 360, rgbKeyframeInterval: Int = 10) {
        self.maximumFrameCount = maximumFrameCount
        self.rgbKeyframeInterval = rgbKeyframeInterval
    }

    func reset() {
        frames.removeAll(keepingCapacity: true)
        gate.reset()
        lastCaptureDecision = nil
    }

    func capture(frame: ARFrame) -> CaptureResult? {
        lastCaptureDecision = nil
        guard frames.count < maximumFrameCount else { return nil }
        guard case .normal = frame.camera.trackingState else { return nil }
        guard let sceneDepth = frame.sceneDepth ?? frame.smoothedSceneDepth else { return nil }
        guard let depthMap = Self.copyDepthMap(sceneDepth.depthMap) else { return nil }
        guard let confidenceMap = sceneDepth.confidenceMap.flatMap(Self.copyConfidenceMap) else {
            lastCaptureDecision = .rejectLowQuality
            return nil
        }

        let cameraTransform = frame.camera.transform
        let translation = cameraTransform.columns.3
        let cameraPosition = SIMD3<Float>(translation.x, translation.y, translation.z)
        let cameraRotationMatrix = simd_float3x3(
            SIMD3<Float>(
                cameraTransform.columns.0.x,
                cameraTransform.columns.0.y,
                cameraTransform.columns.0.z
            ),
            SIMD3<Float>(
                cameraTransform.columns.1.x,
                cameraTransform.columns.1.y,
                cameraTransform.columns.1.z
            ),
            SIMD3<Float>(
                cameraTransform.columns.2.x,
                cameraTransform.columns.2.y,
                cameraTransform.columns.2.z
            )
        )
        let cameraRotation = simd_quatf(cameraRotationMatrix)
        let captureDecision = gate.evaluate(
            timestamp: frame.timestamp,
            position: cameraPosition,
            rotation: cameraRotation,
            validDepthFraction: depthMap.validSampleFraction,
            confidentDepthFraction: confidenceMap.mediumOrHighFraction
        )
        lastCaptureDecision = captureDecision
        guard captureDecision == .capture else {
            return nil
        }

        let shouldCaptureRGB = rgbKeyframeInterval > 0
            && frames.count.isMultiple(of: rgbKeyframeInterval)
        let capturedFrame = CapturedLiDARFrame(
            depthMap: depthMap,
            confidenceMap: confidenceMap,
            cameraImageWidth: CVPixelBufferGetWidth(frame.capturedImage),
            cameraImageHeight: CVPixelBufferGetHeight(frame.capturedImage),
            cameraPosition: cameraPosition,
            cameraRotation: cameraRotation,
            cameraTransform: cameraTransform,
            cameraIntrinsics: frame.camera.intrinsics,
            timestamp: frame.timestamp,
            rgbImage: shouldCaptureRGB ? Self.copyRGBImage(frame.capturedImage) : nil
        )
        frames.append(capturedFrame)
        return CaptureResult(
            frameCount: frames.count,
            reachedCapacity: frames.count == maximumFrameCount,
            capturedFrame: capturedFrame
        )
    }

    private static func copyDepthMap(_ pixelBuffer: CVPixelBuffer) -> RawDepthMap? {
        guard CVPixelBufferGetPixelFormatType(pixelBuffer) == kCVPixelFormatType_DepthFloat32 else {
            return nil
        }
        let width = CVPixelBufferGetWidth(pixelBuffer)
        let height = CVPixelBufferGetHeight(pixelBuffer)
        guard width > 0, height > 0 else { return nil }

        CVPixelBufferLockBaseAddress(pixelBuffer, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(pixelBuffer, .readOnly) }
        guard let baseAddress = CVPixelBufferGetBaseAddress(pixelBuffer) else { return nil }

        let bytesPerRow = CVPixelBufferGetBytesPerRow(pixelBuffer)
        var values: [Float32] = []
        values.reserveCapacity(width * height)
        for rowIndex in 0..<height {
            let row = baseAddress
                .advanced(by: rowIndex * bytesPerRow)
                .assumingMemoryBound(to: Float32.self)
            values.append(contentsOf: UnsafeBufferPointer(start: row, count: width))
        }
        return RawDepthMap(width: width, height: height, values: values)
    }

    private static func copyConfidenceMap(
        _ pixelBuffer: CVPixelBuffer
    ) -> RawDepthConfidenceMap? {
        guard CVPixelBufferGetPixelFormatType(pixelBuffer) == kCVPixelFormatType_OneComponent8 else {
            return nil
        }
        let width = CVPixelBufferGetWidth(pixelBuffer)
        let height = CVPixelBufferGetHeight(pixelBuffer)
        guard width > 0, height > 0 else { return nil }

        CVPixelBufferLockBaseAddress(pixelBuffer, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(pixelBuffer, .readOnly) }
        guard let baseAddress = CVPixelBufferGetBaseAddress(pixelBuffer) else { return nil }

        let bytesPerRow = CVPixelBufferGetBytesPerRow(pixelBuffer)
        var values: [UInt8] = []
        values.reserveCapacity(width * height)
        for rowIndex in 0..<height {
            let row = baseAddress
                .advanced(by: rowIndex * bytesPerRow)
                .assumingMemoryBound(to: UInt8.self)
            values.append(contentsOf: UnsafeBufferPointer(start: row, count: width))
        }
        return RawDepthConfidenceMap(width: width, height: height, values: values)
    }

    private static func copyRGBImage(_ pixelBuffer: CVPixelBuffer) -> RawRGBImage? {
        let width = CVPixelBufferGetWidth(pixelBuffer)
        let height = CVPixelBufferGetHeight(pixelBuffer)
        guard width > 0, height > 0 else { return nil }

        CVPixelBufferLockBaseAddress(pixelBuffer, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(pixelBuffer, .readOnly) }

        let planeCount = CVPixelBufferGetPlaneCount(pixelBuffer)
        var planes: [RawRGBPlane] = []
        if planeCount > 0 {
            planes.reserveCapacity(planeCount)
            for planeIndex in 0..<planeCount {
                guard let baseAddress = CVPixelBufferGetBaseAddressOfPlane(
                    pixelBuffer,
                    planeIndex
                ) else {
                    return nil
                }
                let planeHeight = CVPixelBufferGetHeightOfPlane(pixelBuffer, planeIndex)
                let bytesPerRow = CVPixelBufferGetBytesPerRowOfPlane(pixelBuffer, planeIndex)
                planes.append(
                    RawRGBPlane(
                        width: CVPixelBufferGetWidthOfPlane(pixelBuffer, planeIndex),
                        height: planeHeight,
                        bytesPerRow: bytesPerRow,
                        bytes: Data(bytes: baseAddress, count: bytesPerRow * planeHeight)
                    )
                )
            }
        } else {
            guard let baseAddress = CVPixelBufferGetBaseAddress(pixelBuffer) else { return nil }
            let bytesPerRow = CVPixelBufferGetBytesPerRow(pixelBuffer)
            planes.append(
                RawRGBPlane(
                    width: width,
                    height: height,
                    bytesPerRow: bytesPerRow,
                    bytes: Data(bytes: baseAddress, count: bytesPerRow * height)
                )
            )
        }

        return RawRGBImage(
            width: width,
            height: height,
            pixelFormat: CVPixelBufferGetPixelFormatType(pixelBuffer),
            planes: planes
        )
    }
}
