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
            if depth.isFinite, depth >= 0.15, depth <= 5 {
                count += 1
            }
        }
        return Float(validCount) / Float(values.count)
    }
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

struct LiDARFrameCaptureGate {
    let minimumTimeInterval: TimeInterval
    let maximumTimeInterval: TimeInterval
    let minimumTranslation: Float
    let minimumRotationRadians: Float
    let minimumValidDepthFraction: Float
    let minimumConfidentDepthFraction: Float

    private var lastTimestamp: TimeInterval?
    private var lastPosition: SIMD3<Float>?
    private var lastRotation: simd_quatf?

    init(
        minimumTimeInterval: TimeInterval = 0.15,
        maximumTimeInterval: TimeInterval = 0.8,
        minimumTranslation: Float = 0.015,
        minimumRotationRadians: Float = 2 * .pi / 180,
        minimumValidDepthFraction: Float = 0.05,
        minimumConfidentDepthFraction: Float = 0.02
    ) {
        self.minimumTimeInterval = minimumTimeInterval
        self.maximumTimeInterval = maximumTimeInterval
        self.minimumTranslation = minimumTranslation
        self.minimumRotationRadians = minimumRotationRadians
        self.minimumValidDepthFraction = minimumValidDepthFraction
        self.minimumConfidentDepthFraction = minimumConfidentDepthFraction
    }

    mutating func shouldCapture(
        timestamp: TimeInterval,
        position: SIMD3<Float>,
        rotation: simd_quatf,
        validDepthFraction: Float,
        confidentDepthFraction: Float?
    ) -> Bool {
        guard validDepthFraction >= minimumValidDepthFraction else { return false }
        if let confidentDepthFraction,
           confidentDepthFraction < minimumConfidentDepthFraction {
            return false
        }

        guard
            let lastTimestamp,
            let lastPosition,
            let lastRotation
        else {
            record(timestamp: timestamp, position: position, rotation: rotation)
            return true
        }

        let elapsed = timestamp - lastTimestamp
        guard elapsed >= minimumTimeInterval else { return false }

        let translation = simd_distance(position, lastPosition)
        let relativeRotation = lastRotation.inverse * rotation
        let clampedReal = min(max(abs(relativeRotation.real), 0), 1)
        let rotationAngle = 2 * acos(clampedReal)
        guard
            translation >= minimumTranslation
                || rotationAngle >= minimumRotationRadians
                || elapsed >= maximumTimeInterval
        else {
            return false
        }

        record(timestamp: timestamp, position: position, rotation: rotation)
        return true
    }

    mutating func reset() {
        lastTimestamp = nil
        lastPosition = nil
        lastRotation = nil
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

    init(maximumFrameCount: Int = 360, rgbKeyframeInterval: Int = 10) {
        self.maximumFrameCount = maximumFrameCount
        self.rgbKeyframeInterval = rgbKeyframeInterval
    }

    func reset() {
        frames.removeAll(keepingCapacity: true)
        gate.reset()
    }

    func capture(frame: ARFrame) -> CaptureResult? {
        guard frames.count < maximumFrameCount else { return nil }
        guard case .normal = frame.camera.trackingState else { return nil }
        guard let sceneDepth = frame.sceneDepth ?? frame.smoothedSceneDepth else { return nil }
        guard let depthMap = Self.copyDepthMap(sceneDepth.depthMap) else { return nil }
        let confidenceMap = sceneDepth.confidenceMap.flatMap(Self.copyConfidenceMap)

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
        guard gate.shouldCapture(
            timestamp: frame.timestamp,
            position: cameraPosition,
            rotation: cameraRotation,
            validDepthFraction: depthMap.validSampleFraction,
            confidentDepthFraction: confidenceMap?.mediumOrHighFraction
        ) else {
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
