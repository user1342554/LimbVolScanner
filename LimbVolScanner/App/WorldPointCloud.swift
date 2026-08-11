import ARKit
import Foundation
import simd

struct WorldPointCloudIntegrationResult {
    let addedPointCount: Int
    let totalPointCount: Int
    let reachedCapacity: Bool
}

final class WorldPointCloudBuilder {
    private struct VoxelKey: Hashable {
        let x: Int
        let y: Int
        let z: Int
    }

    private struct VoxelSample {
        var position: SIMD3<Float>
        var observationCount: Int
    }

    let voxelSize: Float
    let sampleStride: Int
    let maximumPointCount: Int
    private var voxels: [VoxelKey: VoxelSample] = [:]
    private var regionCenter: SIMD3<Float>?
    private var maximumRegionRadius: Float?

    init(
        voxelSize: Float = 0.008,
        sampleStride: Int = 2,
        maximumPointCount: Int = 120_000
    ) {
        precondition(voxelSize > 0)
        precondition(sampleStride > 0)
        precondition(maximumPointCount > 0)
        self.voxelSize = voxelSize
        self.sampleStride = sampleStride
        self.maximumPointCount = maximumPointCount
        voxels.reserveCapacity(maximumPointCount)
    }

    var pointCount: Int {
        voxels.count
    }

    func reset(
        regionCenter: SIMD3<Float>? = nil,
        maximumRegionRadius: Float? = nil
    ) {
        voxels.removeAll(keepingCapacity: true)
        self.regionCenter = regionCenter
        self.maximumRegionRadius = maximumRegionRadius
    }

    @discardableResult
    func integrate(_ frame: CapturedLiDARFrame) -> WorldPointCloudIntegrationResult {
        let depthMap = frame.depthMap
        guard
            depthMap.width > 0,
            depthMap.height > 0,
            depthMap.values.count == depthMap.width * depthMap.height,
            frame.cameraImageWidth > 0,
            frame.cameraImageHeight > 0
        else {
            return result(addedPointCount: 0)
        }

        let confidenceValues: [UInt8]?
        if let confidenceMap = frame.confidenceMap,
           confidenceMap.width == depthMap.width,
           confidenceMap.height == depthMap.height,
           confidenceMap.values.count == depthMap.values.count {
            confidenceValues = confidenceMap.values
        } else {
            confidenceValues = nil
        }

        let intrinsics = DepthPointProjector.scaledIntrinsics(
            frame.cameraIntrinsics,
            imageWidth: frame.cameraImageWidth,
            imageHeight: frame.cameraImageHeight,
            depthWidth: depthMap.width,
            depthHeight: depthMap.height
        )
        guard intrinsics.fx.isFinite, intrinsics.fx > 0,
              intrinsics.fy.isFinite, intrinsics.fy > 0 else {
            return result(addedPointCount: 0)
        }

        let confidenceThreshold = UInt8(ARConfidenceLevel.medium.rawValue)
        var addedPointCount = 0

        for y in Swift.stride(from: 0, to: depthMap.height, by: sampleStride) {
            for x in Swift.stride(from: 0, to: depthMap.width, by: sampleStride) {
                let index = y * depthMap.width + x
                let depth = depthMap.values[index]
                guard depth.isFinite, depth >= 0.15, depth <= 5 else { continue }
                if let confidenceValues,
                   confidenceValues[index] < confidenceThreshold {
                    continue
                }

                let cameraPoint = DepthPointProjector.cameraSpacePoint(
                    x: x,
                    y: y,
                    depth: depth,
                    intrinsics: intrinsics
                )
                let worldPoint = DepthPointProjector.worldSpacePoint(
                    cameraPoint: cameraPoint,
                    cameraTransform: frame.cameraTransform
                )
                guard worldPoint.x.isFinite,
                      worldPoint.y.isFinite,
                      worldPoint.z.isFinite else { continue }
                if let regionCenter,
                   let maximumRegionRadius,
                   simd_distance(worldPoint, regionCenter) > maximumRegionRadius {
                    continue
                }

                let key = voxelKey(for: worldPoint)
                if var sample = voxels[key] {
                    let nextCount = min(sample.observationCount + 1, 32)
                    let weight = 1 / Float(nextCount)
                    sample.position += (worldPoint - sample.position) * weight
                    sample.observationCount = nextCount
                    voxels[key] = sample
                } else if voxels.count < maximumPointCount {
                    voxels[key] = VoxelSample(position: worldPoint, observationCount: 1)
                    addedPointCount += 1
                }
            }
        }

        return result(addedPointCount: addedPointCount)
    }

    func snapshot() -> [SIMD3<Float>] {
        voxels.values.map(\.position)
    }

    private func voxelKey(for point: SIMD3<Float>) -> VoxelKey {
        VoxelKey(
            x: Int(floor(point.x / voxelSize)),
            y: Int(floor(point.y / voxelSize)),
            z: Int(floor(point.z / voxelSize))
        )
    }

    private func result(addedPointCount: Int) -> WorldPointCloudIntegrationResult {
        WorldPointCloudIntegrationResult(
            addedPointCount: addedPointCount,
            totalPointCount: voxels.count,
            reachedCapacity: voxels.count >= maximumPointCount
        )
    }
}
