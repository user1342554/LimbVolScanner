import ARKit
import Foundation
import simd

struct WorldPointCloudIntegrationResult {
    let addedPointCount: Int
    let totalPointCount: Int
    let reachedCapacity: Bool
    let acceptedSampleCount: Int
    let rejectedInvalidDepthPointCount: Int
    let rejectedLowConfidencePointCount: Int
    let rejectedOutlierPointCount: Int
    let rejectedOutsideRegionPointCount: Int
    let mergedDuplicatePointCount: Int
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
    let minimumDepth: Float
    let maximumDepth: Float
    let minimumConfidence: UInt8
    let localOutlierNeighborhoodRadius: Int
    let minimumLocalNeighborCount: Int
    let absoluteLocalDepthTolerance: Float
    let relativeLocalDepthTolerance: Float
    let voxelOutlierNeighborhoodRadius: Int
    let minimumVoxelNeighborCount: Int
    private var voxels: [VoxelKey: VoxelSample] = [:]
    private var regionCenter: SIMD3<Float>?
    private var maximumRegionRadius: Float?

    init(
        voxelSize: Float = 0.008,
        sampleStride: Int = 2,
        maximumPointCount: Int = 120_000,
        minimumDepth: Float = 0.2,
        maximumDepth: Float = 3,
        minimumConfidence: UInt8 = UInt8(ARConfidenceLevel.medium.rawValue),
        localOutlierNeighborhoodRadius: Int = 1,
        minimumLocalNeighborCount: Int = 2,
        absoluteLocalDepthTolerance: Float = 0.025,
        relativeLocalDepthTolerance: Float = 0.015,
        voxelOutlierNeighborhoodRadius: Int = 2,
        minimumVoxelNeighborCount: Int = 1
    ) {
        precondition(voxelSize > 0)
        precondition(sampleStride > 0)
        precondition(maximumPointCount > 0)
        precondition(minimumDepth > 0 && maximumDepth > minimumDepth)
        precondition(localOutlierNeighborhoodRadius >= 0)
        precondition(minimumLocalNeighborCount >= 0)
        precondition(absoluteLocalDepthTolerance >= 0)
        precondition(relativeLocalDepthTolerance >= 0)
        precondition(voxelOutlierNeighborhoodRadius >= 0)
        precondition(minimumVoxelNeighborCount >= 0)
        precondition(minimumVoxelNeighborCount == 0 || voxelOutlierNeighborhoodRadius > 0)
        self.voxelSize = voxelSize
        self.sampleStride = sampleStride
        self.maximumPointCount = maximumPointCount
        self.minimumDepth = minimumDepth
        self.maximumDepth = maximumDepth
        self.minimumConfidence = minimumConfidence
        self.localOutlierNeighborhoodRadius = localOutlierNeighborhoodRadius
        self.minimumLocalNeighborCount = minimumLocalNeighborCount
        self.absoluteLocalDepthTolerance = absoluteLocalDepthTolerance
        self.relativeLocalDepthTolerance = relativeLocalDepthTolerance
        self.voxelOutlierNeighborhoodRadius = voxelOutlierNeighborhoodRadius
        self.minimumVoxelNeighborCount = minimumVoxelNeighborCount
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

        guard let confidenceMap = frame.confidenceMap,
              confidenceMap.width == depthMap.width,
              confidenceMap.height == depthMap.height,
              confidenceMap.values.count == depthMap.values.count else {
            return result(
                addedPointCount: 0,
                rejectedLowConfidencePointCount: sampledPixelCount(
                    width: depthMap.width,
                    height: depthMap.height
                )
            )
        }
        let confidenceValues = confidenceMap.values

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

        var addedPointCount = 0
        var acceptedSampleCount = 0
        var rejectedInvalidDepthPointCount = 0
        var rejectedLowConfidencePointCount = 0
        var rejectedOutlierPointCount = 0
        var rejectedOutsideRegionPointCount = 0
        var mergedDuplicatePointCount = 0

        for y in Swift.stride(from: 0, to: depthMap.height, by: sampleStride) {
            for x in Swift.stride(from: 0, to: depthMap.width, by: sampleStride) {
                let index = y * depthMap.width + x
                let depth = depthMap.values[index]
                guard depth.isFinite, depth >= minimumDepth, depth <= maximumDepth else {
                    rejectedInvalidDepthPointCount += 1
                    continue
                }
                guard confidenceValues[index] >= minimumConfidence else {
                    rejectedLowConfidencePointCount += 1
                    continue
                }
                guard hasLocalDepthSupport(
                    x: x,
                    y: y,
                    depth: depth,
                    depthMap: depthMap,
                    confidenceValues: confidenceValues
                ) else {
                    rejectedOutlierPointCount += 1
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
                    rejectedOutsideRegionPointCount += 1
                    continue
                }

                acceptedSampleCount += 1
                let key = voxelKey(for: worldPoint)
                if var sample = voxels[key] {
                    let nextCount = min(sample.observationCount + 1, 32)
                    let weight = 1 / Float(nextCount)
                    sample.position += (worldPoint - sample.position) * weight
                    sample.observationCount = nextCount
                    voxels[key] = sample
                    mergedDuplicatePointCount += 1
                } else if voxels.count < maximumPointCount {
                    voxels[key] = VoxelSample(position: worldPoint, observationCount: 1)
                    addedPointCount += 1
                }
            }
        }

        return result(
            addedPointCount: addedPointCount,
            acceptedSampleCount: acceptedSampleCount,
            rejectedInvalidDepthPointCount: rejectedInvalidDepthPointCount,
            rejectedLowConfidencePointCount: rejectedLowConfidencePointCount,
            rejectedOutlierPointCount: rejectedOutlierPointCount,
            rejectedOutsideRegionPointCount: rejectedOutsideRegionPointCount,
            mergedDuplicatePointCount: mergedDuplicatePointCount
        )
    }

    func snapshot() -> [SIMD3<Float>] {
        guard minimumVoxelNeighborCount > 0 else {
            return voxels.values.map(\.position)
        }
        return voxels.compactMap { key, sample in
            hasVoxelSupport(around: key) ? sample.position : nil
        }
    }

    private func hasLocalDepthSupport(
        x: Int,
        y: Int,
        depth: Float,
        depthMap: RawDepthMap,
        confidenceValues: [UInt8]
    ) -> Bool {
        guard minimumLocalNeighborCount > 0 else { return true }

        let tolerance = max(
            absoluteLocalDepthTolerance,
            depth * relativeLocalDepthTolerance
        )
        let minimumX = max(0, x - localOutlierNeighborhoodRadius)
        let maximumX = min(depthMap.width - 1, x + localOutlierNeighborhoodRadius)
        let minimumY = max(0, y - localOutlierNeighborhoodRadius)
        let maximumY = min(depthMap.height - 1, y + localOutlierNeighborhoodRadius)
        var supportingNeighborCount = 0

        for neighborY in minimumY...maximumY {
            for neighborX in minimumX...maximumX {
                guard neighborX != x || neighborY != y else { continue }
                let neighborIndex = neighborY * depthMap.width + neighborX
                let neighborDepth = depthMap.values[neighborIndex]
                guard neighborDepth.isFinite,
                      neighborDepth >= minimumDepth,
                      neighborDepth <= maximumDepth,
                      confidenceValues[neighborIndex] >= minimumConfidence,
                      abs(neighborDepth - depth) <= tolerance else {
                    continue
                }
                supportingNeighborCount += 1
                if supportingNeighborCount >= minimumLocalNeighborCount {
                    return true
                }
            }
        }
        return false
    }

    private func hasVoxelSupport(around key: VoxelKey) -> Bool {
        guard voxelOutlierNeighborhoodRadius > 0 else { return false }
        var neighborCount = 0
        for radius in 1...voxelOutlierNeighborhoodRadius {
            for zOffset in -radius...radius {
                for yOffset in -radius...radius {
                    for xOffset in -radius...radius {
                        guard max(abs(xOffset), abs(yOffset), abs(zOffset)) == radius else {
                            continue
                        }
                        let neighborKey = VoxelKey(
                            x: key.x + xOffset,
                            y: key.y + yOffset,
                            z: key.z + zOffset
                        )
                        guard voxels[neighborKey] != nil else { continue }
                        neighborCount += 1
                        if neighborCount >= minimumVoxelNeighborCount {
                            return true
                        }
                    }
                }
            }
        }
        return false
    }

    private func voxelKey(for point: SIMD3<Float>) -> VoxelKey {
        VoxelKey(
            x: Int(floor(point.x / voxelSize)),
            y: Int(floor(point.y / voxelSize)),
            z: Int(floor(point.z / voxelSize))
        )
    }

    private func sampledPixelCount(width: Int, height: Int) -> Int {
        let sampledWidth = (width + sampleStride - 1) / sampleStride
        let sampledHeight = (height + sampleStride - 1) / sampleStride
        return sampledWidth * sampledHeight
    }

    private func result(
        addedPointCount: Int,
        acceptedSampleCount: Int = 0,
        rejectedInvalidDepthPointCount: Int = 0,
        rejectedLowConfidencePointCount: Int = 0,
        rejectedOutlierPointCount: Int = 0,
        rejectedOutsideRegionPointCount: Int = 0,
        mergedDuplicatePointCount: Int = 0
    ) -> WorldPointCloudIntegrationResult {
        WorldPointCloudIntegrationResult(
            addedPointCount: addedPointCount,
            totalPointCount: voxels.count,
            reachedCapacity: voxels.count >= maximumPointCount,
            acceptedSampleCount: acceptedSampleCount,
            rejectedInvalidDepthPointCount: rejectedInvalidDepthPointCount,
            rejectedLowConfidencePointCount: rejectedLowConfidencePointCount,
            rejectedOutlierPointCount: rejectedOutlierPointCount,
            rejectedOutsideRegionPointCount: rejectedOutsideRegionPointCount,
            mergedDuplicatePointCount: mergedDuplicatePointCount
        )
    }
}
