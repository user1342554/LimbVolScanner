import XCTest
import simd
@testable import LimbVolScanner

final class LimbVolScannerTests: XCTestCase {
    func testSimulatorReportsLiDARAsUnsupported() {
#if targetEnvironment(simulator)
        XCTAssertFalse(LiDARSupport.isAvailable)
#endif
    }

    func testDepthPointAtOpticalCenterProjectsStraightAhead() {
        let intrinsics = DepthPointProjector.Intrinsics(
            fx: 100,
            fy: 100,
            cx: 50,
            cy: 40
        )

        let point = DepthPointProjector.cameraSpacePoint(
            x: 50,
            y: 40,
            depth: 2,
            intrinsics: intrinsics
        )

        XCTAssertEqual(point.x, 0, accuracy: 0.0001)
        XCTAssertEqual(point.y, 0, accuracy: 0.0001)
        XCTAssertEqual(point.z, -2, accuracy: 0.0001)
    }

    func testDepthPointProjectionUsesCameraAxes() {
        let intrinsics = DepthPointProjector.Intrinsics(
            fx: 100,
            fy: 100,
            cx: 50,
            cy: 40
        )

        let point = DepthPointProjector.cameraSpacePoint(
            x: 60,
            y: 30,
            depth: 2,
            intrinsics: intrinsics
        )

        XCTAssertEqual(point.x, 0.2, accuracy: 0.0001)
        XCTAssertEqual(point.y, 0.2, accuracy: 0.0001)
        XCTAssertEqual(point.z, -2, accuracy: 0.0001)
    }

    func testIntrinsicsAreScaledToDepthResolution() {
        let cameraIntrinsics = simd_float3x3(
            SIMD3<Float>(1000, 0, 0),
            SIMD3<Float>(0, 800, 0),
            SIMD3<Float>(500, 400, 1)
        )

        let scaled = DepthPointProjector.scaledIntrinsics(
            cameraIntrinsics,
            imageWidth: 1000,
            imageHeight: 800,
            depthWidth: 250,
            depthHeight: 200
        )

        XCTAssertEqual(scaled.fx, 250, accuracy: 0.0001)
        XCTAssertEqual(scaled.fy, 200, accuracy: 0.0001)
        XCTAssertEqual(scaled.cx, 125, accuracy: 0.0001)
        XCTAssertEqual(scaled.cy, 100, accuracy: 0.0001)
    }

    func testCameraPointIsTransformedIntoWorldSpace() {
        var cameraTransform = matrix_identity_float4x4
        cameraTransform.columns.3 = SIMD4<Float>(1, 2, 3, 1)

        let point = DepthPointProjector.worldSpacePoint(
            cameraPoint: SIMD3<Float>(0.25, -0.5, -2),
            cameraTransform: cameraTransform
        )

        XCTAssertEqual(point.x, 1.25, accuracy: 0.0001)
        XCTAssertEqual(point.y, 1.5, accuracy: 0.0001)
        XCTAssertEqual(point.z, 1, accuracy: 0.0001)
    }

    func testPointCloudUnprojectsDepthIntoWorldSpace() {
        let builder = WorldPointCloudBuilder(
            voxelSize: 0.001,
            sampleStride: 1,
            maximumPointCount: 10,
            minimumLocalNeighborCount: 0,
            minimumVoxelNeighborCount: 0
        )
        let frame = capturedFrame(
            depthValues: [.nan, 1, .nan],
            cameraTransform: matrix_identity_float4x4
        )

        let result = builder.integrate(frame)
        let points = builder.snapshot()

        XCTAssertEqual(result.addedPointCount, 1)
        XCTAssertEqual(points.count, 1)
        XCTAssertEqual(points[0].x, 0, accuracy: 0.0001)
        XCTAssertEqual(points[0].y, 0, accuracy: 0.0001)
        XCTAssertEqual(points[0].z, -1, accuracy: 0.0001)
    }

    func testTwoCameraViewsShareOneWorldCoordinateSystem() {
        let builder = WorldPointCloudBuilder(
            voxelSize: 0.01,
            sampleStride: 1,
            maximumPointCount: 10,
            minimumLocalNeighborCount: 0,
            minimumVoxelNeighborCount: 0
        )
        let firstFrame = capturedFrame(
            depthValues: [.nan, 1, .nan],
            cameraTransform: matrix_identity_float4x4
        )
        var translatedCamera = matrix_identity_float4x4
        translatedCamera.columns.3 = SIMD4<Float>(1, 0, 0, 1)
        let secondFrame = capturedFrame(
            depthValues: [1, .nan, .nan],
            cameraTransform: translatedCamera
        )

        builder.integrate(firstFrame)
        builder.integrate(secondFrame)
        let points = builder.snapshot()

        XCTAssertEqual(points.count, 1, "Both views of the stationary point must occupy one world voxel")
        XCTAssertEqual(points[0].x, 0, accuracy: 0.0001)
        XCTAssertEqual(points[0].y, 0, accuracy: 0.0001)
        XCTAssertEqual(points[0].z, -1, accuracy: 0.0001)
    }

    func testPointCloudRejectsLowConfidenceDepthPoints() {
        let builder = WorldPointCloudBuilder(
            voxelSize: 0.001,
            sampleStride: 1,
            maximumPointCount: 10,
            minimumLocalNeighborCount: 0,
            minimumVoxelNeighborCount: 0
        )
        let frame = capturedFrame(
            depthValues: [1, 1, 1],
            confidenceValues: [2, 0, 2],
            cameraTransform: matrix_identity_float4x4
        )

        let result = builder.integrate(frame)

        XCTAssertEqual(result.acceptedSampleCount, 2)
        XCTAssertEqual(result.rejectedLowConfidencePointCount, 1)
        XCTAssertEqual(builder.snapshot().count, 2)
    }

    func testPointCloudRequiresAConfidenceMap() {
        let builder = WorldPointCloudBuilder(
            sampleStride: 1,
            minimumLocalNeighborCount: 0,
            minimumVoxelNeighborCount: 0
        )
        let frame = capturedFrame(
            depthValues: [1, 1, 1],
            includeConfidenceMap: false,
            cameraTransform: matrix_identity_float4x4
        )

        let result = builder.integrate(frame)

        XCTAssertEqual(result.rejectedLowConfidencePointCount, 3)
        XCTAssertTrue(builder.snapshot().isEmpty)
    }

    func testPointCloudRejectsDepthOutsideUsefulRange() {
        let builder = WorldPointCloudBuilder(
            voxelSize: 0.001,
            sampleStride: 1,
            maximumPointCount: 10,
            minimumLocalNeighborCount: 0,
            minimumVoxelNeighborCount: 0
        )
        let frame = capturedFrame(
            depthValues: [0.19, 1, 3.01],
            cameraTransform: matrix_identity_float4x4
        )

        let result = builder.integrate(frame)

        XCTAssertEqual(result.acceptedSampleCount, 1)
        XCTAssertEqual(result.rejectedInvalidDepthPointCount, 2)
        XCTAssertEqual(builder.snapshot().count, 1)
    }

    func testPointCloudRejectsAnIsolatedDepthSpike() {
        let builder = WorldPointCloudBuilder(
            voxelSize: 0.008,
            sampleStride: 1,
            maximumPointCount: 20,
            minimumVoxelNeighborCount: 0
        )
        let frame = capturedFrame(
            depthValues: [
                .nan, .nan, .nan,
                .nan, 1, .nan,
                .nan, .nan, .nan
            ],
            width: 3,
            height: 3,
            cameraTransform: matrix_identity_float4x4
        )

        let result = builder.integrate(frame)

        XCTAssertEqual(result.rejectedOutlierPointCount, 1)
        XCTAssertTrue(builder.snapshot().isEmpty)
    }

    func testPointCloudKeepsACompactSupportedSurface() {
        let builder = WorldPointCloudBuilder(
            voxelSize: 0.008,
            sampleStride: 1,
            maximumPointCount: 20
        )
        let frame = capturedFrame(
            depthValues: Array(repeating: 1, count: 9),
            width: 3,
            height: 3,
            focalLength: 100,
            cameraTransform: matrix_identity_float4x4
        )

        let result = builder.integrate(frame)

        XCTAssertEqual(result.acceptedSampleCount, 9)
        XCTAssertEqual(result.rejectedOutlierPointCount, 0)
        XCTAssertEqual(builder.snapshot().count, 9)
    }

    func testPointCloudSnapshotRemovesAnIsolatedWorldVoxel() {
        let builder = WorldPointCloudBuilder(
            voxelSize: 0.008,
            sampleStride: 1,
            maximumPointCount: 10,
            minimumLocalNeighborCount: 0
        )
        let frame = capturedFrame(
            depthValues: [.nan, 1, .nan],
            cameraTransform: matrix_identity_float4x4
        )

        let result = builder.integrate(frame)

        XCTAssertEqual(result.addedPointCount, 1)
        XCTAssertTrue(builder.snapshot().isEmpty)
    }

    func testPointCloudMergesDuplicateSamplesIntoOneVoxel() {
        let builder = WorldPointCloudBuilder(
            voxelSize: 0.008,
            sampleStride: 1,
            maximumPointCount: 10,
            minimumLocalNeighborCount: 0,
            minimumVoxelNeighborCount: 0
        )
        let frame = capturedFrame(
            depthValues: [.nan, 1, .nan],
            cameraTransform: matrix_identity_float4x4
        )

        let firstResult = builder.integrate(frame)
        let secondResult = builder.integrate(frame)

        XCTAssertEqual(firstResult.addedPointCount, 1)
        XCTAssertEqual(secondResult.addedPointCount, 0)
        XCTAssertEqual(secondResult.mergedDuplicatePointCount, 1)
        XCTAssertEqual(builder.snapshot().count, 1)
    }

    func testPointCloudFiltersToCylinderAndReportsSurfaceCoverage() throws {
        let builder = WorldPointCloudBuilder(
            voxelSize: 0.001,
            sampleStride: 1,
            maximumPointCount: 10,
            minimumLocalNeighborCount: 0,
            minimumVoxelNeighborCount: 0
        )
        let region = try XCTUnwrap(
            ScanCylinderRegion(
                lowerBoundary: SIMD3<Float>(0, -0.5, -1),
                upperBoundary: SIMD3<Float>(0, 0.5, -1),
                radius: 0.10,
                cameraPosition: .zero
            )
        )
        builder.reset(scanRegion: region)

        let result = builder.integrate(
            capturedFrame(
                depthValues: [.nan, 1, .nan],
                cameraTransform: matrix_identity_float4x4
            )
        )

        XCTAssertEqual(result.acceptedSampleCount, 1)
        XCTAssertEqual(result.rejectedOutsideRegionPointCount, 0)
        XCTAssertEqual(result.coverageCellSampleCounts.values.reduce(0, +), 1)
    }

    func testPointCloudRejectsDepthOutsideSelectedCylinder() throws {
        let builder = WorldPointCloudBuilder(
            voxelSize: 0.001,
            sampleStride: 1,
            maximumPointCount: 10,
            minimumLocalNeighborCount: 0,
            minimumVoxelNeighborCount: 0
        )
        let distantRegion = try XCTUnwrap(
            ScanCylinderRegion(
                lowerBoundary: SIMD3<Float>(1, -0.5, -1),
                upperBoundary: SIMD3<Float>(1, 0.5, -1),
                radius: 0.10,
                cameraPosition: .zero
            )
        )
        builder.reset(scanRegion: distantRegion)

        let result = builder.integrate(
            capturedFrame(
                depthValues: [.nan, 1, .nan],
                cameraTransform: matrix_identity_float4x4
            )
        )

        XCTAssertEqual(result.acceptedSampleCount, 0)
        XCTAssertEqual(result.rejectedOutsideRegionPointCount, 1)
        XCTAssertTrue(builder.snapshot().isEmpty)
    }

    func testTwoPointSelectionRequiresExplicitReset() {
        var selection = TwoPointSelection()
        selection.add(SIMD3<Float>(1, 0, 0))
        selection.add(SIMD3<Float>(2, 0, 0))
        selection.add(SIMD3<Float>(3, 0, 0))

        XCTAssertEqual(
            selection.points,
            [SIMD3<Float>(1, 0, 0), SIMD3<Float>(2, 0, 0)]
        )

        selection.reset()
        XCTAssertTrue(selection.points.isEmpty)
    }

    func testScanStateMachineCompletesHappyPathInOrder() {
        var machine = ScanStateMachine()

        XCTAssertEqual(machine.state, .ready)
        XCTAssertTrue(machine.send(.start))
        XCTAssertEqual(machine.state, .selectingObject)
        XCTAssertTrue(machine.send(.regionSelected))
        XCTAssertEqual(machine.state, .scanning)
        XCTAssertTrue(machine.send(.stop))
        XCTAssertEqual(machine.state, .processing)
        XCTAssertTrue(machine.send(.processingCompleted))
        XCTAssertEqual(machine.state, .reviewing)
    }

    func testScanStateMachineRejectsOutOfOrderTransition() {
        var machine = ScanStateMachine()

        XCTAssertFalse(machine.send(.stop))
        XCTAssertEqual(machine.state, .ready)
    }

    func testScanStateMachineCanFailRetryAndCancel() {
        var machine = ScanStateMachine()
        XCTAssertTrue(machine.send(.start))
        XCTAssertTrue(machine.send(.regionSelected))
        XCTAssertTrue(machine.send(.fail(reason: "Depth unavailable")))

        XCTAssertEqual(machine.state, .failed(reason: "Depth unavailable"))
        XCTAssertEqual(machine.state.failureReason, "Depth unavailable")
        XCTAssertTrue(machine.send(.retry))
        XCTAssertEqual(machine.state, .ready)

        XCTAssertTrue(machine.send(.start))
        XCTAssertTrue(machine.send(.cancel))
        XCTAssertEqual(machine.state, .ready)
    }

    func testReviewingScanCanStartAgain() {
        var machine = ScanStateMachine()
        XCTAssertTrue(machine.send(.start))
        XCTAssertTrue(machine.send(.regionSelected))
        XCTAssertTrue(machine.send(.stop))
        XCTAssertTrue(machine.send(.processingCompleted))

        XCTAssertTrue(machine.send(.start))
        XCTAssertEqual(machine.state, .selectingObject)
    }

    func testCylinderUsesTappedSurfaceToEstimateAxisBehindTheLeg() throws {
        let region = try XCTUnwrap(makeVerticalTestRegion(radius: 0.10))

        XCTAssertEqual(region.height, 1, accuracy: 0.001)
        XCTAssertEqual(region.center.x, 0, accuracy: 0.001)
        XCTAssertEqual(region.center.y, 0.5, accuracy: 0.001)
        XCTAssertEqual(region.center.z, -0.1, accuracy: 0.001)
        XCTAssertTrue(region.contains(SIMD3<Float>(0, 0.5, 0)))
        XCTAssertFalse(region.contains(SIMD3<Float>(0, 1.3, 0)))
        XCTAssertFalse(region.contains(SIMD3<Float>(0.5, 0.5, 0)))
    }

    func testCylinderRadiusIsAdjustableAndClamped() throws {
        let region = try XCTUnwrap(makeVerticalTestRegion(radius: 0.10))

        XCTAssertEqual(region.updatingRadius(0.18).radius, 0.18, accuracy: 0.001)
        XCTAssertEqual(
            region.updatingRadius(1).radius,
            ScanCylinderRegion.radiusRange.upperBound,
            accuracy: 0.001
        )
    }

    func testCoverageRequiresViewsAndSurfaceAcrossEveryHeightBand() throws {
        var coverage = ScanCoverageTracker(
            sectorCount: 4,
            verticalBandCount: 3,
            minimumSamplesPerCell: 1
        )
        let region = try XCTUnwrap(makeVerticalTestRegion(radius: 0.10))
        let center = region.center

        XCTAssertTrue(coverage.observe(cameraPosition: center + SIMD3<Float>(0, 0, 1), region: region))
        XCTAssertFalse(coverage.observe(cameraPosition: center + SIMD3<Float>(0, 0, 2), region: region))
        XCTAssertTrue(coverage.observe(cameraPosition: center + SIMD3<Float>(1, 0, 0), region: region))
        XCTAssertTrue(coverage.observe(cameraPosition: center + SIMD3<Float>(0, 0, -1), region: region))
        XCTAssertTrue(coverage.observe(cameraPosition: center + SIMD3<Float>(-1, 0, 0), region: region))
        XCTAssertFalse(coverage.isComplete, "An orbit alone must not claim surface coverage")

        var cells: [ScanCoverageCell: Int] = [:]
        for sector in 0..<4 {
            for band in 0..<3 {
                cells[ScanCoverageCell(angularSector: sector, verticalBand: band)] = 1
            }
        }
        XCTAssertTrue(coverage.observe(surfaceCells: cells))
        XCTAssertEqual(coverage.progress, 1, accuracy: 0.001)
        XCTAssertTrue(coverage.isComplete)
        XCTAssertEqual(coverage.remainingViewSectorCount, 0)
        XCTAssertEqual(coverage.remainingSurfaceCellCount, 0)
    }

    func testGuidancePrioritizesSpeedAndDistance() throws {
        let region = try XCTUnwrap(makeVerticalTestRegion(radius: 0.10))
        let coverage = ScanCoverageTracker()

        XCTAssertEqual(
            ScanGuidanceEvaluator.guidance(
                coverage: coverage,
                cameraPosition: region.center + SIMD3<Float>(0, 0, 0.2),
                region: region,
                movingTooFast: true
            ),
            .tooFast
        )
        XCTAssertEqual(
            ScanGuidanceEvaluator.guidance(
                coverage: coverage,
                cameraPosition: region.center + SIMD3<Float>(0, 0, 0.2),
                region: region,
                movingTooFast: false
            ),
            .increaseDistance
        )
        XCTAssertEqual(ScanGuidance.increaseDistance.text, "Mehr Abstand halten")
        XCTAssertEqual(ScanGuidance.moveToBack.text, "Bewege dich zur Rückseite")
        XCTAssertEqual(ScanGuidance.lowerAreaMissing.text, "Unterer Bereich fehlt")
    }

    func testRawDepthMapReportsUsableSampleFraction() {
        let map = RawDepthMap(
            width: 3,
            height: 2,
            values: [0.1, 0.2, 1, .nan, 6, 2]
        )

        XCTAssertEqual(map.validSampleFraction, 0.5, accuracy: 0.001)
    }

    func testRawFrameGateRequiresQualityAndUsefulMotion() {
        var gate = LiDARFrameCaptureGate(
            minimumTimeInterval: 0.1,
            maximumTimeInterval: 1,
            minimumTranslation: 0.05,
            minimumRotationRadians: 0.1,
            minimumValidDepthFraction: 0.5,
            minimumConfidentDepthFraction: 0.25
        )
        let origin = SIMD3<Float>(0, 0, 0)
        let identity = simd_quatf(angle: 0, axis: SIMD3<Float>(0, 1, 0))

        XCTAssertFalse(
            gate.shouldCapture(
                timestamp: 0,
                position: origin,
                rotation: identity,
                validDepthFraction: 0.4,
                confidentDepthFraction: 1
            )
        )
        XCTAssertTrue(
            gate.shouldCapture(
                timestamp: 0.1,
                position: origin,
                rotation: identity,
                validDepthFraction: 1,
                confidentDepthFraction: 1
            )
        )
        XCTAssertFalse(
            gate.shouldCapture(
                timestamp: 0.2,
                position: origin,
                rotation: identity,
                validDepthFraction: 1,
                confidentDepthFraction: 1
            )
        )
        XCTAssertTrue(
            gate.shouldCapture(
                timestamp: 0.3,
                position: SIMD3<Float>(0.06, 0, 0),
                rotation: identity,
                validDepthFraction: 1,
                confidentDepthFraction: 1
            )
        )
        XCTAssertTrue(
            gate.shouldCapture(
                timestamp: 0.5,
                position: SIMD3<Float>(0.06, 0, 0),
                rotation: simd_quatf(angle: 0.2, axis: SIMD3<Float>(0, 1, 0)),
                validDepthFraction: 1,
                confidentDepthFraction: 1
            )
        )
    }

    func testRawFrameGateRejectsExcessiveTranslationSpeed() {
        var gate = LiDARFrameCaptureGate(
            minimumTimeInterval: 0.1,
            maximumTimeInterval: 1,
            minimumTranslation: 0.01,
            minimumRotationRadians: 0.01,
            minimumValidDepthFraction: 0.5,
            minimumConfidentDepthFraction: 0.5,
            maximumTranslationSpeed: 0.5,
            maximumRotationSpeedRadians: 1
        )
        let identity = simd_quatf(angle: 0, axis: SIMD3<Float>(0, 1, 0))

        XCTAssertEqual(
            gate.evaluate(
                timestamp: 0,
                position: .zero,
                rotation: identity,
                validDepthFraction: 1,
                confidentDepthFraction: 1
            ),
            .rejectTooSoon
        )
        XCTAssertEqual(
            gate.evaluate(
                timestamp: 0.1,
                position: .zero,
                rotation: identity,
                validDepthFraction: 1,
                confidentDepthFraction: 1
            ),
            .capture
        )
        XCTAssertEqual(
            gate.evaluate(
                timestamp: 0.3,
                position: SIMD3<Float>(0.2, 0, 0),
                rotation: identity,
                validDepthFraction: 1,
                confidentDepthFraction: 1
            ),
            .rejectExcessiveMotion
        )
        XCTAssertEqual(
            gate.evaluate(
                timestamp: 0.5,
                position: SIMD3<Float>(0.21, 0, 0),
                rotation: identity,
                validDepthFraction: 1,
                confidentDepthFraction: 1
            ),
            .capture,
            "A later slow frame should recover after the fast frame was discarded"
        )
    }

    func testRawFrameGateRejectsExcessiveRotationSpeed() {
        var gate = LiDARFrameCaptureGate(
            minimumTimeInterval: 0.1,
            maximumTimeInterval: 1,
            minimumTranslation: 0.01,
            minimumRotationRadians: 0.01,
            minimumValidDepthFraction: 0.5,
            minimumConfidentDepthFraction: 0.5,
            maximumTranslationSpeed: 1,
            maximumRotationSpeedRadians: 1
        )
        let identity = simd_quatf(angle: 0, axis: SIMD3<Float>(0, 1, 0))

        XCTAssertEqual(
            gate.evaluate(
                timestamp: 0,
                position: .zero,
                rotation: identity,
                validDepthFraction: 1,
                confidentDepthFraction: 1
            ),
            .rejectTooSoon
        )
        XCTAssertEqual(
            gate.evaluate(
                timestamp: 0.1,
                position: .zero,
                rotation: identity,
                validDepthFraction: 1,
                confidentDepthFraction: 1
            ),
            .capture
        )
        XCTAssertEqual(
            gate.evaluate(
                timestamp: 0.3,
                position: .zero,
                rotation: simd_quatf(angle: 0.4, axis: SIMD3<Float>(0, 1, 0)),
                validDepthFraction: 1,
                confidentDepthFraction: 1
            ),
            .rejectExcessiveMotion
        )
    }

    func testScanStateTitlesMatchProductLanguage() {
        let states: [ScanState] = [
            .ready,
            .selectingObject,
            .scanning,
            .processing,
            .reviewing,
            .failed(reason: "Test")
        ]

        XCTAssertEqual(
            states.map(\.title),
            [
                "Ready",
                "Selecting object",
                "Scanning",
                "Processing",
                "Reviewing",
                "Failed"
            ]
        )
    }

    func testOperatorChecklistContainsAllScanningInstructions() {
        XCTAssertEqual(
            ARCameraView.scanningInstructions,
            [
                "Keep the leg stationary",
                "Maintain the correct distance",
                "Move slowly",
                "Circle the entire limb",
                "Avoid loose clothing and reflective surfaces"
            ]
        )
    }

    private func capturedFrame(
        depthValues: [Float32],
        width: Int = 3,
        height: Int = 1,
        confidenceValues: [UInt8]? = nil,
        includeConfidenceMap: Bool = true,
        focalLength: Float = 1,
        cameraTransform: simd_float4x4
    ) -> CapturedLiDARFrame {
        precondition(depthValues.count == width * height)
        let resolvedConfidenceValues = confidenceValues
            ?? Array(repeating: UInt8(2), count: depthValues.count)
        precondition(resolvedConfidenceValues.count == depthValues.count)
        let translation = cameraTransform.columns.3
        return CapturedLiDARFrame(
            depthMap: RawDepthMap(width: width, height: height, values: depthValues),
            confidenceMap: includeConfidenceMap
                ? RawDepthConfidenceMap(
                    width: width,
                    height: height,
                    values: resolvedConfidenceValues
                )
                : nil,
            cameraImageWidth: width,
            cameraImageHeight: height,
            cameraPosition: SIMD3<Float>(translation.x, translation.y, translation.z),
            cameraRotation: simd_quatf(real: 1, imag: .zero),
            cameraTransform: cameraTransform,
            cameraIntrinsics: simd_float3x3(
                SIMD3<Float>(focalLength, 0, 0),
                SIMD3<Float>(0, focalLength, 0),
                SIMD3<Float>(Float(width / 2), Float(height / 2), 1)
            ),
            timestamp: 0,
            rgbImage: nil
        )
    }

    private func makeVerticalTestRegion(radius: Float) -> ScanCylinderRegion? {
        ScanCylinderRegion(
            lowerBoundary: SIMD3<Float>(0, 0, 0),
            upperBoundary: SIMD3<Float>(0, 1, 0),
            radius: radius,
            cameraPosition: SIMD3<Float>(0, 0.5, 1)
        )
    }
}
