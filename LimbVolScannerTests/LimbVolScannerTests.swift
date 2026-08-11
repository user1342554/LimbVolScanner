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
        XCTAssertEqual(machine.state, .selectingScanRegion)
        XCTAssertTrue(machine.send(.regionSelected))
        XCTAssertEqual(machine.state, .scanning)
        XCTAssertTrue(machine.send(.stop))
        XCTAssertEqual(machine.state, .processing)
        XCTAssertTrue(machine.send(.processingCompleted))
        XCTAssertEqual(machine.state, .reviewing)
        XCTAssertTrue(machine.send(.reviewCompleted))
        XCTAssertEqual(machine.state, .finished)
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

    func testFinishedScanCanStartAgain() {
        var machine = ScanStateMachine()
        XCTAssertTrue(machine.send(.start))
        XCTAssertTrue(machine.send(.regionSelected))
        XCTAssertTrue(machine.send(.stop))
        XCTAssertTrue(machine.send(.processingCompleted))
        XCTAssertTrue(machine.send(.reviewCompleted))

        XCTAssertTrue(machine.send(.start))
        XCTAssertEqual(machine.state, .selectingScanRegion)
    }

    func testCoverageProgressUsesDistinctViewsAroundRegion() {
        var coverage = ScanCoverageTracker(sectorCount: 4)
        let center = SIMD3<Float>(0, 0, 0)

        XCTAssertTrue(coverage.observe(cameraPosition: SIMD3<Float>(0, 0, 1), regionCenter: center))
        XCTAssertFalse(coverage.observe(cameraPosition: SIMD3<Float>(0, 0, 2), regionCenter: center))
        XCTAssertEqual(coverage.progress, 0.25, accuracy: 0.001)

        XCTAssertTrue(coverage.observe(cameraPosition: SIMD3<Float>(1, 0, 0), regionCenter: center))
        XCTAssertTrue(coverage.observe(cameraPosition: SIMD3<Float>(0, 0, -1), regionCenter: center))
        XCTAssertTrue(coverage.observe(cameraPosition: SIMD3<Float>(-1, 0, 0), regionCenter: center))
        XCTAssertEqual(coverage.progress, 1, accuracy: 0.001)
        XCTAssertEqual(coverage.remainingSectorCount, 0)
    }

    func testScanStateTitlesMatchProductLanguage() {
        let states: [ScanState] = [
            .ready,
            .selectingScanRegion,
            .scanning,
            .processing,
            .reviewing,
            .finished,
            .failed(reason: "Test")
        ]

        XCTAssertEqual(
            states.map(\.title),
            [
                "Ready",
                "Selecting scan region",
                "Scanning",
                "Processing",
                "Reviewing",
                "Finished",
                "Failed"
            ]
        )
    }
}
