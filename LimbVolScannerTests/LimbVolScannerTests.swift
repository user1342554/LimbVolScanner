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

    func testThirdPointStartsANewTwoPointSelection() {
        var selection = TwoPointSelection()
        selection.add(SIMD3<Float>(1, 0, 0))
        selection.add(SIMD3<Float>(2, 0, 0))
        selection.add(SIMD3<Float>(3, 0, 0))

        XCTAssertEqual(selection.points, [SIMD3<Float>(3, 0, 0)])
    }
}
