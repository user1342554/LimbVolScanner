import XCTest
@testable import LimbVolScanner

final class LimbVolScannerTests: XCTestCase {
    func testSimulatorReportsLiDARAsUnsupported() {
#if targetEnvironment(simulator)
        XCTAssertFalse(LiDARSupport.isAvailable)
#endif
    }
}
