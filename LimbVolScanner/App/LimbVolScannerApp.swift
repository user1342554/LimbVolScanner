import SwiftUI

@main
struct LimbVolScannerApp: App {
    var body: some Scene {
        WindowGroup {
            ARCameraView()
                .background(.black)
                .ignoresSafeArea()
        }
    }
}
