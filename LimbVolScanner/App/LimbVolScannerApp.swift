import SwiftUI

@main
struct LimbVolScannerApp: App {
    var body: some Scene {
        WindowGroup {
            Group {
                if LiDARSupport.isAvailable {
                    ARCameraView()
                        .background(.black)
                } else {
                    UnsupportedLiDARView()
                }
            }
                .ignoresSafeArea()
        }
    }
}
