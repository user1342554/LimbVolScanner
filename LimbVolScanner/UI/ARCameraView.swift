import ARKit
import AVFoundation
import SceneKit
import SwiftUI

enum LiDARSupport {
    static var isAvailable: Bool {
        ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth)
    }
}

struct UnsupportedLiDARView: View {
    var body: some View {
        ZStack {
            Color.black

            VStack(spacing: 16) {
                Image(systemName: "viewfinder")
                    .font(.system(size: 48, weight: .light))

                Text("LiDAR Not Supported")
                    .font(.title2.bold())

                Text("This app requires an iPhone or iPad with ARKit scene depth.")
                    .font(.body)
                    .foregroundStyle(.secondary)
                    .multilineTextAlignment(.center)
                    .frame(maxWidth: 320)
            }
            .foregroundStyle(.white)
            .padding(32)
        }
    }
}

struct ARCameraView: UIViewRepresentable {
    func makeCoordinator() -> Coordinator {
        Coordinator()
    }

    func makeUIView(context: Context) -> ARSCNView {
        let view = ARSCNView(frame: .zero)
        view.backgroundColor = .black
        view.scene = SCNScene()
        view.automaticallyUpdatesLighting = true
        context.coordinator.sceneView = view
#if !targetEnvironment(simulator)
        context.coordinator.requestCameraAccessAndRun()
#endif
        return view
    }

    func updateUIView(_ uiView: ARSCNView, context: Context) {}

    static func dismantleUIView(_ uiView: ARSCNView, coordinator: Coordinator) {
        uiView.session.pause()
    }

    final class Coordinator {
        weak var sceneView: ARSCNView?

        func requestCameraAccessAndRun() {
            guard LiDARSupport.isAvailable else { return }

            switch AVCaptureDevice.authorizationStatus(for: .video) {
            case .authorized:
                runSession()
            case .notDetermined:
                AVCaptureDevice.requestAccess(for: .video) { [weak self] granted in
                    guard granted else { return }
                    DispatchQueue.main.async {
                        self?.runSession()
                    }
                }
            case .denied, .restricted:
                break
            @unknown default:
                break
            }
        }

        private func runSession() {
            guard let sceneView, LiDARSupport.isAvailable else { return }

            let configuration = ARWorldTrackingConfiguration()
            configuration.isAutoFocusEnabled = true
            configuration.frameSemantics = [.sceneDepth]
            sceneView.session.run(
                configuration,
                options: [.resetTracking, .removeExistingAnchors]
            )
        }
    }
}
