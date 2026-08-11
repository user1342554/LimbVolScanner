import ARKit
import AVFoundation
import SceneKit
import SwiftUI

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
            guard let sceneView, ARWorldTrackingConfiguration.isSupported else { return }

            let configuration = ARWorldTrackingConfiguration()
            configuration.isAutoFocusEnabled = true
            sceneView.session.run(
                configuration,
                options: [.resetTracking, .removeExistingAnchors]
            )
        }
    }
}
