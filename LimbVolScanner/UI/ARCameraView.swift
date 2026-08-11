import ARKit
import AVFoundation
import SceneKit
import SwiftUI
import UIKit

struct DepthPointProjector {
    struct Intrinsics: Equatable {
        let fx: Float
        let fy: Float
        let cx: Float
        let cy: Float
    }

    static func scaledIntrinsics(
        _ cameraIntrinsics: simd_float3x3,
        imageWidth: Int,
        imageHeight: Int,
        depthWidth: Int,
        depthHeight: Int
    ) -> Intrinsics {
        let xScale = Float(depthWidth) / Float(imageWidth)
        let yScale = Float(depthHeight) / Float(imageHeight)
        return Intrinsics(
            fx: cameraIntrinsics.columns.0.x * xScale,
            fy: cameraIntrinsics.columns.1.y * yScale,
            cx: cameraIntrinsics.columns.2.x * xScale,
            cy: cameraIntrinsics.columns.2.y * yScale
        )
    }

    static func cameraSpacePoint(
        x: Int,
        y: Int,
        depth: Float,
        intrinsics: Intrinsics
    ) -> SIMD3<Float> {
        SIMD3<Float>(
            (Float(x) - intrinsics.cx) * depth / intrinsics.fx,
            -(Float(y) - intrinsics.cy) * depth / intrinsics.fy,
            -depth
        )
    }

    static func worldSpacePoint(
        cameraPoint: SIMD3<Float>,
        cameraTransform: simd_float4x4
    ) -> SIMD3<Float> {
        let world = cameraTransform * SIMD4<Float>(cameraPoint, 1)
        return SIMD3<Float>(world.x, world.y, world.z)
    }
}

struct TwoPointSelection {
    private(set) var points: [SIMD3<Float>] = []

    mutating func add(_ point: SIMD3<Float>) {
        guard points.count < 2 else { return }
        points.append(point)
    }

    mutating func reset() {
        points.removeAll(keepingCapacity: true)
    }
}

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
        view.session.delegate = context.coordinator
        context.coordinator.sceneView = view

        let statusLabel = UILabel()
        statusLabel.translatesAutoresizingMaskIntoConstraints = false
        statusLabel.text = "Ready\nMove slowly, then tap the first point"
        statusLabel.textColor = .white
        statusLabel.font = .preferredFont(forTextStyle: .callout)
        statusLabel.textAlignment = .center
        statusLabel.numberOfLines = 0
        statusLabel.backgroundColor = UIColor.black.withAlphaComponent(0.68)
        statusLabel.layer.cornerRadius = 14
        statusLabel.layer.masksToBounds = true
        view.addSubview(statusLabel)
        NSLayoutConstraint.activate([
            statusLabel.topAnchor.constraint(equalTo: view.safeAreaLayoutGuide.topAnchor, constant: 12),
            statusLabel.centerXAnchor.constraint(equalTo: view.centerXAnchor),
            statusLabel.widthAnchor.constraint(lessThanOrEqualTo: view.widthAnchor, multiplier: 0.86),
            statusLabel.heightAnchor.constraint(greaterThanOrEqualToConstant: 44)
        ])
        context.coordinator.statusLabel = statusLabel
        context.coordinator.showCurrentState()

        let tapRecognizer = UITapGestureRecognizer(
            target: context.coordinator,
            action: #selector(Coordinator.handleTap(_:))
        )
        view.addGestureRecognizer(tapRecognizer)
#if !targetEnvironment(simulator)
        context.coordinator.requestCameraAccessAndRun()
#endif
        return view
    }

    func updateUIView(_ uiView: ARSCNView, context: Context) {}

    static func dismantleUIView(_ uiView: ARSCNView, coordinator: Coordinator) {
        uiView.session.pause()
    }

    final class Coordinator: NSObject, ARSessionDelegate {
        weak var sceneView: ARSCNView?
        weak var statusLabel: UILabel?
        private(set) var selection = TwoPointSelection()
        private(set) var stateMachine = ScanStateMachine()
        private var markerNodes: [SCNNode] = []

        @objc func handleTap(_ recognizer: UITapGestureRecognizer) {
            guard recognizer.state == .ended, let sceneView else { return }
            guard acceptsRegionPoint else {
                showCurrentState()
                return
            }
            guard let frame = sceneView.session.currentFrame else {
                showCurrentState(detail: "AR is starting — try again in a moment")
                return
            }
            guard
                let worldPoint = depthWorldPoint(
                    at: recognizer.location(in: sceneView),
                    viewportSize: sceneView.bounds.size,
                    orientation: sceneView.window?.windowScene?.interfaceOrientation ?? .portrait,
                    frame: frame
            )
            else {
                showCurrentState(detail: "No LiDAR depth here — try another surface")
                UINotificationFeedbackGenerator().notificationOccurred(.warning)
                return
            }

            selection.add(worldPoint)
            addMarker(at: worldPoint)

            if selection.points.count == 1 {
                _ = stateMachine.send(.beginRegionSelection)
                showCurrentState(detail: "Tap the second point")
            } else {
                _ = stateMachine.send(.beginScanning)
                showCurrentState(detail: "Move slowly around the selected region")
            }
            UISelectionFeedbackGenerator().selectionChanged()
        }

        func requestCameraAccessAndRun() {
            guard LiDARSupport.isAvailable else { return }

            switch AVCaptureDevice.authorizationStatus(for: .video) {
            case .authorized:
                runSession()
            case .notDetermined:
                AVCaptureDevice.requestAccess(for: .video) { [weak self] granted in
                    DispatchQueue.main.async {
                        if granted {
                            self?.runSession()
                        } else {
                            self?.fail("Camera access was denied. Enable it in Settings.")
                        }
                    }
                }
            case .denied, .restricted:
                fail("Camera access is required. Enable it in Settings.")
            @unknown default:
                fail("Camera access could not be determined.")
            }
        }

        private func runSession() {
            guard let sceneView, LiDARSupport.isAvailable else { return }

            let configuration = ARWorldTrackingConfiguration()
            configuration.isAutoFocusEnabled = true
            if ARWorldTrackingConfiguration.supportsFrameSemantics(.smoothedSceneDepth) {
                configuration.frameSemantics = [.smoothedSceneDepth]
            } else {
                configuration.frameSemantics = [.sceneDepth]
            }
            sceneView.session.run(
                configuration,
                options: [.resetTracking, .removeExistingAnchors]
            )
            resetScan()
        }

        func session(_ session: ARSession, didFailWithError error: Error) {
            DispatchQueue.main.async { [weak self] in
                self?.fail(error.localizedDescription)
            }
        }

        func sessionWasInterrupted(_ session: ARSession) {
            DispatchQueue.main.async { [weak self] in
                self?.fail("The AR session was interrupted.")
            }
        }

        func sessionInterruptionEnded(_ session: ARSession) {
            DispatchQueue.main.async { [weak self] in
                self?.runSession()
            }
        }

        private func depthWorldPoint(
            at screenPoint: CGPoint,
            viewportSize: CGSize,
            orientation: UIInterfaceOrientation,
            frame: ARFrame
        ) -> SIMD3<Float>? {
            guard
                viewportSize.width > 0,
                viewportSize.height > 0,
                let sceneDepth = frame.smoothedSceneDepth ?? frame.sceneDepth
            else {
                return nil
            }

            let normalizedViewPoint = CGPoint(
                x: screenPoint.x / viewportSize.width,
                y: screenPoint.y / viewportSize.height
            )
            let normalizedImagePoint = normalizedViewPoint.applying(
                frame.displayTransform(
                    for: orientation,
                    viewportSize: viewportSize
                ).inverted()
            )
            guard
                (0...1).contains(normalizedImagePoint.x),
                (0...1).contains(normalizedImagePoint.y)
            else {
                return nil
            }

            let depthMap = sceneDepth.depthMap
            let confidenceMap = sceneDepth.confidenceMap
            let depthWidth = CVPixelBufferGetWidth(depthMap)
            let depthHeight = CVPixelBufferGetHeight(depthMap)
            guard depthWidth > 0, depthHeight > 0 else { return nil }

            let targetX = min(
                max(Int(normalizedImagePoint.x * CGFloat(depthWidth)), 0),
                depthWidth - 1
            )
            let targetY = min(
                max(Int(normalizedImagePoint.y * CGFloat(depthHeight)), 0),
                depthHeight - 1
            )

            CVPixelBufferLockBaseAddress(depthMap, .readOnly)
            if let confidenceMap {
                CVPixelBufferLockBaseAddress(confidenceMap, .readOnly)
            }
            defer {
                CVPixelBufferUnlockBaseAddress(depthMap, .readOnly)
                if let confidenceMap {
                    CVPixelBufferUnlockBaseAddress(confidenceMap, .readOnly)
                }
            }

            guard let sample = nearestDepthSample(
                targetX: targetX,
                targetY: targetY,
                depthMap: depthMap,
                confidenceMap: confidenceMap
            ) else {
                return nil
            }

            let intrinsics = DepthPointProjector.scaledIntrinsics(
                frame.camera.intrinsics,
                imageWidth: CVPixelBufferGetWidth(frame.capturedImage),
                imageHeight: CVPixelBufferGetHeight(frame.capturedImage),
                depthWidth: depthWidth,
                depthHeight: depthHeight
            )
            let cameraPoint = DepthPointProjector.cameraSpacePoint(
                x: sample.x,
                y: sample.y,
                depth: sample.depth,
                intrinsics: intrinsics
            )
            return DepthPointProjector.worldSpacePoint(
                cameraPoint: cameraPoint,
                cameraTransform: frame.camera.transform
            )
        }

        private func nearestDepthSample(
            targetX: Int,
            targetY: Int,
            depthMap: CVPixelBuffer,
            confidenceMap: CVPixelBuffer?
        ) -> (x: Int, y: Int, depth: Float)? {
            guard let depthBaseAddress = CVPixelBufferGetBaseAddress(depthMap) else {
                return nil
            }

            let width = CVPixelBufferGetWidth(depthMap)
            let height = CVPixelBufferGetHeight(depthMap)
            let depthBytesPerRow = CVPixelBufferGetBytesPerRow(depthMap)
            let confidenceBaseAddress = confidenceMap.flatMap(CVPixelBufferGetBaseAddress)
            let confidenceBytesPerRow = confidenceMap.map(CVPixelBufferGetBytesPerRow) ?? 0
            var bestConfidentSample: (x: Int, y: Int, depth: Float, squaredDistance: Int)?
            var bestAnySample: (x: Int, y: Int, depth: Float, squaredDistance: Int)?

            for y in max(0, targetY - 8)...min(height - 1, targetY + 8) {
                let depthRow = depthBaseAddress
                    .advanced(by: y * depthBytesPerRow)
                    .assumingMemoryBound(to: Float32.self)
                let confidenceRow = confidenceBaseAddress?
                    .advanced(by: y * confidenceBytesPerRow)
                    .assumingMemoryBound(to: UInt8.self)

                for x in max(0, targetX - 8)...min(width - 1, targetX + 8) {
                    let depth = depthRow[x]
                    guard depth.isFinite, depth >= 0.15, depth <= 5 else { continue }

                    let dx = x - targetX
                    let dy = y - targetY
                    let squaredDistance = dx * dx + dy * dy
                    if bestAnySample == nil || squaredDistance < bestAnySample!.squaredDistance {
                        bestAnySample = (x, y, depth, squaredDistance)
                    }

                    let isConfident = confidenceRow.map {
                        $0[x] >= UInt8(ARConfidenceLevel.medium.rawValue)
                    } ?? true
                    if isConfident,
                       bestConfidentSample == nil
                        || squaredDistance < bestConfidentSample!.squaredDistance {
                        bestConfidentSample = (x, y, depth, squaredDistance)
                    }
                }
            }

            let sample = bestConfidentSample ?? bestAnySample
            return sample.map { ($0.x, $0.y, $0.depth) }
        }

        private func addMarker(at position: SIMD3<Float>) {
            guard let sceneView else { return }

            let sphere = SCNSphere(radius: 0.008)
            sphere.firstMaterial?.diffuse.contents = UIColor.systemYellow
            sphere.firstMaterial?.emission.contents = UIColor.systemYellow
            sphere.firstMaterial?.lightingModel = .constant

            let node = SCNNode(geometry: sphere)
            node.simdPosition = position
            sceneView.scene.rootNode.addChildNode(node)
            markerNodes.append(node)
        }

        private func clearMarkers() {
            markerNodes.forEach { $0.removeFromParentNode() }
            markerNodes.removeAll()
        }

        private var acceptsRegionPoint: Bool {
            stateMachine.state == .ready || stateMachine.state == .selectingScanRegion
        }

        private func resetScan() {
            if stateMachine.state != .ready {
                _ = stateMachine.send(.reset)
            }
            selection.reset()
            clearMarkers()
            showCurrentState()
        }

        private func fail(_ reason: String) {
            guard stateMachine.send(.fail(reason: reason)) else { return }
            showCurrentState()
            UINotificationFeedbackGenerator().notificationOccurred(.error)
        }

        func showCurrentState(detail: String? = nil) {
            let state = stateMachine.state
            let message = detail ?? state.failureReason ?? defaultDetail(for: state)
            let text = "\(state.title)\n\(message)"
            DispatchQueue.main.async { [weak self] in
                self?.statusLabel?.text = "  \(text)  "
            }
        }

        private func defaultDetail(for state: ScanState) -> String {
            switch state {
            case .ready:
                "Move slowly, then tap the first point"
            case .selectingScanRegion:
                "Tap the second point"
            case .scanning:
                "Move slowly around the selected region"
            case .processing:
                "Building the scan result"
            case .reviewing:
                "Review the captured scan"
            case .finished:
                "Scan saved"
            case .failed:
                "The scan could not continue"
            }
        }
    }
}
