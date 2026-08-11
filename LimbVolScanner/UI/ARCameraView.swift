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
        if points.count == 2 {
            points.removeAll(keepingCapacity: true)
        }
        points.append(point)
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
        context.coordinator.sceneView = view

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

    final class Coordinator: NSObject {
        weak var sceneView: ARSCNView?
        private(set) var selection = TwoPointSelection()

        @objc func handleTap(_ recognizer: UITapGestureRecognizer) {
            guard
                recognizer.state == .ended,
                let sceneView,
                let frame = sceneView.session.currentFrame,
                let worldPoint = depthWorldPoint(
                    at: recognizer.location(in: sceneView),
                    viewportSize: sceneView.bounds.size,
                    orientation: sceneView.window?.windowScene?.interfaceOrientation ?? .portrait,
                    frame: frame
                )
            else {
                return
            }

            selection.add(worldPoint)
            UISelectionFeedbackGenerator().selectionChanged()
        }

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

        private func depthWorldPoint(
            at screenPoint: CGPoint,
            viewportSize: CGSize,
            orientation: UIInterfaceOrientation,
            frame: ARFrame
        ) -> SIMD3<Float>? {
            guard
                viewportSize.width > 0,
                viewportSize.height > 0,
                let sceneDepth = frame.sceneDepth
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
            var bestSample: (x: Int, y: Int, depth: Float, squaredDistance: Int)?

            for y in max(0, targetY - 4)...min(height - 1, targetY + 4) {
                let depthRow = depthBaseAddress
                    .advanced(by: y * depthBytesPerRow)
                    .assumingMemoryBound(to: Float32.self)
                let confidenceRow = confidenceBaseAddress?
                    .advanced(by: y * confidenceBytesPerRow)
                    .assumingMemoryBound(to: UInt8.self)

                for x in max(0, targetX - 4)...min(width - 1, targetX + 4) {
                    let depth = depthRow[x]
                    guard depth.isFinite, depth >= 0.15, depth <= 5 else { continue }
                    if let confidenceRow,
                       confidenceRow[x] < UInt8(ARConfidenceLevel.medium.rawValue) {
                        continue
                    }

                    let dx = x - targetX
                    let dy = y - targetY
                    let squaredDistance = dx * dx + dy * dy
                    if bestSample == nil || squaredDistance < bestSample!.squaredDistance {
                        bestSample = (x, y, depth, squaredDistance)
                    }
                }
            }

            return bestSample.map { ($0.x, $0.y, $0.depth) }
        }
    }
}
