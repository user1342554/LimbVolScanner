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

        let scanGuide = UIView()
        scanGuide.translatesAutoresizingMaskIntoConstraints = false
        scanGuide.isUserInteractionEnabled = false
        scanGuide.layer.borderWidth = 2
        scanGuide.layer.borderColor = UIColor.systemYellow.cgColor
        scanGuide.layer.cornerRadius = 24
        scanGuide.backgroundColor = UIColor.systemYellow.withAlphaComponent(0.04)
        scanGuide.isHidden = true

        let guideLabel = UILabel()
        guideLabel.translatesAutoresizingMaskIntoConstraints = false
        guideLabel.text = "KEEP THE YELLOW REGION INSIDE"
        guideLabel.textColor = .systemYellow
        guideLabel.font = .systemFont(ofSize: 12, weight: .bold)
        guideLabel.textAlignment = .center
        guideLabel.backgroundColor = UIColor.black.withAlphaComponent(0.62)
        guideLabel.layer.cornerRadius = 9
        guideLabel.layer.masksToBounds = true
        scanGuide.addSubview(guideLabel)

        let centerDot = UIView()
        centerDot.translatesAutoresizingMaskIntoConstraints = false
        centerDot.backgroundColor = .systemYellow
        centerDot.layer.cornerRadius = 4
        scanGuide.addSubview(centerDot)
        view.addSubview(scanGuide)
        NSLayoutConstraint.activate([
            scanGuide.centerXAnchor.constraint(equalTo: view.centerXAnchor),
            scanGuide.centerYAnchor.constraint(equalTo: view.centerYAnchor, constant: -24),
            scanGuide.widthAnchor.constraint(equalTo: view.widthAnchor, multiplier: 0.66),
            scanGuide.heightAnchor.constraint(equalTo: scanGuide.widthAnchor),
            guideLabel.centerXAnchor.constraint(equalTo: scanGuide.centerXAnchor),
            guideLabel.bottomAnchor.constraint(equalTo: scanGuide.bottomAnchor, constant: -10),
            guideLabel.heightAnchor.constraint(equalToConstant: 28),
            guideLabel.widthAnchor.constraint(lessThanOrEqualTo: scanGuide.widthAnchor, constant: -20),
            centerDot.centerXAnchor.constraint(equalTo: scanGuide.centerXAnchor),
            centerDot.centerYAnchor.constraint(equalTo: scanGuide.centerYAnchor),
            centerDot.widthAnchor.constraint(equalToConstant: 8),
            centerDot.heightAnchor.constraint(equalToConstant: 8)
        ])
        context.coordinator.scanGuide = scanGuide

        let progressPanel = UIStackView()
        progressPanel.translatesAutoresizingMaskIntoConstraints = false
        progressPanel.axis = .vertical
        progressPanel.spacing = 7
        progressPanel.isLayoutMarginsRelativeArrangement = true
        progressPanel.layoutMargins = UIEdgeInsets(top: 10, left: 14, bottom: 11, right: 14)
        progressPanel.backgroundColor = UIColor.black.withAlphaComponent(0.72)
        progressPanel.layer.cornerRadius = 14
        progressPanel.isUserInteractionEnabled = false

        let progressLabel = UILabel()
        progressLabel.textColor = .white
        progressLabel.font = .systemFont(ofSize: 13, weight: .semibold)
        progressLabel.text = "Scan progress 0%"
        progressPanel.addArrangedSubview(progressLabel)

        let progressView = UIProgressView(progressViewStyle: .default)
        progressView.progressTintColor = .systemYellow
        progressView.trackTintColor = UIColor.white.withAlphaComponent(0.24)
        progressView.progress = 0
        progressPanel.addArrangedSubview(progressView)
        view.addSubview(progressPanel)
        context.coordinator.progressLabel = progressLabel
        context.coordinator.progressView = progressView

        func makeButton(
            title: String,
            color: UIColor,
            action: Selector
        ) -> UIButton {
            var configuration = UIButton.Configuration.filled()
            configuration.title = title
            configuration.baseBackgroundColor = color
            configuration.baseForegroundColor = .white
            configuration.cornerStyle = .large
            let button = UIButton(configuration: configuration)
            button.addTarget(context.coordinator, action: action, for: .touchUpInside)
            return button
        }

        let startButton = makeButton(
            title: "Start",
            color: .systemGreen,
            action: #selector(Coordinator.startTapped)
        )
        let stopButton = makeButton(
            title: "Stop",
            color: .systemOrange,
            action: #selector(Coordinator.stopTapped)
        )
        let retryButton = makeButton(
            title: "Retry",
            color: .systemBlue,
            action: #selector(Coordinator.retryTapped)
        )
        let cancelButton = makeButton(
            title: "Cancel",
            color: .systemRed,
            action: #selector(Coordinator.cancelTapped)
        )

        let buttonStack = UIStackView(arrangedSubviews: [
            startButton,
            stopButton,
            retryButton,
            cancelButton
        ])
        buttonStack.translatesAutoresizingMaskIntoConstraints = false
        buttonStack.axis = .horizontal
        buttonStack.distribution = .fillEqually
        buttonStack.spacing = 8
        view.addSubview(buttonStack)
        NSLayoutConstraint.activate([
            buttonStack.leadingAnchor.constraint(equalTo: view.safeAreaLayoutGuide.leadingAnchor, constant: 12),
            buttonStack.trailingAnchor.constraint(equalTo: view.safeAreaLayoutGuide.trailingAnchor, constant: -12),
            buttonStack.bottomAnchor.constraint(equalTo: view.safeAreaLayoutGuide.bottomAnchor, constant: -12),
            buttonStack.heightAnchor.constraint(equalToConstant: 48),
            progressPanel.leadingAnchor.constraint(equalTo: buttonStack.leadingAnchor),
            progressPanel.trailingAnchor.constraint(equalTo: buttonStack.trailingAnchor),
            progressPanel.bottomAnchor.constraint(equalTo: buttonStack.topAnchor, constant: -10)
        ])
        context.coordinator.startButton = startButton
        context.coordinator.stopButton = stopButton
        context.coordinator.retryButton = retryButton
        context.coordinator.cancelButton = cancelButton
        context.coordinator.showCurrentState()

        let tapRecognizer = UITapGestureRecognizer(
            target: context.coordinator,
            action: #selector(Coordinator.handleTap(_:))
        )
        tapRecognizer.delegate = context.coordinator
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

    final class Coordinator: NSObject, ARSessionDelegate, UIGestureRecognizerDelegate {
        weak var sceneView: ARSCNView?
        weak var statusLabel: UILabel?
        weak var scanGuide: UIView?
        weak var progressLabel: UILabel?
        weak var progressView: UIProgressView?
        weak var startButton: UIButton?
        weak var stopButton: UIButton?
        weak var retryButton: UIButton?
        weak var cancelButton: UIButton?
        private(set) var selection = TwoPointSelection()
        private(set) var stateMachine = ScanStateMachine()
        private var markerNodes: [SCNNode] = []
        private var coverage = ScanCoverageTracker()
        private var regionCenter: SIMD3<Float>?
        private var regionDistanceText: String?
        private var lastCoverageTimestamp: TimeInterval = 0
        private var pendingTransition: DispatchWorkItem?

        @objc func startTapped() {
            guard stateMachine.send(.start) else { return }
            prepareNewScan()
            showCurrentState()
        }

        @objc func stopTapped() {
            finishCapture()
        }

        @objc func retryTapped() {
            guard stateMachine.send(.retry) else { return }
            prepareNewScan()
            showCurrentState(detail: "Ready to try again — tap Start")
            requestCameraAccessAndRun()
        }

        @objc func cancelTapped() {
            guard stateMachine.send(.cancel) else { return }
            prepareNewScan()
            showCurrentState(detail: "Scan cancelled — tap Start when ready")
        }

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
                showCurrentState(detail: "First point set — tap the opposite edge")
            } else {
                let firstPoint = selection.points[0]
                let secondPoint = selection.points[1]
                regionCenter = (firstPoint + secondPoint) / 2
                let distance = simd_distance(firstPoint, secondPoint)
                regionDistanceText = formattedDistance(distance)
                addConnectingLine(from: firstPoint, to: secondPoint)
                coverage.reset()
                updateProgressDisplay(0, animated: false)
                _ = stateMachine.send(.regionSelected)
                showCurrentState()
            }
            UISelectionFeedbackGenerator().selectionChanged()
        }

        func gestureRecognizer(
            _ gestureRecognizer: UIGestureRecognizer,
            shouldReceive touch: UITouch
        ) -> Bool {
            var touchedView = touch.view
            while let currentView = touchedView {
                if currentView is UIControl { return false }
                touchedView = currentView.superview
            }
            return true
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

        func session(_ session: ARSession, didUpdate frame: ARFrame) {
            guard frame.timestamp - lastCoverageTimestamp >= 0.15 else { return }
            lastCoverageTimestamp = frame.timestamp
            let cameraTranslation = frame.camera.transform.columns.3
            let cameraPosition = SIMD3<Float>(
                cameraTranslation.x,
                cameraTranslation.y,
                cameraTranslation.z
            )

            DispatchQueue.main.async { [weak self] in
                self?.recordCoverage(cameraPosition: cameraPosition)
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

        private func addConnectingLine(
            from start: SIMD3<Float>,
            to end: SIMD3<Float>
        ) {
            guard let sceneView else { return }

            let vector = end - start
            let length = simd_length(vector)
            guard length > 0.001 else { return }

            let cylinder = SCNCylinder(radius: 0.0035, height: CGFloat(length))
            cylinder.firstMaterial?.diffuse.contents = UIColor.systemYellow
            cylinder.firstMaterial?.emission.contents = UIColor.systemYellow
            cylinder.firstMaterial?.lightingModel = .constant

            let node = SCNNode(geometry: cylinder)
            node.simdPosition = (start + end) / 2
            node.simdOrientation = simd_quatf(
                from: SIMD3<Float>(0, 1, 0),
                to: simd_normalize(vector)
            )
            sceneView.scene.rootNode.addChildNode(node)
            markerNodes.append(node)
        }

        private func clearMarkers() {
            markerNodes.forEach { $0.removeFromParentNode() }
            markerNodes.removeAll()
        }

        private var acceptsRegionPoint: Bool {
            stateMachine.state == .selectingScanRegion
        }

        private func resetScan() {
            _ = stateMachine.send(.cancel)
            prepareNewScan()
            showCurrentState()
        }

        private func prepareNewScan() {
            pendingTransition?.cancel()
            pendingTransition = nil
            selection.reset()
            coverage.reset()
            regionCenter = nil
            regionDistanceText = nil
            clearMarkers()
            updateProgressDisplay(0, animated: false)
        }

        private func recordCoverage(cameraPosition: SIMD3<Float>) {
            guard stateMachine.state == .scanning, let regionCenter else { return }
            guard coverage.observe(
                cameraPosition: cameraPosition,
                regionCenter: regionCenter
            ) else { return }

            updateProgressDisplay(coverage.progress, animated: true)
            showCurrentState()
            UISelectionFeedbackGenerator().selectionChanged()

            if coverage.progress >= 1 {
                finishCapture()
            }
        }

        private func finishCapture() {
            guard stateMachine.send(.stop) else { return }
            updateProgressDisplay(1, animated: true)
            showCurrentState()

            let processing = DispatchWorkItem { [weak self] in
                guard let self, self.stateMachine.send(.processingCompleted) else { return }
                self.showCurrentState()

                let reviewing = DispatchWorkItem { [weak self] in
                    guard let self, self.stateMachine.send(.reviewCompleted) else { return }
                    self.updateProgressDisplay(1, animated: true)
                    self.showCurrentState()
                    UINotificationFeedbackGenerator().notificationOccurred(.success)
                }
                self.pendingTransition = reviewing
                DispatchQueue.main.asyncAfter(deadline: .now() + 1.0, execute: reviewing)
            }
            pendingTransition = processing
            DispatchQueue.main.asyncAfter(deadline: .now() + 0.8, execute: processing)
        }

        private func fail(_ reason: String) {
            pendingTransition?.cancel()
            pendingTransition = nil
            guard stateMachine.send(.fail(reason: reason)) else { return }
            showCurrentState()
            UINotificationFeedbackGenerator().notificationOccurred(.error)
        }

        func showCurrentState(detail: String? = nil) {
            let state = stateMachine.state
            let message = detail ?? state.failureReason ?? defaultDetail(for: state)
            let text = "\(state.title)\n\(message)"
            DispatchQueue.main.async { [weak self] in
                guard let self else { return }
                self.statusLabel?.text = "  \(text)  "
                self.updateControls(for: state)
                self.updateProgressLabel(for: state)
            }
        }

        private func updateControls(for state: ScanState) {
            setButton(startButton, enabled: state == .ready || state == .finished)
            setButton(stopButton, enabled: state == .scanning)
            setButton(retryButton, enabled: state.failureReason != nil)
            setButton(
                cancelButton,
                enabled: state == .selectingScanRegion
                    || state == .scanning
                    || state == .processing
                    || state == .reviewing
                    || state.failureReason != nil
            )
            scanGuide?.isHidden = state != .selectingScanRegion && state != .scanning
        }

        private func setButton(_ button: UIButton?, enabled: Bool) {
            button?.isEnabled = enabled
            button?.alpha = enabled ? 1 : 0.34
        }

        private func updateProgressDisplay(_ progress: Float, animated: Bool) {
            progressView?.setProgress(progress, animated: animated)
        }

        private func updateProgressLabel(for state: ScanState) {
            let percent = Int((progressView?.progress ?? 0) * 100)
            switch state {
            case .ready:
                progressLabel?.text = "Scan progress 0% • Tap Start"
            case .selectingScanRegion:
                progressLabel?.text = "Scan progress 0% • Select two yellow points"
            case .scanning:
                progressLabel?.text = "Scan progress \(percent)% • \(coverage.remainingSectorCount) views left"
            case .processing:
                progressLabel?.text = "Scan complete • Processing 100%"
            case .reviewing:
                progressLabel?.text = "Scan complete • Reviewing"
            case .finished:
                progressLabel?.text = "Scan progress 100% • Finished"
            case .failed:
                progressLabel?.text = "Scan failed at \(percent)% • Retry or Cancel"
            }
        }

        private func defaultDetail(for state: ScanState) -> String {
            switch state {
            case .ready:
                "Tap Start to select the scan region"
            case .selectingScanRegion:
                selection.points.isEmpty
                    ? "Keep the object in the yellow guide, then tap its first edge"
                    : "First point set — tap the opposite edge"
            case .scanning:
                "\(regionDistanceText ?? "Region selected") • Walk around it and keep it inside the guide"
            case .processing:
                "Scanning stopped — building the result"
            case .reviewing:
                "Checking the captured scan"
            case .finished:
                "Scanning finished — tap Start for another scan"
            case .failed:
                "The scan could not continue"
            }
        }

        private func formattedDistance(_ distanceInMetres: Float) -> String {
            if distanceInMetres < 1 {
                return String(format: "%.1f cm region", distanceInMetres * 100)
            }
            return String(format: "%.2f m region", distanceInMetres)
        }
    }
}
