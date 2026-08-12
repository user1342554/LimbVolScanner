# LimbVolScanner

LimbVolScanner is a native iOS 17 ARKit app for guided LiDAR limb capture. It requires a LiDAR-capable iPhone or iPad; unsupported devices are blocked before the AR camera opens.

## Scan workflow

1. Tap **Start scan**.
2. Tap the lower boundary of the limb (for example, the ankle).
3. Tap the upper boundary (for example, the knee).
4. Adjust the radius of the yellow 3D cylinder until it contains the limb, then tap **Begin scanning**.
5. Move slowly around the stationary limb. The cyan world-space point cloud shows the surface already captured.
6. Follow the live guidance. A scan finishes automatically only after the camera has completed the orbit and LiDAR surface samples cover every angular sector in the lower, middle, and upper cylinder bands.

The visible states are **Ready**, **Selecting object**, **Scanning**, **Processing**, and **Reviewing**. During capture the app retains depth, confidence, camera pose, selected RGB keyframes, and the latest geometry for each ARKit mesh anchor.

## Validation

The Xcode test target covers tap/depth projection, shared-world point fusion, noise filtering, scan-state transitions, cylinder geometry, region filtering, coverage completion, guidance priority, and excessive-motion rejection.

The GitHub Actions workflow runs the tests on an iOS Simulator, builds an unsigned ARM64 iPhone app, and packages an unsigned IPA. A successful CI build establishes compilation and packaging only; scan quality and automatic 360-degree completion still need validation on a supported physical LiDAR device.
