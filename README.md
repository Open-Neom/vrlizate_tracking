# vrlizate_tracking

Multi-camera 6DoF optical fiducial PnP tracking, **Estación Faro** 3-device spatial coordinator (Desktop/PC + Smartphone Visor + Smartphone Joystick), optical passthrough feeds, camera calibration, tri-camera spatial inspector HUDs, and `VrDeviceProfiler` for **VRlizate SDK**.

## Features

- **Clean-Room Optical Fiducial & PnP Pose (`VrVisualMarkerPoseSolver`, `VrMarkerOrientationResolver`, `VrVisualTrackingSession`):** Homography + iterative Gauss-Newton PnP refinement, Savitzky-Golay velocity estimation, Huber IRLS multi-view triangulation, and One-Euro filtering in pure Dart.
- **Estación Faro (`VrDesktopBeaconStation`, `VrDesktopBeaconClient`, `VrTriNodeDepthSolver`):** Coordinates a personal computer as an optional 3rd spatial node and LAN relay over authenticated `VRB1` UDP (`dart:io`).
- **Optical Passthrough & Tri-Camera Inspector (`VrPassthroughCameraFeed`, `VrSpatialTrackingService`, `VrTriCameraLiveInspectorCard`, `VrStereoSpatialInspectorHud`):** Dual-lens / tri-camera optical curtain, calibration, and stereoscopic diagnostic HUDs.
- **Device Capability Profiling (`VrDeviceProfiler`):** Catalog-backed quality tier selection and optional Android refresh-rate / Wi-Fi band inspection.

## Platform, Native Channel & Sensing Contract

`vrlizate_tracking` does not bundle native ARCore, ARKit, Apple Vision, or acoustic sonar engines inside `android/` or `ios/` folders. Integrators should distinguish pure-Dart algorithms, standard/bundled camera plugins, host-supplied native channels, and simulation generators:

| Subsystem | SDK Entry Points | Runtime / Plugin Contract | Behavior in a Standalone SDK Host |
| --- | --- | --- | --- |
| **Optical PnP & Multi-View Math** | `VrVisualMarkerPoseSolver`, `VrFiducialTagCodec`, `VrTriNodeDepthSolver`, `VrEMudraEngine` | Pure Dart | Runs in any Dart/Flutter isolate when fed calibrated intrinsics and marker/keypoint observations. |
| **Estación Faro LAN Transport** | `VrDesktopBeaconStation`, `VrDesktopBeaconClient` | `dart:io` (`RawDatagramSocket`, `HttpServer`) | Supported on Dart VM / native targets (Android, iOS, macOS, Windows, Linux); **not** available on Flutter Web. |
| **Android Camera Frame Budget** | `camera` + `camera_android_camerax` (`0.7.4+8` bundled in SDK `dependencyPackages`) | `vrlizate/camera_analysis_budget` | Bundled in the SDK distribution for Android camera stream admission control. |
| **Native 6DoF VIO / SLAM Bridge** | `VrSpatialTrackingService` (`vrlizate/spatial_tracking` + `vrlizate/spatial_tracking/poses`) | **Host-supplied** native ARCore/ARKit bridge or custom `VrSpatialTrackingPlatform` | Catches `MissingPluginException`, cancels stale `EventChannel` epochs, and reports `'unavailable'`. Live camera heuristics without calibrated extrinsics are **not** promoted into physical 6DoF room poses. |
| **Ultrasonic TW-ToF Ranging** | `VrSpatialTrackingService` (`vrlizate/ultrasonic_sonar`) | **Host-supplied** native audio chirp I/O | Marks sonar unavailable on `MissingPluginException` / 2 s timeout and continues without acoustic ranging. |
| **Desktop Webcam Pose & Voice** | `MethodChannelVrDesktopVisionPlatform` (`vrlizate/desktop_beacon_vision` + `/frames`), `MethodChannelVrDesktopVoicePlatform` (`vrlizate/desktop_beacon_voice` + `/events`) | **Host-supplied** desktop runner (e.g. macOS `AVCaptureSession` + Apple `Vision` / `SFSpeechRecognizer`) or injected platform interface | `isSupported()` returns `false` on `MissingPluginException` / 2 s timeout. |
| **Device Capability Inspection** | `VrDeviceProfiler` (`vrlizate/device_capabilities`) | Optional Android host `MethodChannel` | Falls back to conservative `medium` tier with `status: 'missing_plugin'` after 900 ms timeout or `MissingPluginException`. |
| **Synthetic / Diagnostic Generators** | `VrSyntheticDesktopVisionGenerator`, `VrSpatialTrackingService` simulation helpers | Pure Dart deterministic test/UI fixtures | Explicitly intended for headless testing, inspector previews, and UI exercises — **never** a substitute for physical camera/SLAM measurements or hardware FPS certification. |

## License

Apache License 2.0 — see `LICENSE`.
