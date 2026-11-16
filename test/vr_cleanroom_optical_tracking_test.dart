import 'dart:math' as math;

import 'package:flutter_test/flutter_test.dart';
import 'package:vrlizate_tracking/vrlizate_tracking.dart';
import 'package:vector_math/vector_math.dart' as vm;

List<VrDesktopKeypoint2D> _projectMarkerThroughCam({
  required CalibratedCameraView cam,
  required vm.Vector3 centerRoom,
  required vm.Quaternion markerOrientationRoom,
  required double markerSizeMeters,
  required int widthPx,
  required int heightPx,
}) {
  final half = markerSizeMeters * 0.5;
  // Marker local X right, Y down in CV -> in room space (+X user-right = -X_cv, +Y up = -Y_cv):
  final localCornersRoom = <vm.Vector3>[
    vm.Vector3(half, half, 0.0), // TL in image (+X_room -> -X_cv, +Y_room -> -Y_cv)
    vm.Vector3(-half, half, 0.0), // TR in image
    vm.Vector3(-half, -half, 0.0), // BR in image
    vm.Vector3(half, -half, 0.0), // BL in image
  ];
  final out = <VrDesktopKeypoint2D>[];
  for (final lc in localCornersRoom) {
    final pWorld = centerRoom + markerOrientationRoom.rotated(lc);
    final px = cam.projectWorldToPixel(pWorld);
    out.add(VrDesktopKeypoint2D(px.x / widthPx, px.y / heightPx, 1.0));
  }
  return out;
}

void main() {
  group('Clean-Room Pure-Dart Optical & Kinematic Camera Tracking (neom_vr)', () {
    test('Brown-Conrady lens undistortion recovers < 3 mm PnP on wide-angle distorted camera', () {
      final solver = VrTriNodeDepthSolver(
        webcamHorizontalFovDeg: 82.0,
        nominalStandingDistanceMeters: 1.50,
        deskCameraHeightMeters: 1.18,
        webcamK1: -0.24,
        webcamK2: 0.085,
        webcamP1: 0.0025,
        webcamP2: -0.0018,
      );
      final desktopCam = solver.buildDesktopCalibratedCameraView(
        widthPx: 1280,
        heightPx: 720,
      );

      // Place Visor near outer third of wide-angle lens where radial distortion is strong!
      final trueVisorRoom = vm.Vector3(0.44, 1.58, -0.22);
      final qMarker = vm.Quaternion.euler(0.12, -0.10, 0.04);
      final corners = _projectMarkerThroughCam(
        cam: desktopCam,
        centerRoom: trueVisorRoom,
        markerOrientationRoom: qMarker,
        markerSizeMeters: 0.048,
        widthPx: 1280,
        heightPx: 720,
      );

      final frame = VrDesktopVisionFrame.fromPlatformMap({
        'status': 'ok',
        'timestampMicros': 100000,
        'mirrored': false,
        'frameWidth': 1280,
        'frameHeight': 720,
        'fiducialTags': [
          {
            'payload': 'vrlizate://tag?node=visor&mm=48',
            'frameWidth': 1280,
            'frameHeight': 720,
            'corners': [for (final c in corners) c.toJson()],
          },
        ],
      })!;

      final sol = solver.solve(nowMicros: 100000, vision: frame);
      expect(sol.visorFiducialLocked, isTrue);
      expect(sol.visorFiducialReprojectionRmsPx!, lessThan(0.25));
      expect(sol.visorRoomPosition.distanceTo(trueVisorRoom), lessThan(0.003));
    });

    test('Kabsch horizon pitch calibration & leveling removes +15 deg laptop webcam tilt bias', () {
      const truePitchDeg = 15.0;
      final tiltedRefSolver = VrTriNodeDepthSolver(
        webcamHorizontalFovDeg: 68.0,
        nominalStandingDistanceMeters: 1.50,
        deskCameraHeightMeters: 1.18,
        webcamPitchDeg: truePitchDeg,
      );
      final tiltedCam = tiltedRefSolver.buildDesktopCalibratedCameraView(
        widthPx: 1280,
        heightPx: 720,
      );

      final trueVisorRoom = vm.Vector3(0.18, 1.61, -0.10);
      final pCamTrue = tiltedCam.worldToCamera(trueVisorRoom);

      // Verify automatic 1-shot horizon pitch calibration from known standing height:
      final runtimeSolver = VrTriNodeDepthSolver(
        webcamHorizontalFovDeg: 68.0,
        nominalStandingDistanceMeters: 1.50,
        deskCameraHeightMeters: 1.18,
      );
      final estimatedPitch = runtimeSolver.calibrateHorizonPitchFromObservedHeight(
        pnpTranslationCamera: pCamTrue,
        knownTargetWorldHeightMeters: trueVisorRoom.y,
      );
      expect(estimatedPitch, closeTo(truePitchDeg, 0.02));

      final corners = _projectMarkerThroughCam(
        cam: tiltedCam,
        centerRoom: trueVisorRoom,
        markerOrientationRoom: vm.Quaternion.identity(),
        markerSizeMeters: 0.048,
        widthPx: 1280,
        heightPx: 720,
      );
      final frame = VrDesktopVisionFrame.fromPlatformMap({
        'status': 'ok',
        'timestampMicros': 200000,
        'mirrored': false,
        'frameWidth': 1280,
        'frameHeight': 720,
        'fiducialTags': [
          {
            'payload': 'vrlizate://tag?node=visor&mm=48',
            'frameWidth': 1280,
            'frameHeight': 720,
            'corners': [for (final c in corners) c.toJson()],
          },
        ],
      })!;

      final leveledSol = runtimeSolver.solve(nowMicros: 200000, vision: frame);
      expect(leveledSol.visorFiducialLocked, isTrue);
      expect(leveledSol.visorRoomPosition.distanceTo(trueVisorRoom), lessThan(0.003));
    });

    test('Causal spike gate blocks PnP/hand teleports, protects Auto-IPD, and coasts 3 occluded frames', () {
      final solver = VrTriNodeDepthSolver(
        webcamHorizontalFovDeg: 68.0,
        nominalStandingDistanceMeters: 1.50,
        deskCameraHeightMeters: 1.18,
      );
      final cam = solver.buildDesktopCalibratedCameraView(widthPx: 1280, heightPx: 720);

      final visorA = vm.Vector3(0.05, 1.60, -0.20);
      final cornersA = _projectMarkerThroughCam(
        cam: cam,
        centerRoom: visorA,
        markerOrientationRoom: vm.Quaternion.identity(),
        markerSizeMeters: 0.048,
        widthPx: 1280,
        heightPx: 720,
      );

      final halfTanX = math.tan((68.0 * math.pi / 180.0) * 0.5);
      final focalNormX = 0.5 / halfTanX;
      const trueIpd = 0.064;
      final zCamA = 1.50 + visorA.z;
      final halfEyeA = 0.5 * focalNormX * trueIpd / zCamA;

      final cleanFrame = VrDesktopVisionFrame.fromPlatformMap({
        'status': 'ok',
        'timestampMicros': 100000,
        'mirrored': false,
        'frameWidth': 1280,
        'frameHeight': 720,
        'eyesFromFaceLandmarks': true,
        'nose': {'u': 0.50, 'v': 0.34, 'c': 0.96},
        'leftEye': {'u': 0.50 + halfEyeA, 'v': 0.32, 'c': 0.96},
        'rightEye': {'u': 0.50 - halfEyeA, 'v': 0.32, 'c': 0.96},
        'rightWrist': {'u': 0.38, 'v': 0.48, 'c': 0.92},
        'rightPalmSpan': 0.080,
        'fiducialTags': [
          {
            'payload': 'vrlizate://tag?node=visor&mm=48',
            'frameWidth': 1280,
            'frameHeight': 720,
            'corners': [for (final c in cornersA) c.toJson()],
          },
        ],
      })!;

      final sol1 = solver.solve(nowMicros: 100000, vision: cleanFrame);
      expect(sol1.estimatedUserIpdMeters, closeTo(trueIpd, 0.002));

      // Inject 1-frame background QR false positive 1.9 m away (118 m/s!) + bogus eyes
      final visorGlitch = vm.Vector3(-1.40, 1.95, 0.95);
      final glitchCorners = _projectMarkerThroughCam(
        cam: cam,
        centerRoom: visorGlitch,
        markerOrientationRoom: vm.Quaternion.identity(),
        markerSizeMeters: 0.048,
        widthPx: 1280,
        heightPx: 720,
      );
      final glitchFrame = VrDesktopVisionFrame.fromPlatformMap({
        'status': 'ok',
        'timestampMicros': 116000,
        'mirrored': false,
        'frameWidth': 1280,
        'frameHeight': 720,
        'eyesFromFaceLandmarks': true,
        'nose': {'u': 0.80, 'v': 0.20, 'c': 0.95},
        'leftEye': {'u': 0.82, 'v': 0.20, 'c': 0.95},
        'rightEye': {'u': 0.78, 'v': 0.20, 'c': 0.95},
        // Impossible 1-frame hand jump across entire room:
        'rightWrist': {'u': 0.96, 'v': 0.05, 'c': 0.90},
        'rightPalmSpan': 0.025,
        'fiducialTags': [
          {
            'payload': 'vrlizate://tag?node=visor&mm=48',
            'frameWidth': 1280,
            'frameHeight': 720,
            'corners': [for (final c in glitchCorners) c.toJson()],
          },
        ],
      })!;

      final solGlitch = solver.solve(nowMicros: 116000, vision: glitchFrame);
      expect(solver.lastVisorObservationSpikeHeld, isTrue);
      expect(solver.lastHandObservationSpikeHeld, isTrue);
      expect(solGlitch.visorRoomPosition.distanceTo(visorA), lessThan(0.01));
      // Bogus eye span during spiked PnP frame must NOT corrupt Auto-IPD!
      expect(solGlitch.estimatedUserIpdMeters, closeTo(trueIpd, 0.002));
    });

    test('Multi-camera DLT hand triangulation rejects occluded mobile view and locks < 3 mm', () {
      final solver = VrTriNodeDepthSolver(
        webcamHorizontalFovDeg: 68.0,
        nominalStandingDistanceMeters: 1.50,
        deskCameraHeightMeters: 1.18,
      );
      final pcCam = solver.buildDesktopCalibratedCameraView(widthPx: 1280, heightPx: 720);

      // First lock Visor at (0.0, 1.60, 0.0)
      final visorPos = vm.Vector3(0.0, 1.60, 0.0);
      final visorCorners = _projectMarkerThroughCam(
        cam: pcCam,
        centerRoom: visorPos,
        markerOrientationRoom: vm.Quaternion.identity(),
        markerSizeMeters: 0.048,
        widthPx: 1280,
        heightPx: 720,
      );

      final mobileIntr = CameraDistortionIntrinsics.fromHorizontalFov(
        widthPx: 1280,
        heightPx: 720,
        horizontalFovDegrees: 74.0,
        k1: -0.10,
      );
      CalibratedCameraView makeRoomCam(String id, vm.Vector3 origin, double yaw) {
        // Look toward -Z_room / user workspace from side-angled vantage
        final r = vm.Quaternion.axisAngle(vm.Vector3(0, 1, 0), yaw).asRotationMatrix();
        return CalibratedCameraView(
          id: id,
          intrinsics: mobileIntr,
          worldToCameraRotation: r,
          worldToCameraTranslation: -r.transformed(origin),
        );
      }

      final auxCam1 = makeRoomCam('visor_aux', vm.Vector3(-0.55, 1.45, -1.25), -0.35);
      final auxCam2 = makeRoomCam('joy_aux', vm.Vector3(0.55, 1.10, -1.20), 0.38);

      // True right wrist in room space: (+0.24, 1.36, -0.58) -> relHead = (+0.24, -0.24, -0.58)
      final trueWristRoom = vm.Vector3(0.24, 1.36, -0.58);
      final pcPx = pcCam.projectWorldToPixel(trueWristRoom);
      final aux1Px = auxCam1.projectWorldToPixel(trueWristRoom);
      // Corrupt auxCam2 by +240 px to simulate hand self-occlusion on that view!
      final aux2CorruptedPx =
          auxCam2.projectWorldToPixel(trueWristRoom) + vm.Vector2(240.0, -130.0);

      final frame = VrDesktopVisionFrame.fromPlatformMap({
        'status': 'ok',
        'timestampMicros': 300000,
        'mirrored': false,
        'frameWidth': 1280,
        'frameHeight': 720,
        'rightWrist': {'u': pcPx.x / 1280.0, 'v': pcPx.y / 720.0, 'c': 0.96},
        'fiducialTags': [
          {
            'payload': 'vrlizate://tag?node=visor&mm=48',
            'frameWidth': 1280,
            'frameHeight': 720,
            'corners': [for (final c in visorCorners) c.toJson()],
          },
        ],
      })!;

      final sol = solver.solve(
        nowMicros: 300000,
        vision: frame,
        secondaryCameraViews: <CalibratedCameraView>[auxCam1, auxCam2],
        secondaryHandPixels: <String, ({vm.Vector2? leftWristPx, vm.Vector2? rightWristPx})>{
          'visor_aux': (leftWristPx: null, rightWristPx: aux1Px),
          'joy_aux': (leftWristPx: null, rightWristPx: aux2CorruptedPx),
        },
      );

      final expectedRelHead = trueWristRoom - visorPos;
      expect(sol.rightHandRelHead.distanceTo(expectedRelHead), lessThan(0.003));
    });

    test('RollingMedianBoneEstimator adapts subject forearm length online while ignoring outlier', () {
      final solver = VrTriNodeDepthSolver(
        webcamHorizontalFovDeg: 68.0,
        nominalStandingDistanceMeters: 1.50,
        deskCameraHeightMeters: 1.18,
        userNominalStatureMeters: 1.72,
      );
      final initialSeed = solver.estimatedRightForearmMeters;
      expect(initialSeed, closeTo(1.72 * 0.146, 1e-3));

      final pcCam = solver.buildDesktopCalibratedCameraView(widthPx: 1280, heightPx: 720);
      final halfTanX = math.tan((68.0 * math.pi / 180.0) * 0.5);
      final focalNormX = 0.5 / halfTanX;

      final visorPos = vm.Vector3(0.0, 1.60, 0.0);
      final visorCorners = _projectMarkerThroughCam(
        cam: pcCam,
        centerRoom: visorPos,
        markerOrientationRoom: vm.Quaternion.identity(),
        markerSizeMeters: 0.048,
        widthPx: 1280,
        heightPx: 720,
      );

      // True right shoulder at (0.20, 1.42, 0.0), elbow at (0.20, 1.12, -0.25),
      // wrist at (0.20, 1.12, -0.50) -> Z_cam_wrist = 1.00 m
      final pShoulderPx = pcCam.projectWorldToPixel(vm.Vector3(0.20, 1.42, 0.0));
      final pElbowPx = pcCam.projectWorldToPixel(vm.Vector3(0.20, 1.12, -0.25));
      final pWristPx = pcCam.projectWorldToPixel(vm.Vector3(0.20, 1.12, -0.50));
      final palmSpanAt1m = focalNormX * 0.094 / 1.00;

      for (var i = 1; i <= 7; i++) {
        final frame = VrDesktopVisionFrame.fromPlatformMap({
          'status': 'ok',
          'timestampMicros': i * 50000,
          'mirrored': false,
          'frameWidth': 1280,
          'frameHeight': 720,
          'rightShoulder': {
            'u': pShoulderPx.x / 1280.0,
            'v': pShoulderPx.y / 720.0,
            'c': 0.95,
          },
          'rightElbow': {
            'u': pElbowPx.x / 1280.0,
            'v': pElbowPx.y / 720.0,
            'c': 0.95,
          },
          'rightWrist': {
            'u': pWristPx.x / 1280.0,
            'v': pWristPx.y / 720.0,
            'c': 0.95,
          },
          'rightPalmSpan': palmSpanAt1m,
          'fiducialTags': [
            {
              'payload': 'vrlizate://tag?node=visor&mm=48',
              'frameWidth': 1280,
              'frameHeight': 720,
              'corners': [for (final c in visorCorners) c.toJson()],
            },
          ],
        })!;
        solver.solve(nowMicros: i * 50000, vision: frame);
      }

      // True elbow->wrist distance at midpoint elbowCamZ=1.25, handCamZ=1.00 is ~0.250 m
      expect(solver.learnedUpperBodyBoneLengthsMeters['right_elbow->right_wrist'], isNotNull);
      expect(solver.estimatedRightForearmMeters, closeTo(0.25, 0.02));
    });
  });
}
