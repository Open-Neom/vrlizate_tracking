import 'dart:math' as math;

import 'package:vector_math/vector_math.dart' as vm;
import 'package:vrlizate/vrlizate.dart'
    show
        CalibratedCameraView,
        CameraDistortionIntrinsics,
        CausalLandmarkStabilizer3D,
        ForwardKinematicChainRigidifier,
        KabschHorizonAligner,
        MultiViewDltTriangulator,
        RollingMedianBoneEstimator,
        VrAnthropometricSegmentRatios;

import '../session/vr_visual_marker_pose.dart';
import 'vr_desktop_beacon_models.dart';
import 'vr_fiducial_tag_codec.dart';

/// Solves 3-node metric triangulation (`Computadora ↔ Visor ↔ Joystick`) and
/// 3D left/right hand kinematics from Desktop AI vision keypoints, BLE/Wi-Fi RF
/// proximity, stereo ultrasonic azimuth, and Visor-reported inter-phone range.
class VrTriNodeDepthSolver {
  VrTriNodeDepthSolver({
    this.webcamHorizontalFovDeg = 68.0,
    this.webcamAspect = 16.0 / 9.0,
    this.nominalStandingDistanceMeters = 1.50,
    this.deskEdgeOffsetMeters = 0.42,
    this.deskCameraHeightMeters = 1.18,
    this.biacromialShoulderWidthMeters = 0.40,
    this.interocularSpanMeters = 0.064,
    double webcamPitchDeg = 0.0,
    double webcamRollDeg = 0.0,
    this.webcamK1 = 0.0,
    this.webcamK2 = 0.0,
    this.webcamP1 = 0.0,
    this.webcamP2 = 0.0,
    this.userNominalStatureMeters = 1.72,
  }) : _webcamPitchRad = webcamPitchDeg * math.pi / 180.0,
       _webcamRollRad = webcamRollDeg * math.pi / 180.0,
       _boneEstimator = RollingMedianBoneEstimator(
         seedLengthsMeters: VrAnthropometricSegmentRatios.buildUpperBodySeeds(
           userNominalStatureMeters,
         ),
       );

  final double webcamHorizontalFovDeg;
  final double webcamAspect;
  final double nominalStandingDistanceMeters;
  final double deskEdgeOffsetMeters;
  final double deskCameraHeightMeters;
  final double biacromialShoulderWidthMeters;
  final double interocularSpanMeters;
  final double webcamK1;
  final double webcamK2;
  final double webcamP1;
  final double webcamP2;
  final double userNominalStatureMeters;

  double _webcamPitchRad;
  double _webcamRollRad;

  bool get hasWebcamDistortion =>
      webcamK1 != 0.0 || webcamK2 != 0.0 || webcamP1 != 0.0 || webcamP2 != 0.0;

  double get webcamPitchDegrees => _webcamPitchRad * 180.0 / math.pi;
  double get webcamRollDegrees => _webcamRollRad * 180.0 / math.pi;

  /// Updates the desktop webcam pitch/roll angles relative to the room horizon
  /// (`+pitchDeg` when laptop/monitor camera is tilted upward).
  void setWebcamHorizonAngles({
    required double pitchDeg,
    double rollDeg = 0.0,
  }) {
    _webcamPitchRad = pitchDeg * math.pi / 180.0;
    _webcamRollRad = rollDeg * math.pi / 180.0;
  }

  /// Estimates and applies the desktop webcam upward/downward pitch angle from
  /// a single fiducial PnP observation (`pnpTranslationCamera` in CV camera
  /// meters) of a target at known world height `knownTargetWorldHeightMeters`.
  double calibrateHorizonPitchFromObservedHeight({
    required vm.Vector3 pnpTranslationCamera,
    required double knownTargetWorldHeightMeters,
  }) {
    final dyTrue = knownTargetWorldHeightMeters - deskCameraHeightMeters;
    final yUpCam = -pnpTranslationCamera.y;
    final zCam = pnpTranslationCamera.z;
    final denom = yUpCam * yUpCam + zCam * zCam;
    if (!denom.isFinite || denom < 1e-4) return webcamPitchDegrees;
    // dyTrue = yUpCam * cos(theta) + zCam * sin(theta)
    final r = math.sqrt(denom);
    final clamped = (dyTrue / r).clamp(-1.0, 1.0);
    final phi = math.atan2(yUpCam, zCam);
    final theta = math.asin(clamped) - phi;
    _webcamPitchRad = theta.clamp(-0.785398, 0.785398); // [-45 deg .. +45 deg]
    return webcamPitchDegrees;
  }

  int _sequence = 0;
  int _lastTimestampUs = 0;
  double _smoothedDcv = 1.50;
  double _smoothedDcj = 1.18;
  double _smoothedDvj = 0.46;
  final vm.Vector3 _smoothedVisorPos = vm.Vector3(0.0, 1.60, 0.0);
  final vm.Vector3 _smoothedJoyPos = vm.Vector3(0.0, 1.18, -0.38);
  final vm.Vector3 _smoothedLeftHand = vm.Vector3(-0.18, -0.22, -0.56);
  final vm.Vector3 _smoothedRightHand = vm.Vector3(0.18, -0.22, -0.56);
  final vm.Vector3 _leftHandVel = vm.Vector3.zero();
  final vm.Vector3 _rightHandVel = vm.Vector3.zero();
  final CausalLandmarkStabilizer3D _visorStabilizer =
      CausalLandmarkStabilizer3D(
        minCutoffHz: 2.2,
        beta: 0.45,
        maxSpeedMetersPerSec: 8.5,
        maxConsecutiveRejections: 4,
        maxExtrapolationFrames: 3,
      );
  final CausalLandmarkStabilizer3D _visorPnpStabilizer =
      CausalLandmarkStabilizer3D(
        minCutoffHz: 9.0,
        beta: 0.90,
        maxSpeedMetersPerSec: 8.5,
        maxConsecutiveRejections: 4,
        maxExtrapolationFrames: 2,
      );
  final CausalLandmarkStabilizer3D _joyStabilizer =
      CausalLandmarkStabilizer3D(
        minCutoffHz: 3.0,
        beta: 0.55,
        maxSpeedMetersPerSec: 11.0,
        maxConsecutiveRejections: 4,
        maxExtrapolationFrames: 3,
      );
  final CausalLandmarkStabilizer3D _leftHandStabilizer =
      CausalLandmarkStabilizer3D(
        minCutoffHz: 2.4,
        beta: 0.65,
        maxSpeedMetersPerSec: 12.0,
        maxConsecutiveRejections: 4,
        maxExtrapolationFrames: 3,
      );
  final CausalLandmarkStabilizer3D _rightHandStabilizer =
      CausalLandmarkStabilizer3D(
        minCutoffHz: 2.4,
        beta: 0.65,
        maxSpeedMetersPerSec: 12.0,
        maxConsecutiveRejections: 4,
        maxExtrapolationFrames: 3,
      );
  final RollingMedianBoneEstimator _boneEstimator;
  final ForwardKinematicChainRigidifier _armRigidifier =
      ForwardKinematicChainRigidifier(
        jointHierarchy: const <String, List<String>>{
          'left_shoulder': <String>['left_elbow'],
          'left_elbow': <String>['left_wrist'],
          'right_shoulder': <String>['right_elbow'],
          'right_elbow': <String>['right_wrist'],
        },
      );
  double _elapsedTrackingSec = 0.0;
  bool _lastVisorSpikeHeld = false;
  bool _lastHandSpikeHeld = false;
  double _leftConfidence = 0.0;
  double _rightConfidence = 0.0;
  bool _leftRaised = false;
  bool _rightRaised = false;
  double _leftPinch = 0.0;
  double _rightPinch = 0.0;
  int _waveOscillations = 0;
  int _lastWaveSign = 0;
  int _lastWaveFlipUs = 0;
  bool _hadVisorPnpLock = false;
  bool _hadJoyPnpLock = false;
  double _calibratedShoulderScale = 1.0;
  double? _smoothedEstimatedIpdMeters;
  VrDesktopVisionFrame? _lastObservedVisionRef;
  int _lastVisionObservedAtUs = 0;

  VrFacialExpression _activeFacialExpression = VrFacialExpression.neutral;
  double _activeFacialConfidence = 0.0;
  int _lastFacialHoldUntilUs = 0;
  VrFacialExpression _lastRawWink = VrFacialExpression.neutral;
  int _consecutiveWinkFrames = 0;
  final List<({int tsUs, double pitch, double yaw, double roll})>
  _visorImuHistory = [];

  static const VrVisualMarkerPoseSolver _pnpSolver = VrVisualMarkerPoseSolver(
    maxReprojectionRmsPixels: 4.5,
    maxRawAxisOrthogonalityError: 0.24,
    maxRelativeAxisScaleMismatch: 0.28,
    minimumQuadAreaPixels: 25,
    minimumEdgePixels: 3,
  );

  /// Personal shoulder-width scale factor learned automatically whenever a
  /// 1:1 millimeter fiducial tag and upper-body shoulders are seen together.
  double get calibratedShoulderScale => _calibratedShoulderScale;

  /// Smoothed optical IPD (`[0.050 .. 0.080]` m) measured from webcam eye
  /// keypoints + metric depth.
  double? get estimatedUserIpdMeters => _smoothedEstimatedIpdMeters;

  /// Subject-specific right forearm length (`m`) learned via rolling median.
  double get estimatedRightForearmMeters =>
      _boneEstimator.lengthFor('right_elbow->right_wrist');

  /// Subject-specific left forearm length (`m`) learned via rolling median.
  double get estimatedLeftForearmMeters =>
      _boneEstimator.lengthFor('left_elbow->left_wrist');

  /// Current rolling-median upper-body bone length map (`m`).
  Map<String, double> get learnedUpperBodyBoneLengthsMeters =>
      _boneEstimator.currentLengths;

  /// True if the last visor observation was rejected by the velocity spike gate.
  bool get lastVisorObservationSpikeHeld => _lastVisorSpikeHeld;

  /// True if either hand's last observation was rejected by the velocity spike gate.
  bool get lastHandObservationSpikeHeld => _lastHandSpikeHeld;

  void reset() {
    _lastTimestampUs = 0;
    _elapsedTrackingSec = 0.0;
    _lastVisorSpikeHeld = false;
    _lastHandSpikeHeld = false;
    _smoothedDcv = nominalStandingDistanceMeters;
    _smoothedDcj = nominalStandingDistanceMeters - 0.34;
    _smoothedDvj = 0.46;
    _smoothedVisorPos.setValues(0.0, 1.60, 0.0);
    _smoothedJoyPos.setValues(0.0, 1.18, -0.38);
    _smoothedLeftHand.setValues(-0.18, -0.22, -0.56);
    _smoothedRightHand.setValues(0.18, -0.22, -0.56);
    _leftHandVel.setZero();
    _rightHandVel.setZero();
    _visorStabilizer.reset();
    _visorPnpStabilizer.reset();
    _joyStabilizer.reset();
    _leftHandStabilizer.reset();
    _rightHandStabilizer.reset();
    _boneEstimator.reset();
    _armRigidifier.reset();
    _leftConfidence = 0.0;
    _rightConfidence = 0.0;
    _leftRaised = false;
    _rightRaised = false;
    _leftPinch = 0.0;
    _rightPinch = 0.0;
    _waveOscillations = 0;
    _lastWaveSign = 0;
    _hadVisorPnpLock = false;
    _hadJoyPnpLock = false;
    _calibratedShoulderScale = 1.0;
    _smoothedEstimatedIpdMeters = null;
    _lastObservedVisionRef = null;
    _lastVisionObservedAtUs = 0;
    _activeFacialExpression = VrFacialExpression.neutral;
    _activeFacialConfidence = 0.0;
    _lastFacialHoldUntilUs = 0;
    _lastRawWink = VrFacialExpression.neutral;
    _consecutiveWinkFrames = 0;
    _visorImuHistory.clear();
  }

  VrVisualMarkerPose? _solveTagPnp(
    VrFiducialMarkerObservation tag, {
    required bool frameMirrored,
    required double halfTanX,
  }) {
    if (!tag.isValid) return null;
    final fw = tag.frameWidth.toDouble();
    final fh = tag.frameHeight.toDouble();
    final fx = (fw * 0.5) / halfTanX;
    final calibration = VrMarkerFrameCalibration(
      fx: fx,
      fy: fx,
      cx: fw * 0.5,
      cy: fh * 0.5,
      k1: webcamK1,
      k2: webcamK2,
      p1: webcamP1,
      p2: webcamP2,
      verifiedForFrame: true,
    );
    var pxCorners = <vm.Vector2>[
      for (final c in tag.corners)
        vm.Vector2(
          (frameMirrored ? (1.0 - c.u) : c.u) * fw,
          c.v * fh,
        ),
    ];
    // Ensure clockwise Y-down winding [TL, TR, BR, BL] even if a camera driver
    // mirrored the stream without setting `mirrored`.
    var signedArea2 = 0.0;
    for (var i = 0; i < 4; i++) {
      final a = pxCorners[i];
      final b = pxCorners[(i + 1) % 4];
      signedArea2 += a.x * b.y - a.y * b.x;
    }
    if (signedArea2 < 0) {
      pxCorners = <vm.Vector2>[
        pxCorners[1],
        pxCorners[0],
        pxCorners[3],
        pxCorners[2],
      ];
    }
    try {
      return _pnpSolver.solve(
        corners: pxCorners,
        markerWidthMeters: tag.markerSizeMeters,
        markerHeightMeters: tag.markerSizeMeters,
        calibration: calibration,
      );
    } on VrVisualMarkerPoseException {
      return null;
    }
  }

  /// Computes a fused 3-node depth & hand-tracking frame.
  ///
  /// When optional [secondaryCameraViews] and [secondaryHandPixels] are
  /// supplied (calibrated in user room coordinates), left/right 3D wrists are
  /// triangulated via [MultiViewDltTriangulator] with automatic occluded-view
  /// outlier rejection.
  VrTriNodeDepthSolution solve({
    required int nowMicros,
    VrDesktopVisionFrame? vision,
    VrVisorBeaconFeedback? visorFeedback,
    int activeModalitiesMask = VrTriNodeDepthSolution.modalityWifiUdp,
    List<CalibratedCameraView>? secondaryCameraViews,
    Map<String, ({vm.Vector2? leftWristPx, vm.Vector2? rightWristPx})>?
    secondaryHandPixels,
  }) {
    final rawDtSec = _lastTimestampUs > 0
        ? math.max(0.0, (nowMicros - _lastTimestampUs) / 1e6)
        : 0.016;
    final dtSec = rawDtSec.clamp(0.004, 0.25);
    final handConfidenceDecay = math
        .pow(0.93, (rawDtSec / 0.016).clamp(0.25, 120.0))
        .toDouble();
    _lastTimestampUs = nowMicros;
    _elapsedTrackingSec += dtSec;
    _lastVisorSpikeHeld = false;
    _lastHandSpikeHeld = false;
    final isDuplicateVisionFrame =
        vision != null && identical(vision, _lastObservedVisionRef);
    var visionDtSec = dtSec;
    if (vision != null && !isDuplicateVisionFrame) {
      if (_lastVisionObservedAtUs > 0) {
        visionDtSec = ((nowMicros - _lastVisionObservedAtUs) / 1e6).clamp(
          0.004,
          0.25,
        );
      }
      _lastObservedVisionRef = vision;
      _lastVisionObservedAtUs = nowMicros;
    } else if (vision == null) {
      _lastObservedVisionRef = null;
    }
    final alphaPos = (dtSec * 12.0).clamp(0.08, 0.85);
    final alphaHand = (dtSec * 18.0).clamp(0.12, 0.92);

    final halfTanX = math.tan((webcamHorizontalFovDeg * math.pi / 180.0) * 0.5);
    final halfTanY = halfTanX / webcamAspect;
    final focalNormX = 0.5 / halfTanX;

    final fwPx = (vision != null && vision.frameWidth > 0)
        ? vision.frameWidth
        : 1280;
    final fhPx = (vision != null && vision.frameHeight > 0)
        ? vision.frameHeight
        : (fwPx / webcamAspect).round();
    final distIntr = hasWebcamDistortion
        ? CameraDistortionIntrinsics.fromHorizontalFov(
            widthPx: fwPx,
            heightPx: fhPx,
            horizontalFovDegrees: webcamHorizontalFovDeg,
            k1: webcamK1,
            k2: webcamK2,
            p1: webcamP1,
            p2: webcamP2,
          )
        : null;

    ({double u, double v}) undistortUserUv(
      VrDesktopKeypoint2D kp, {
      required bool mirrored,
    }) {
      if (distIntr == null) {
        return (u: mirrored ? kp.u : (1.0 - kp.u), v: kp.v);
      }
      final rawCamU = mirrored ? (1.0 - kp.u) : kp.u;
      final ideal = distIntr.undistortUv(rawCamU, kp.v, fwPx, fhPx);
      return (u: 1.0 - ideal.x, v: ideal.y);
    }

    // -------------------------------------------------------------------------
    // 1. Solve Computer -> Visor Distance (d_CV) & 3D Head Anchor
    //    Priority A: 1:1 mm Fiducial 6-DoF Homography PnP
    //    Priority B: Auto-calibrated AI Upper-Body & Interocular Pinhole
    //    Priority C: BLE RSSI Path-Loss + Ultrasonic Azimuth Fallback
    // -------------------------------------------------------------------------
    VrVisualMarkerPose? visorPnpPose;
    VrVisualMarkerPose? joyPnpPose;
    if (vision != null && vision.fiducialTags.isNotEmpty) {
      for (final tag in vision.fiducialTags) {
        final solved = _solveTagPnp(
          tag,
          frameMirrored: vision.mirrored,
          halfTanX: halfTanX,
        );
        if (solved == null) continue;
        if (tag.node == VrFiducialNodeKind.visor && visorPnpPose == null) {
          visorPnpPose = solved;
        } else if (tag.node == VrFiducialNodeKind.joystick &&
            joyPnpPose == null) {
          joyPnpPose = solved;
        }
      }
    }

    var effectiveModalitiesMask = activeModalitiesMask;
    if (visorPnpPose != null || joyPnpPose != null) {
      effectiveModalitiesMask |= VrTriNodeDepthSolution.modalityFiducialPnp;
    }

    double? opticalDcv;
    double? rawUncalibratedShoulderZ;
    double headU = 0.5;
    double headV = 0.36;
    var visionConfidence = 0.0;

    if (vision != null && vision.hasUpperBody) {
      final ls = vision.leftShoulder;
      final rs = vision.rightShoulder;
      final le = vision.leftEye;
      final re = vision.rightEye;
      final nose = vision.nose;

      if (nose != null && nose.isValid) {
        final uv = undistortUserUv(nose, mirrored: vision.mirrored);
        headU = uv.u;
        headV = uv.v;
        visionConfidence = nose.confidence;
      } else if (ls != null && ls.isValid && rs != null && rs.isValid) {
        final luv = undistortUserUv(ls, mirrored: vision.mirrored);
        final ruv = undistortUserUv(rs, mirrored: vision.mirrored);
        headU = 0.5 * (luv.u + ruv.u);
        headV = (0.5 * (luv.v + ruv.v) - 0.14).clamp(0.05, 0.95);
        visionConfidence = 0.5 * (ls.confidence + rs.confidence);
      }

      if (ls != null && ls.isValid && rs != null && rs.isValid) {
        final luv = undistortUserUv(ls, mirrored: vision.mirrored);
        final ruv = undistortUserUv(rs, mirrored: vision.mirrored);
        final du = (ruv.u - luv.u).abs();
        final dv = (ruv.v - luv.v).abs() / webcamAspect;
        final spanNorm = math.sqrt(du * du + dv * dv);
        if (spanNorm > 0.045) {
          rawUncalibratedShoulderZ =
              focalNormX * biacromialShoulderWidthMeters / spanNorm;
          opticalDcv = (rawUncalibratedShoulderZ * _calibratedShoulderScale)
              .clamp(0.45, 4.20);
          visionConfidence = math.max(
            visionConfidence,
            0.5 * (ls.confidence + rs.confidence),
          );
        }
      }
      if (le != null && le.isValid && re != null && re.isValid) {
        final leUv = undistortUserUv(le, mirrored: vision.mirrored);
        final reUv = undistortUserUv(re, mirrored: vision.mirrored);
        final eyeSpan = (reUv.u - leUv.u).abs();
        if (eyeSpan > 0.014) {
          final referenceIpd =
              _smoothedEstimatedIpdMeters ?? interocularSpanMeters;
          final eyeDepth = (focalNormX * referenceIpd / eyeSpan).clamp(
            0.40,
            3.60,
          );
          opticalDcv = opticalDcv == null
              ? eyeDepth
              : (opticalDcv * 0.72 + eyeDepth * 0.28);
        }
      }
    }

    // Optional BLE RSSI path-loss range backup when user steps out of frame
    double? rfDcv;
    final rssi = visorFeedback?.desktopBleRssiDbm;
    if (rssi != null && rssi.isFinite && rssi >= -95 && rssi <= -25) {
      // Log-distance path-loss model: RSSI_1m = -53 dBm, n = 2.2
      rfDcv = math
          .pow(10.0, (-53.0 - rssi) / (10.0 * 2.2))
          .toDouble()
          .clamp(0.45, 4.50);
    }

    double zFromDesk;
    double visorX;
    double visorY;
    double visorZ;
    var isVisorMeasured = false;

    if (visorPnpPose != null) {
      final t = visorPnpPose.translationMeters;
      final leveled = KabschHorizonAligner.levelCameraPointToHorizon(
        t,
        pitchRadians: _webcamPitchRad,
        rollRadians: _webcamRollRad,
      );
      final pnpDcv = t.length.clamp(0.30, 5.0);
      zFromDesk = leveled.z.clamp(0.28, 4.80);
      visorX = leveled.x.clamp(-2.5, 2.5);
      visorY = (deskCameraHeightMeters + leveled.y).clamp(0.65, 2.35);
      visorZ = (zFromDesk - nominalStandingDistanceMeters).clamp(-1.40, 3.0);

      final rawVisorRoom = vm.Vector3(visorX, visorY, visorZ);
      if (!_hadVisorPnpLock) {
        _hadVisorPnpLock = true;
        _visorPnpStabilizer.snapTo(rawVisorRoom);
        _visorStabilizer.snapTo(rawVisorRoom);
        _smoothedVisorPos.setFrom(rawVisorRoom);
        _smoothedDcv = pnpDcv;
        isVisorMeasured = !isDuplicateVisionFrame;
      } else if (isDuplicateVisionFrame) {
        final pred = _visorPnpStabilizer.updateMissing(dtSec);
        if (pred != null) {
          _smoothedVisorPos.setFrom(pred.position);
        }
        isVisorMeasured = false;
      } else {
        final s = _visorPnpStabilizer.updateObserved(rawVisorRoom, visionDtSec);
        _lastVisorSpikeHeld = s.isHeldBySpikeGate;
        isVisorMeasured = s.isMeasuredObservation;
        _smoothedVisorPos.setFrom(s.position);
        if (isVisorMeasured) {
          _visorStabilizer.snapTo(s.position);
          final alphaVisor = (visionDtSec * 32.0).clamp(0.42, 0.96);
          _smoothedDcv += (pnpDcv - _smoothedDcv) * alphaVisor;
        }
      }

      // Self-calibrate anthropometric shoulder width ONLY on genuine (non-spiked)
      // 1:1 mm PnP frames!
      if (isVisorMeasured &&
          rawUncalibratedShoulderZ != null &&
          rawUncalibratedShoulderZ > 0.35) {
        final observedScale = (zFromDesk / rawUncalibratedShoulderZ).clamp(
          0.75,
          1.30,
        );
        _calibratedShoulderScale =
            _calibratedShoulderScale * 0.80 + observedScale * 0.20;
      }

      visionConfidence = math.max(
        visionConfidence,
        visorPnpPose.confidence.clamp(0.85, 1.0),
      );
    } else {
      _hadVisorPnpLock = false;
      final hasVisorObs =
          (vision != null && vision.hasUpperBody) ||
          rfDcv != null ||
          (visorFeedback?.desktopSonarAzimuthRad != null);
      double targetDcv = _smoothedDcv;
      if (opticalDcv != null && rfDcv != null) {
        targetDcv = opticalDcv * 0.85 + rfDcv * 0.15;
      } else if (opticalDcv != null) {
        targetDcv = opticalDcv;
      } else if (rfDcv != null) {
        targetDcv = rfDcv;
      }
      _smoothedDcv += (targetDcv - _smoothedDcv) * alphaPos;

      double rayX = (headU - 0.5) * 2.0 * halfTanX;
      final rayY = (0.5 - headV) * 2.0 * halfTanY;
      final sonarAz = visorFeedback?.desktopSonarAzimuthRad;
      if (visionConfidence < 0.25 && sonarAz != null && sonarAz.isFinite) {
        rayX = math.tan(sonarAz.clamp(-1.05, 1.05));
      }
      final rayInvLen = 1.0 / math.sqrt(rayX * rayX + rayY * rayY + 1.0);
      final pCvHead = vm.Vector3(
        -_smoothedDcv * rayX * rayInvLen,
        -_smoothedDcv * rayY * rayInvLen,
        _smoothedDcv * rayInvLen,
      );
      final leveledHead = KabschHorizonAligner.levelCameraPointToHorizon(
        pCvHead,
        pitchRadians: _webcamPitchRad,
        rollRadians: _webcamRollRad,
      );
      zFromDesk = leveledHead.z.clamp(0.28, 4.80);
      visorX = leveledHead.x.clamp(-2.5, 2.5);
      visorY = (deskCameraHeightMeters + leveledHead.y).clamp(0.75, 2.25);
      visorZ = (zFromDesk - nominalStandingDistanceMeters).clamp(-1.35, 2.8);

      if (hasVisorObs) {
        if (isDuplicateVisionFrame &&
            rfDcv == null &&
            visorFeedback?.desktopSonarAzimuthRad == null) {
          final pred = _visorStabilizer.updateMissing(dtSec);
          if (pred != null) {
            _smoothedVisorPos.setFrom(pred.position);
          }
          isVisorMeasured = false;
        } else {
          final effVisorDt =
              (vision != null && vision.hasUpperBody && !isDuplicateVisionFrame)
              ? visionDtSec
              : dtSec;
          final s = _visorStabilizer.updateObserved(
            vm.Vector3(visorX, visorY, visorZ),
            effVisorDt,
          );
          _lastVisorSpikeHeld = s.isHeldBySpikeGate;
          isVisorMeasured = s.isMeasuredObservation && !isDuplicateVisionFrame;
          _smoothedVisorPos.setFrom(s.position);
        }
      } else {
        final pred = _visorStabilizer.updateMissing(dtSec);
        if (pred != null) {
          _smoothedVisorPos.setFrom(pred.position);
        }
        isVisorMeasured = false;
      }
    }

    // Webcam Auto-IPD measurement when both uncovered face-landmark eyes are
    // visible & facing the PC AND visor depth is a genuine measured observation.
    if (isVisorMeasured &&
        vision != null &&
        (vision.eyesFromFaceLandmarks || vision.inferenceMs == null)) {
      final le = vision.leftEye;
      final re = vision.rightEye;
      if (le != null &&
          le.isValid &&
          re != null &&
          re.isValid &&
          le.confidence >= 0.50 &&
          re.confidence >= 0.50 &&
          (visorPnpPose != null || rawUncalibratedShoulderZ != null)) {
        final leUv = undistortUserUv(le, mirrored: vision.mirrored);
        final reUv = undistortUserUv(re, mirrored: vision.mirrored);
        var frontal = true;
        final nose = vision.nose;
        if (nose != null && nose.isValid) {
          final nUv = undistortUserUv(nose, mirrored: vision.mirrored);
          final dl = (nUv.u - leUv.u).abs();
          final dr = (reUv.u - nUv.u).abs();
          final maxGap = math.max(dl, dr);
          frontal = maxGap > 1e-4 && (math.min(dl, dr) / maxGap) >= 0.58;
        }
        if (frontal) {
          final du = (reUv.u - leUv.u).abs();
          final dv = (reUv.v - leUv.v).abs() / webcamAspect;
          final eyeSpanNorm = math.sqrt(du * du + dv * dv);
          if (eyeSpanNorm > 0.012) {
            final measuredIpd = eyeSpanNorm * zFromDesk / focalNormX;
            if (measuredIpd >= 0.050 && measuredIpd <= 0.080) {
              _smoothedEstimatedIpdMeters = _smoothedEstimatedIpdMeters == null
                  ? measuredIpd
                  : (_smoothedEstimatedIpdMeters! * 0.82 + measuredIpd * 0.18);
            }
          }
        }
      }
    }

    final visorAzimuth = math.atan2(visorX, zFromDesk);
    // Gentle drift anchor: when the user faces the desk, yaw correction nudges
    // subtle gyro drift without fighting intentional head turns.
    final yawCorrection = -visorAzimuth * 0.15;

    // -------------------------------------------------------------------------
    // 2. Solve 3D Left & Right Hand Kinematics + Raised/Pinch/Wave AI Gestures
    // -------------------------------------------------------------------------
    final reportedDvj = (visorFeedback?.visorToJoystickMeters ?? 0.44).clamp(
      0.20,
      0.88,
    );

    double shoulderYNorm = (headV + 0.14).clamp(0.10, 0.95);
    if (vision != null) {
      final ls = vision.leftShoulder;
      final rs = vision.rightShoulder;
      if (ls != null && ls.isValid && rs != null && rs.isValid) {
        final luv = undistortUserUv(ls, mirrored: vision.mirrored);
        final ruv = undistortUserUv(rs, mirrored: vision.mirrored);
        shoulderYNorm = 0.5 * (luv.v + ruv.v);
      }
    }

    // Optional N-camera DLT hand triangulation when calibrated auxiliary views
    // are attached alongside the desktop Faro webcam.
    MultiViewDltTriangulator? handDlt;
    if (secondaryCameraViews != null &&
        secondaryCameraViews.isNotEmpty &&
        secondaryHandPixels != null) {
      final desktopCam = buildDesktopCalibratedCameraView(
        widthPx: fwPx,
        heightPx: fhPx,
      );
      handDlt = MultiViewDltTriangulator(
        cameras: <CalibratedCameraView>[desktopCam, ...secondaryCameraViews],
        targetNormalizedReprojectionError: 0.012,
        maxCamerasToDrop: 1,
      );
    }

    void updateHand({
      required bool isRight,
      required VrDesktopKeypoint2D? shoulder,
      required VrDesktopKeypoint2D? wrist,
      required VrDesktopKeypoint2D? elbow,
      required VrDesktopKeypoint2D? indexTip,
      required VrDesktopKeypoint2D? thumbTip,
      required double? palmSpanNorm,
      required int? extendedFingers,
      required bool mirrored,
    }) {
      final handStab = isRight ? _rightHandStabilizer : _leftHandStabilizer;
      final prevPos = isRight ? _smoothedRightHand : _smoothedLeftHand;
      final targetVel = isRight ? _rightHandVel : _leftHandVel;

      // Attempt multi-camera DLT triangulation if secondary pixel observations exist.
      vm.Vector3? triangulatedWristRoom;
      if (handDlt != null && secondaryHandPixels != null) {
        final obs = <String, vm.Vector2>{};
        if (wrist != null && wrist.isValid) {
          final rawCamU = mirrored ? (1.0 - wrist.u) : wrist.u;
          obs['pc_faro'] = vm.Vector2(rawCamU * fwPx, wrist.v * fhPx);
        }
        for (final entry in secondaryHandPixels.entries) {
          final px = isRight ? entry.value.rightWristPx : entry.value.leftWristPx;
          if (px != null) obs[entry.key] = px;
        }
        final tri = handDlt.triangulatePixels(obs, enableOutlierRejection: true);
        if (tri != null) {
          triangulatedWristRoom = tri.pointWorld;
        }
      }

      if ((wrist == null || !wrist.isValid) && triangulatedWristRoom == null) {
        final pred = handStab.updateMissing(dtSec);
        if (pred != null) {
          prevPos.setFrom(pred.position);
          targetVel.setFrom(pred.velocity);
        }
        if (isRight) {
          // Gentle ~320 ms hold across 20 Hz webcam frames so hands never blink,
          // scaled by real elapsed time so stale observations expire cleanly.
          _rightConfidence *= handConfidenceDecay;
          if (_rightConfidence < 0.05) _rightConfidence = 0.0;
          if (_rightConfidence < 0.18) {
            _rightRaised = false;
            _rightPinch = 0.0;
          }
        } else {
          _leftConfidence *= handConfidenceDecay;
          if (_leftConfidence < 0.05) _leftConfidence = 0.0;
          if (_leftConfidence < 0.18) {
            _leftRaised = false;
            _leftPinch = 0.0;
          }
        }
        return;
      }

      final boneKey = isRight
          ? 'right_elbow->right_wrist'
          : 'left_elbow->left_wrist';
      final realForearmMeters = _boneEstimator.lengthFor(boneKey);

      double wu = 0.5;
      double wv = 0.5;
      double handZFromDesk;
      vm.Vector3 localRel;
      var hasDirectPalmDepth = false;

      if (triangulatedWristRoom != null) {
        handZFromDesk = math.max(
          0.24,
          triangulatedWristRoom.z + nominalStandingDistanceMeters,
        );
        if (wrist != null && wrist.isValid) {
          final wUv = undistortUserUv(wrist, mirrored: mirrored);
          wu = wUv.u;
          wv = wUv.v;
        }
        localRel = vm.Vector3(
          (triangulatedWristRoom.x - _smoothedVisorPos.x).clamp(-0.75, 0.75),
          (triangulatedWristRoom.y - _smoothedVisorPos.y).clamp(-0.65, 0.60),
          (triangulatedWristRoom.z - _smoothedVisorPos.z).clamp(-0.98, -0.22),
        );
        hasDirectPalmDepth = true;
      } else {
        final wUv = undistortUserUv(wrist!, mirrored: mirrored);
        wu = wUv.u;
        wv = wUv.v;

        // 1) AI Monocular Palm-Scale Metric Depth (`~9.4 cm` adult palm span)
        //    combined with rolling-median forearm foreshortening & d_VJ telemetry.
        var reachForward = 0.46;
        if (palmSpanNorm != null &&
            palmSpanNorm.isFinite &&
            palmSpanNorm >= 0.020 &&
            palmSpanNorm <= 0.65) {
          const adultPalmSpanMeters = 0.094;
          final zHandFromPalm =
              (focalNormX * adultPalmSpanMeters / palmSpanNorm).clamp(
                0.24,
                2.30,
              );
          reachForward = (zFromDesk - zHandFromPalm).clamp(0.24, 0.92);
          hasDirectPalmDepth = true;
        }
        if (elbow != null && elbow.isValid) {
          final eUv = undistortUserUv(elbow, mirrored: mirrored);
          final du = wu - eUv.u;
          final dv = (wv - eUv.v) / webcamAspect;
          final forearm2D =
              math.sqrt(du * du + dv * dv) * zFromDesk / focalNormX;
          final ratio = (forearm2D / realForearmMeters).clamp(0.0, 1.0);
          final dzForearm =
              realForearmMeters * math.sqrt(math.max(0.0, 1.0 - ratio * ratio));
          final elbowReach = (0.28 + dzForearm * 1.30).clamp(0.24, 0.84);
          reachForward = hasDirectPalmDepth
              ? (reachForward * 0.72 + elbowReach * 0.28)
              : elbowReach;
        }
        if (visorFeedback?.joystickConnected == true && !hasDirectPalmDepth) {
          reachForward = reachForward * 0.60 + reportedDvj * 0.40;
        }

        final handCamZ = math.max(0.24, zFromDesk - reachForward);
        final pCvHand = vm.Vector3(
          -(wu - 0.5) * 2.0 * halfTanX * handCamZ,
          -(0.5 - wv) * 2.0 * halfTanY * handCamZ,
          handCamZ,
        );
        final leveledHand = KabschHorizonAligner.levelCameraPointToHorizon(
          pCvHand,
          pitchRadians: _webcamPitchRad,
          rollRadians: _webcamRollRad,
        );
        handZFromDesk = math.max(0.24, leveledHand.z);
        final handXRoom = leveledHand.x;
        final handYRoom = deskCameraHeightMeters + leveledHand.y;

        var handRoomVec = vm.Vector3(
          handXRoom,
          handYRoom,
          handZFromDesk - nominalStandingDistanceMeters,
        );

        // Online rolling-median bone estimation & forward-kinematic rigidification
        // when shoulder -> elbow -> wrist are all observed with metric depth.
        if (hasDirectPalmDepth &&
            shoulder != null &&
            shoulder.isValid &&
            elbow != null &&
            elbow.isValid &&
            shoulder.confidence >= 0.55 &&
            elbow.confidence >= 0.55 &&
            wrist.confidence >= 0.55) {
          final sUv = undistortUserUv(shoulder, mirrored: mirrored);
          final eUv = undistortUserUv(elbow, mirrored: mirrored);
          final elbowCamZ = 0.5 * (zFromDesk + handCamZ);
          final pS = KabschHorizonAligner.levelCameraPointToHorizon(
            vm.Vector3(
              -(sUv.u - 0.5) * 2.0 * halfTanX * zFromDesk,
              -(0.5 - sUv.v) * 2.0 * halfTanY * zFromDesk,
              zFromDesk,
            ),
            pitchRadians: _webcamPitchRad,
            rollRadians: _webcamRollRad,
          );
          final pE = KabschHorizonAligner.levelCameraPointToHorizon(
            vm.Vector3(
              -(eUv.u - 0.5) * 2.0 * halfTanX * elbowCamZ,
              -(0.5 - eUv.v) * 2.0 * halfTanY * elbowCamZ,
              elbowCamZ,
            ),
            pitchRadians: _webcamPitchRad,
            rollRadians: _webcamRollRad,
          );
          final side = isRight ? 'right' : 'left';
          final rawChain = <String, vm.Vector3>{
            '${side}_shoulder': vm.Vector3(
              pS.x,
              deskCameraHeightMeters + pS.y,
              pS.z - nominalStandingDistanceMeters,
            ),
            '${side}_elbow': vm.Vector3(
              pE.x,
              deskCameraHeightMeters + pE.y,
              pE.z - nominalStandingDistanceMeters,
            ),
            '${side}_wrist': handRoomVec,
          };
          if (!isDuplicateVisionFrame) {
            _boneEstimator.recordMeasuredFrame(
              rawChain,
              timeSeconds: _elapsedTrackingSec,
            );
          }
          final rigidChain = _armRigidifier.rigidify(
            rawChain,
            _boneEstimator.currentLengths,
          );
          handRoomVec = rigidChain['${side}_wrist'] ?? handRoomVec;
        }

        final relX = (handRoomVec.x - _smoothedVisorPos.x).clamp(-0.62, 0.62);
        final relY = (handRoomVec.y - _smoothedVisorPos.y).clamp(-0.55, 0.48);
        // Project hand depth into ergonomic stereo comfort zone [-0.92m .. -0.48m]
        // so hands NEVER collapse to -0.20m ("muy pegado") due to unanchored IMU
        // yaw drift while smoothly extending deeper as the arm reaches forward.
        final ergonomicStereoDepth =
            (0.48 + (reachForward - 0.24).clamp(0.0, 0.68) * 0.65).clamp(
              0.48,
              0.92,
            );
        localRel = vm.Vector3(relX, relY, -ergonomicStereoDepth);
      }

      // Hysteresis for "hand raised" detection: wrist above shoulder + margin
      final prevRaised = isRight ? _rightRaised : _leftRaised;
      final raiseThresholdV = prevRaised
          ? (shoulderYNorm + 0.04)
          : (shoulderYNorm - 0.02);
      final raised = wv <= raiseThresholdV || localRel.y >= -0.06;

      // Pinch strength from 2D thumbTip <-> indexTip proximity normalized by depth
      var pinch = isRight ? _rightPinch * 0.6 : _leftPinch * 0.6;
      if (indexTip != null &&
          indexTip.isValid &&
          thumbTip != null &&
          thumbTip.isValid) {
        final iUv = undistortUserUv(indexTip, mirrored: mirrored);
        final tUv = undistortUserUv(thumbTip, mirrored: mirrored);
        final dx = (iUv.u - tUv.u) * handZFromDesk / focalNormX;
        final dy =
            ((iUv.v - tUv.v) / webcamAspect) * handZFromDesk / focalNormX;
        final tipDistMeters = math.sqrt(dx * dx + dy * dy);
        // <= 2.4 cm = full pinch (1.0), >= 7.5 cm = open hand (0.0)
        pinch = ((0.075 - tipDistMeters) / (0.075 - 0.024)).clamp(0.0, 1.0);
        if (extendedFingers != null && extendedFingers >= 4 && pinch < 0.45) {
          pinch *= 0.5;
        }
      }

      if (isDuplicateVisionFrame && triangulatedWristRoom == null) {
        final pred = handStab.updateMissing(dtSec);
        if (pred != null) {
          prevPos.setFrom(pred.position);
          targetVel.setFrom(pred.velocity);
        }
        return;
      }

      final handObsDt = triangulatedWristRoom != null ? dtSec : visionDtSec;
      final stabilized = handStab.updateObserved(localRel, handObsDt);
      if (stabilized.isHeldBySpikeGate) {
        _lastHandSpikeHeld = true;
      }
      prevPos.setFrom(stabilized.position);
      targetVel.setFrom(stabilized.velocity);

      // Wave oscillation tracker when hand is raised and moving laterally on
      // genuine non-spiked observations.
      if (!stabilized.isHeldBySpikeGate &&
          raised &&
          targetVel.x.abs() > 0.38) {
        final sign = targetVel.x > 0 ? 1 : -1;
        if (sign != _lastWaveSign) {
          if (nowMicros - _lastWaveFlipUs < 900000) {
            _waveOscillations = (_waveOscillations + 1).clamp(0, 6);
          } else {
            _waveOscillations = 1;
          }
          _lastWaveSign = sign;
          _lastWaveFlipUs = nowMicros;
        }
      } else if (nowMicros - _lastWaveFlipUs > 1100000) {
        _waveOscillations = 0;
      }

      final baseWristConf = wrist?.confidence ?? 0.92;
      final confBoost = (extendedFingers != null && extendedFingers >= 2)
          ? 0.08
          : 0.0;
      if (isRight) {
        _rightConfidence = (baseWristConf + confBoost).clamp(0.0, 1.0);
        _rightRaised = raised;
        _rightPinch = pinch;
      } else {
        _leftConfidence = (baseWristConf + confBoost).clamp(0.0, 1.0);
        _leftRaised = raised;
        _leftPinch = pinch;
      }
    }

    if (vision != null || handDlt != null) {
      final mirrored = vision?.mirrored ?? false;
      updateHand(
        isRight: false,
        shoulder: vision?.leftShoulder,
        wrist: vision?.leftWrist,
        elbow: vision?.leftElbow,
        indexTip: vision?.leftIndexTip,
        thumbTip: vision?.leftThumbTip,
        palmSpanNorm: vision?.leftPalmSpanNorm,
        extendedFingers: vision?.leftExtendedFingers,
        mirrored: mirrored,
      );
      updateHand(
        isRight: true,
        shoulder: vision?.rightShoulder,
        wrist: vision?.rightWrist,
        elbow: vision?.rightElbow,
        indexTip: vision?.rightIndexTip,
        thumbTip: vision?.rightThumbTip,
        palmSpanNorm: vision?.rightPalmSpanNorm,
        extendedFingers: vision?.rightExtendedFingers,
        mirrored: mirrored,
      );
    } else {
      final predL = _leftHandStabilizer.updateMissing(dtSec);
      if (predL != null) {
        _smoothedLeftHand.setFrom(predL.position);
        _leftHandVel.setFrom(predL.velocity);
      }
      final predR = _rightHandStabilizer.updateMissing(dtSec);
      if (predR != null) {
        _smoothedRightHand.setFrom(predR.position);
        _rightHandVel.setFrom(predR.velocity);
      }
      _leftConfidence *= handConfidenceDecay;
      if (_leftConfidence < 0.05) _leftConfidence = 0.0;
      if (_leftConfidence < 0.18) {
        _leftRaised = false;
        _leftPinch = 0.0;
      }
      _rightConfidence *= handConfidenceDecay;
      if (_rightConfidence < 0.05) _rightConfidence = 0.0;
      if (_rightConfidence < 0.18) {
        _rightRaised = false;
        _rightPinch = 0.0;
      }
    }

    // -------------------------------------------------------------------------
    // 3. Solve 3-Node Triangle: Computer (0,0,0) <-> Visor <-> Joystick
    // -------------------------------------------------------------------------
    vm.Vector3 joyRelHead;
    if (_leftConfidence >= 0.30 && _rightConfidence >= 0.30) {
      joyRelHead = (_smoothedLeftHand + _smoothedRightHand) * 0.5;
    } else if (_rightConfidence >= 0.30) {
      joyRelHead = _smoothedRightHand.clone();
    } else if (_leftConfidence >= 0.30) {
      joyRelHead = _smoothedLeftHand.clone();
    } else {
      joyRelHead = vm.Vector3(0.0, -0.28, -reportedDvj);
    }

    if (visorFeedback?.joystickConnected == true &&
        visorFeedback?.visorToJoystickMeters != null) {
      final len = joyRelHead.length;
      if (len > 1e-4) {
        final blendedLen = len * 0.55 + reportedDvj * 0.45;
        joyRelHead.scale(blendedLen / len);
      }
    }

    if (joyPnpPose != null) {
      final tj = joyPnpPose.translationMeters;
      final leveledJoy = KabschHorizonAligner.levelCameraPointToHorizon(
        tj,
        pitchRadians: _webcamPitchRad,
        rollRadians: _webcamRollRad,
      );
      final pnpJoyRoom = vm.Vector3(
        leveledJoy.x.clamp(-2.5, 2.5),
        (deskCameraHeightMeters + leveledJoy.y).clamp(0.40, 2.35),
        (leveledJoy.z.clamp(0.22, 4.80) - nominalStandingDistanceMeters).clamp(
          -1.45,
          3.0,
        ),
      );
      if (!_hadJoyPnpLock) {
        _hadJoyPnpLock = true;
        _joyStabilizer.snapTo(pnpJoyRoom);
        _smoothedJoyPos.setFrom(pnpJoyRoom);
      } else if (isDuplicateVisionFrame) {
        final predJoy = _joyStabilizer.updateMissing(dtSec);
        if (predJoy != null) {
          _smoothedJoyPos.setFrom(predJoy.position);
        }
      } else {
        final sJoy = _joyStabilizer.updateObserved(pnpJoyRoom, visionDtSec);
        _smoothedJoyPos.setFrom(sJoy.position);
      }
    } else {
      _hadJoyPnpLock = false;
      final targetJoyRoom = _smoothedVisorPos + joyRelHead;
      _smoothedJoyPos.x += (targetJoyRoom.x - _smoothedJoyPos.x) * alphaHand;
      _smoothedJoyPos.y += (targetJoyRoom.y - _smoothedJoyPos.y) * alphaHand;
      _smoothedJoyPos.z += (targetJoyRoom.z - _smoothedJoyPos.z) * alphaHand;
    }

    // Metric vector from Desktop camera (0, deskCameraHeight, -nominalStanding)
    final pcOriginInUserWorld = vm.Vector3(
      0.0,
      deskCameraHeightMeters,
      -nominalStandingDistanceMeters,
    );
    final solvedDcv = (_smoothedVisorPos - pcOriginInUserWorld).length.clamp(
      0.30,
      5.0,
    );
    final solvedDcj = (_smoothedJoyPos - pcOriginInUserWorld).length.clamp(
      0.22,
      5.0,
    );
    _smoothedDcj = solvedDcj;
    final solvedDvj = (_smoothedVisorPos - _smoothedJoyPos).length.clamp(
      0.15,
      1.20,
    );
    _smoothedDvj = solvedDvj;

    final deskClearance = math.max(
      0.05,
      (_smoothedVisorPos.z + nominalStandingDistanceMeters) -
          deskEdgeOffsetMeters,
    );

    final gesture = _classifyGesture();
    _updateFacialExpression(
      nowMicros: nowMicros,
      vision: vision,
      visorFeedback: visorFeedback,
    );
    final overallConfidence = math
        .max(
          visionConfidence,
          rfDcv != null ? 0.55 : (_leftConfidence + _rightConfidence) * 0.5,
        )
        .clamp(0.0, 1.0);

    return VrTriNodeDepthSolution(
      timestampMicros: nowMicros,
      sequence: ++_sequence,
      computerToVisorMeters: solvedDcv,
      computerToJoystickMeters: _smoothedDcj,
      visorToJoystickMeters: _smoothedDvj,
      visorRoomPosition: _smoothedVisorPos,
      joystickRoomPosition: _smoothedJoyPos,
      deskClearanceMeters: deskClearance,
      visorAzimuthRadians: visorAzimuth,
      yawAnchorCorrectionRadians: yawCorrection,
      leftHandRelHead: _smoothedLeftHand,
      rightHandRelHead: _smoothedRightHand,
      leftHandVelocity: _leftHandVel,
      rightHandVelocity: _rightHandVel,
      leftHandConfidence: _leftConfidence,
      rightHandConfidence: _rightConfidence,
      leftHandRaised: _leftRaised,
      rightHandRaised: _rightRaised,
      leftPinchStrength: _leftPinch,
      rightPinchStrength: _rightPinch,
      gesture: gesture,
      confidence: overallConfidence,
      activeModalitiesMask: effectiveModalitiesMask,
      visorFiducialLocked: visorPnpPose != null,
      joystickFiducialLocked: joyPnpPose != null,
      visorFiducialReprojectionRmsPx: visorPnpPose?.reprojectionRmsPixels,
      estimatedUserIpdMeters:
          overallConfidence >= VrTriNodeDepthSolution.minValidPoseConfidence
          ? _smoothedEstimatedIpdMeters
          : null,
      facialExpression: _activeFacialExpression,
      facialConfidence: _activeFacialConfidence,
    );
  }

  void _updateFacialExpression({
    required int nowMicros,
    required VrDesktopVisionFrame? vision,
    required VrVisorBeaconFeedback? visorFeedback,
  }) {
    var candidate = VrFacialExpression.neutral;
    var candidateConf = 0.0;

    if (vision != null &&
        vision.facialExpression != VrFacialExpression.neutral &&
        vision.facialConfidence >= 0.35) {
      final rawExpr = vision.facialExpression;
      if (rawExpr == VrFacialExpression.leftWink ||
          rawExpr == VrFacialExpression.rightWink) {
        if (_lastRawWink == rawExpr) {
          _consecutiveWinkFrames++;
        } else {
          _lastRawWink = rawExpr;
          _consecutiveWinkFrames = 1;
        }
        if (_consecutiveWinkFrames >= 2) {
          candidate = rawExpr;
          candidateConf = vision.facialConfidence;
        }
      } else {
        _lastRawWink = VrFacialExpression.neutral;
        _consecutiveWinkFrames = 0;
        candidate = rawExpr;
        candidateConf = vision.facialConfidence;
      }
    } else {
      _lastRawWink = VrFacialExpression.neutral;
      _consecutiveWinkFrames = 0;
    }

    // Fuse Visor 120 Hz IMU head oscillations (nod / shake / tilt) when the
    // headset covers the upper face and webcam expression is neutral.
    if (visorFeedback != null) {
      final q = visorFeedback.visorOrientation;
      final sinp = (2.0 * (q.w * q.x - q.z * q.y)).clamp(-1.0, 1.0);
      final pitch = math.asin(sinp);
      final yaw = math.atan2(
        2.0 * (q.w * q.y + q.x * q.z),
        1.0 - 2.0 * (q.y * q.y + q.x * q.x),
      );
      final roll = math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.x * q.x + q.z * q.z),
      );
      if (_visorImuHistory.isEmpty ||
          nowMicros - _visorImuHistory.last.tsUs >= 28000) {
        _visorImuHistory.add((
          tsUs: nowMicros,
          pitch: pitch,
          yaw: yaw,
          roll: roll,
        ));
        final cutoff = nowMicros - 850000;
        _visorImuHistory.removeWhere((s) => s.tsUs < cutoff);
      }

      if (candidate == VrFacialExpression.neutral &&
          _visorImuHistory.length >= 6) {
        var minP = _visorImuHistory.first.pitch;
        var maxP = minP;
        var minY = _visorImuHistory.first.yaw;
        var maxY = minY;
        var pitchTurns = 0;
        var yawTurns = 0;
        var prevDp = 0;
        var prevDy = 0;
        for (var i = 1; i < _visorImuHistory.length; i++) {
          final cur = _visorImuHistory[i];
          final prev = _visorImuHistory[i - 1];
          if (cur.pitch < minP) minP = cur.pitch;
          if (cur.pitch > maxP) maxP = cur.pitch;
          if (cur.yaw < minY) minY = cur.yaw;
          if (cur.yaw > maxY) maxY = cur.yaw;
          final dp = cur.pitch - prev.pitch;
          if (dp.abs() > 0.015) {
            final sp = dp > 0 ? 1 : -1;
            if (prevDp != 0 && sp != prevDp) pitchTurns++;
            prevDp = sp;
          }
          final dy = cur.yaw - prev.yaw;
          if (dy.abs() > 0.018 && dy.abs() < 1.5) {
            final sy = dy > 0 ? 1 : -1;
            if (prevDy != 0 && sy != prevDy) yawTurns++;
            prevDy = sy;
          }
        }
        final pitchRange = maxP - minP;
        final yawRange = maxY - minY;
        if (pitchRange >= 0.14 &&
            pitchTurns >= 2 &&
            pitchRange > yawRange * 1.25) {
          candidate = VrFacialExpression.headNod;
          candidateConf = (pitchRange / 0.28).clamp(0.55, 0.96);
        } else if (yawRange >= 0.18 &&
            yawRange < 2.2 &&
            yawTurns >= 2 &&
            yawRange > pitchRange * 1.25) {
          candidate = VrFacialExpression.headShake;
          candidateConf = (yawRange / 0.34).clamp(0.55, 0.96);
        } else if (roll.abs() >= 0.25) {
          candidate = roll > 0
              ? VrFacialExpression.headTiltRight
              : VrFacialExpression.headTiltLeft;
          candidateConf = ((roll.abs() - 0.20) / 0.25).clamp(0.50, 0.92);
        }
      }
    }

    if (candidate != VrFacialExpression.neutral) {
      _activeFacialExpression = candidate;
      _activeFacialConfidence = candidateConf.clamp(0.0, 1.0);
      _lastFacialHoldUntilUs = nowMicros + 280000;
    } else if (nowMicros >= _lastFacialHoldUntilUs) {
      _activeFacialExpression = VrFacialExpression.neutral;
      _activeFacialConfidence = 0.0;
    }
  }

  /// Builds a [CalibratedCameraView] (`id: 'pc_faro'`) for the desktop webcam
  /// in user room coordinates (`+X` user-right, `+Y` up above floor, `+Z` away
  /// from desk relative to `nominalStandingDistanceMeters`).
  CalibratedCameraView buildDesktopCalibratedCameraView({
    int widthPx = 1280,
    int heightPx = 720,
  }) {
    final intr = CameraDistortionIntrinsics.fromHorizontalFov(
      widthPx: widthPx,
      heightPx: heightPx,
      horizontalFovDegrees: webcamHorizontalFovDeg,
      k1: webcamK1,
      k2: webcamK2,
      p1: webcamP1,
      p2: webcamP2,
    );
    final cp = math.cos(_webcamPitchRad);
    final sp = math.sin(_webcamPitchRad);
    final cr = math.cos(_webcamRollRad);
    final sr = math.sin(_webcamRollRad);

    // Exact orthogonal inverse of KabschHorizonAligner.levelCameraPointToHorizon:
    final rot = vm.Matrix3.columns(
      vm.Vector3(-cr, sr, 0.0),
      vm.Vector3(-sr * cp, -cr * cp, sp),
      vm.Vector3(sr * sp, cr * sp, cp),
    );
    final camOriginRoom = vm.Vector3(
      0.0,
      deskCameraHeightMeters,
      -nominalStandingDistanceMeters,
    );
    final trans = -rot.transformed(camOriginRoom);
    return CalibratedCameraView(
      id: 'pc_faro',
      intrinsics: intr,
      worldToCameraRotation: rot,
      worldToCameraTranslation: trans,
    );
  }

  VrBeaconHandGesture _classifyGesture() {
    if (_rightConfidence >= 0.30 && _rightPinch >= 0.72) {
      return VrBeaconHandGesture.rightPinch;
    }
    if (_leftConfidence >= 0.30 && _leftPinch >= 0.72) {
      return VrBeaconHandGesture.leftPinch;
    }
    if (_waveOscillations >= 2 && (_leftRaised || _rightRaised)) {
      return VrBeaconHandGesture.wave;
    }
    if (_leftRaised &&
        _leftConfidence >= 0.30 &&
        _rightRaised &&
        _rightConfidence >= 0.30) {
      return VrBeaconHandGesture.bothRaised;
    }
    if (_rightRaised && _rightConfidence >= 0.30) {
      return VrBeaconHandGesture.rightRaised;
    }
    if (_leftRaised && _leftConfidence >= 0.30) {
      return VrBeaconHandGesture.leftRaised;
    }
    return VrBeaconHandGesture.none;
  }
}
