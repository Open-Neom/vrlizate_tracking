import 'dart:async';
import 'dart:convert';
import 'dart:io';
import 'dart:typed_data';

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:vrlizate_assistant/vrlizate_assistant.dart';
import 'package:vrlizate_tracking/vrlizate_tracking.dart';
import 'package:vector_math/vector_math.dart' as vm;
import 'package:vrlizate/vrlizate.dart'
    show
        VrCompactGamepadCodec,
        VrCompactGamepadFrame,
        VrRemoteBinaryCodec,
        VrRemotePoseFrame;
import 'package:vrlizate_joystick/vrlizate_joystick.dart';

class _FakeUptimeWebcamPlatform implements VrDesktopVisionPlatform {
  final StreamController<VrDesktopVisionFrame> _controller =
      StreamController<VrDesktopVisionFrame>.broadcast();
  bool capturing = false;

  @override
  Stream<VrDesktopVisionFrame> get frames => _controller.stream;

  @override
  Future<bool> isSupported() async => true;

  @override
  Future<Map<String, dynamic>?> readCapabilities() async => const {
    'hasWebcam': true,
  };

  @override
  Future<bool> startCapture() async {
    capturing = true;
    return true;
  }

  @override
  Future<void> stopCapture() async {
    capturing = false;
  }

  void pushFrame(VrDesktopVisionFrame frame) {
    if (capturing && !_controller.isClosed) {
      _controller.add(frame);
    }
  }

  Future<void> dispose() => _controller.close();
}

void main() {
  TestWidgetsFlutterBinding.ensureInitialized();
  HttpOverrides.global = null;

  test(
    'Native webcam frames with OS systemUptime epoch are never overwritten by syntheticVision',
    () async {
      final fakeCam = _FakeUptimeWebcamPlatform();
      var stationClockUs = 100000;
      final station = VrDesktopBeaconStation(
        visionPlatform: fakeCam,
        discoveryPort: 0,
        enableMediaVault: true,
        monotonicMicroseconds: () => stationClockUs,
      );
      addTearDown(() async {
        station.dispose();
        await fakeCam.dispose();
      });

      await station.start(enableWebcam: true);
      expect(station.webcamNativeActive, isTrue);

      // Simulate macOS ProcessInfo.systemUptime (~120 hours = 4.32e11 us)
      const osUptimeMicros = 432000000000;
      fakeCam.pushFrame(
        const VrDesktopVisionFrame(
          timestampMicros: osUptimeMicros,
          mirrored: true,
          inferenceMs: 9.4,
          nose: VrDesktopKeypoint2D(0.72, 0.30, 0.96),
          leftShoulder: VrDesktopKeypoint2D(0.58, 0.52, 0.95),
          rightShoulder: VrDesktopKeypoint2D(0.86, 0.52, 0.95),
          rightWrist: VrDesktopKeypoint2D(0.88, 0.22, 0.92),
          rightThumbTip: VrDesktopKeypoint2D(0.87, 0.18, 0.91),
          rightIndexTip: VrDesktopKeypoint2D(0.872, 0.182, 0.91),
        ),
      );
      await Future<void>.delayed(const Duration(milliseconds: 50));
      stationClockUs += 32000;
      await Future<void>.delayed(const Duration(milliseconds: 40));

      expect(station.isNativeVisionLive, isTrue);
      expect(station.nativeVisionFramesReceived, 1);
      expect(station.latestVisionFrame?.timestampMicros, osUptimeMicros);
      expect(station.latestSolution, isNotNull);
      expect(station.latestSolution!.visorRoomPosition.x, greaterThan(0.05));

      // Verify /api/pose.json reports native_webcam telemetry
      final poseUri = station.mediaVault.lanBaseUri!.resolve('api/pose.json');
      final httpClient = HttpClient();
      addTearDown(() => httpClient.close(force: true));
      final req = await httpClient.getUrl(poseUri);
      final resp = await req.close();
      final body = await utf8.decodeStream(resp);
      final map = jsonDecode(body) as Map<String, dynamic>;
      expect(map['visionSource'], 'native_webcam');
      expect((map['webcamDetected'] as Map)['upperBody'], isTrue);
      expect((map['webcamDetected'] as Map)['rightHand'], isTrue);
    },
  );

  test(
    'Full 3D Set coordination: Joystick <-> PC Faro <-> Visor relays 120 Hz frames and control messages',
    () async {
      final station = VrDesktopBeaconStation(
        discoveryPort: 0,
        enableMediaVault: true,
      );
      final visorClient = VrDesktopBeaconClient(discoveryPort: 0);
      final visorRemote = VrRemoteControllerService();
      VrBeaconJoystickControllerLink? joystickLink;

      addTearDown(() async {
        await joystickLink?.close();
        visorRemote.dispose();
        visorClient.dispose();
        station.dispose();
      });

      await station.start(enableWebcam: false);
      final baseInvite = station.pairingInvite!;
      final loopbackInvite = VrDesktopBeaconPairingInvite(
        stationId: baseInvite.stationId,
        stationName: baseInvite.stationName,
        host: '127.0.0.1',
        port: baseInvite.port,
        pin: baseInvite.pin,
      );

      // 1. Connect Joystick (Moto Edge role) to PC 3D Set Coordinator
      final joyUri = loopbackInvite.toJoystickUri().toString();
      final parsedJoyInvite = VrDesktopBeaconPairingInvite.tryParse(joyUri)!;
      expect(parsedJoyInvite.isJoystickRole, isTrue);

      joystickLink = await VrBeaconJoystickControllerLink.connect(
        parsedJoyInvite,
      );
      expect(station.isDirectJoystickConnected, isTrue);

      // 2. Connect Visor (Galaxy S25 role) to PC Faro
      final visorOk = await visorClient.connectWithInvite(loopbackInvite);
      expect(visorOk, isTrue);
      expect(station.isVisorConnected, isTrue);

      // 3. Send binary pose packet from Joystick -> PC -> Visor
      const codec = VrRemoteBinaryCodec();
      final packet1 = codec.encodePose(
        VrRemotePoseFrame(
          sequence: 1,
          senderTimestampMicroseconds: 1000,
          orientation: vm.Quaternion.identity(),
          touchX: (0.65 + 1.0) * 0.5,
          touchY: (-0.40 + 1.0) * 0.5,
          buttonsBitset: 0x0001 | 0x0004,
        ),
      );
      joystickLink.add(packet1);

      await Future<void>.delayed(const Duration(milliseconds: 60));

      expect(station.joystickFramesCoordinated, greaterThanOrEqualTo(1));
      expect(
        station.coordinatorJoystickMirror.latestState.stickX,
        closeTo(0.65, 0.02),
      );
      expect(visorClient.hasCoordinatedJoystick, isTrue);

      // Attach Visor's coordinatedControllerLink to Visor's VrRemoteControllerService
      final attached = visorRemote.attachControllerLink(
        visorClient.coordinatedControllerLink,
        sessionToken: visorRemote.sessionToken,
      );
      expect(attached, isTrue);

      // Track control messages arriving back on Joystick from Visor -> PC -> Joystick
      final incomingOnJoystick = <String>[];
      final joySub = joystickLink.messages.listen((msg) {
        if (msg is String) incomingOnJoystick.add(msg);
      });
      addTearDown(joySub.cancel);

      visorRemote.requestControllerMode(
        RemoteControllerMode.driving,
        actions: const {'A': 'Turbo'},
      );

      final packet2 = codec.encodePose(
        VrRemotePoseFrame(
          sequence: 2,
          senderTimestampMicroseconds: 9000,
          orientation: vm.Quaternion.identity(),
          touchX: (-0.75 + 1.0) * 0.5,
          touchY: (0.85 + 1.0) * 0.5,
          buttonsBitset: 0x0008,
        ),
      );
      joystickLink.add(packet2);

      await Future<void>.delayed(const Duration(milliseconds: 80));

      expect(visorRemote.latestState.stickX, closeTo(-0.75, 0.02));
      expect(visorRemote.latestState.stickY, closeTo(0.85, 0.02));
      expect(visorRemote.latestState.btnB, isTrue);
      expect(incomingOnJoystick.any((m) => m.contains('"driving"')), isTrue);
    },
  );

  test(
    '68-byte VrCompactGamepadFrame (Smallest-Three quaternion) relays zero-JSON across Joystick -> PC -> Visor',
    () async {
      final station = VrDesktopBeaconStation(
        discoveryPort: 0,
        enableMediaVault: false,
      );
      final visorClient = VrDesktopBeaconClient(discoveryPort: 0);
      final visorRemote = VrRemoteControllerService();
      VrBeaconJoystickControllerLink? joystickLink;

      addTearDown(() async {
        await joystickLink?.close();
        visorRemote.dispose();
        visorClient.dispose();
        station.dispose();
      });

      await station.start(enableWebcam: false);
      final baseInvite = station.pairingInvite!;
      final loopbackInvite = VrDesktopBeaconPairingInvite(
        stationId: baseInvite.stationId,
        stationName: baseInvite.stationName,
        host: '127.0.0.1',
        port: baseInvite.port,
        pin: baseInvite.pin,
      );

      joystickLink = await VrBeaconJoystickControllerLink.connect(
        loopbackInvite,
      );
      expect(await visorClient.connectWithInvite(loopbackInvite), isTrue);
      expect(
        visorRemote.attachControllerLink(
          visorClient.coordinatedControllerLink,
          sessionToken: visorRemote.sessionToken,
        ),
        isTrue,
      );

      final compactFrame = VrCompactGamepadFrame(
        sequence: 1,
        senderTimestampMicroseconds: 17000,
        orientation: vm.Quaternion.axisAngle(vm.Vector3(0, 1, 0), 0.35),
        angularVelocity: vm.Vector3(0.4, -0.2, 0.0),
        stickX: 0.42,
        stickY: -0.68,
        lookX: -0.55,
        lookY: 0.25,
        laserX: 0.18,
        laserY: -0.12,
        laserSlideActive: true,
        trigger: true,
        btnA: true,
        btnZL: true,
        btnZR: true,
        pointerPoseReady: true,
        passthroughActive: true,
        passthroughMix: 0.50,
      );
      final encodedRelay = VrDesktopBeaconWire.encodeAuthenticatedRelayFrame(
        kind: VrDesktopBeaconWire.joystickControllerFrameKind,
        key: List<int>.filled(32, 7),
        frame: compactFrame,
      )!;
      expect(
        encodedRelay.length,
        VrDesktopBeaconWire.compactGamepadPacketBytes,
      );
      expect(encodedRelay.length, 68);

      joystickLink.add(compactFrame);
      await Future<void>.delayed(const Duration(milliseconds: 60));

      expect(joystickLink.compactBinaryFramesSent, 1);
      expect(station.compactJoystickFramesCoordinated, 1);
      expect(visorRemote.latestState.stickX, closeTo(0.42, 0.01));
      expect(visorRemote.latestState.stickY, closeTo(-0.68, 0.01));
      expect(visorRemote.latestState.turnRate, closeTo(-0.55, 0.01));
      expect(visorRemote.latestState.laserSlideActive, isTrue);
      expect(visorRemote.latestState.laserX, closeTo(0.18, 0.01));
      expect(visorRemote.latestState.isTriggerPressed, isTrue);
      expect(visorRemote.latestState.btnA, isTrue);
      expect(visorRemote.latestState.btnZL, isTrue);
      expect(visorRemote.latestState.btnZR, isTrue);
      expect(visorRemote.latestState.passthroughMix, closeTo(0.50, 0.02));

      final release = visorRemote.onState.firstWhere(
        (state) => !state.isNeutralized && !state.btnZL && !state.btnZR,
      );
      joystickLink.add(
        VrCompactGamepadFrame(
          sequence: 2,
          senderTimestampMicroseconds: 25000,
          orientation: vm.Quaternion.identity(),
          angularVelocity: vm.Vector3.zero(),
        ),
      );
      final released = await release.timeout(const Duration(seconds: 2));
      expect(released.btnA, isFalse);
    },
  );

  test(
    'VrBeaconDeadReckoningGate suppresses >80% redundant 62.5 Hz packets and extrapolates 120 Hz hands',
    () {
      final gate = VrBeaconDeadReckoningGate();

      VrTriNodeDepthSolution buildSample({
        required int seq,
        required int tsUs,
        required vm.Vector3 head,
        required vm.Vector3 rightHand,
        required vm.Vector3 rightVel,
        VrBeaconHandGesture gesture = VrBeaconHandGesture.none,
      }) => VrTriNodeDepthSolution(
        timestampMicros: tsUs,
        sequence: seq,
        computerToVisorMeters: 1.48,
        computerToJoystickMeters: 1.22,
        visorToJoystickMeters: 0.44,
        visorRoomPosition: head,
        joystickRoomPosition: vm.Vector3(0.1, 1.05, -0.2),
        deskClearanceMeters: 1.48,
        visorAzimuthRadians: 0.0,
        yawAnchorCorrectionRadians: 0.0,
        leftHandRelHead: vm.Vector3(-0.2, -0.25, -0.45),
        rightHandRelHead: rightHand,
        rightHandVelocity: rightVel,
        leftHandConfidence: 0.85,
        rightHandConfidence: 0.92,
        leftHandRaised: false,
        rightHandRaised: true,
        leftPinchStrength: 0.0,
        rightPinchStrength: 0.0,
        gesture: gesture,
        confidence: 0.90,
      );

      // Simulate 1.0 second at 62.5 Hz (62 ticks of 16 ms) while user is stationary
      var sent = 0;
      for (var i = 0; i < 62; i++) {
        final tUs = 1000000 + i * 16000;
        final s = buildSample(
          seq: i + 1,
          tsUs: tUs,
          head: vm.Vector3(0.0, 1.60, 0.0),
          rightHand: vm.Vector3(0.22, -0.10, -0.48),
          rightVel: vm.Vector3.zero(),
        );
        if (gate.shouldTransmit(s, nowMicros: tUs)) {
          sent++;
        }
      }
      // 62 ticks at 16ms -> only initial + ~6 sync heartbeats (160ms) = 7 packets!
      expect(sent, lessThanOrEqualTo(8));
      expect(gate.suppressionRatio, greaterThan(0.84));

      // Sudden hand movement of +4 cm immediately passes the gate on the very next tick!
      final suddenUs = 1000000 + 62 * 16000;
      final sudden = buildSample(
        seq: 63,
        tsUs: suddenUs,
        head: vm.Vector3(0.0, 1.60, 0.0),
        rightHand: vm.Vector3(0.26, -0.10, -0.48),
        rightVel: vm.Vector3(0.80, 0.0, 0.0),
      );
      expect(gate.shouldTransmit(sudden, nowMicros: suddenUs), isTrue);

      // Receiver side: ingest sudden packet and predict 8.33 ms later (120 Hz intermediate frame)
      gate.ingestReceived(sudden, receivedAtMicros: suddenUs);
      final pred120Hz = gate.predictAt(suddenUs + 8333)!;
      expect(pred120Hz.rightHandRelHead.x, greaterThan(0.265));
      expect(pred120Hz.rightHandRelHead.x, lessThan(0.275));
    },
  );

  test(
    'Estación Faro preserves stationId/PIN across restarts and Joystick + Visor recover without mutual timeout deadlock',
    () async {
      final station1 = VrDesktopBeaconStation(
        discoveryPort: 0,
        enableMediaVault: false,
        initialStationId: 'faro_sync_test',
        initialPin: '130315',
      );
      await station1.start(enableWebcam: false);
      final invite1 = station1.pairingInvite!;
      expect(invite1.stationId, 'faro_sync_test');
      expect(invite1.pin, '130315');

      var visorClockUs = 1000000;
      final visorClient = VrDesktopBeaconClient(
        discoveryPort: 0,
        monotonicMicroseconds: () => visorClockUs,
      );
      addTearDown(visorClient.dispose);

      final loopbackInvite = VrDesktopBeaconPairingInvite(
        stationId: invite1.stationId,
        stationName: invite1.stationName,
        host: '127.0.0.1',
        port: invite1.port,
        pin: invite1.pin,
      );
      expect(await visorClient.connectWithInvite(loopbackInvite), isTrue);
      expect(visorClient.rememberedInvite?.stationId, 'faro_sync_test');

      // Advance simulated Visor clock past 5 seconds (>4.5s old timeout) while
      // station1 continues sending 1 Hz keepalives: link MUST stay connected
      // and recover immediately without mutual silence deadlock!
      visorClockUs += 5200000;
      await Future<void>.delayed(const Duration(milliseconds: 1150));
      expect(visorClient.isConnected, isTrue);
      expect(station1.isVisorConnected, isTrue);

      station1.dispose();

      // Restart Estación Faro on a brand-new UDP port with the same stationId & PIN
      final station2 = VrDesktopBeaconStation(
        discoveryPort: 0,
        enableMediaVault: false,
        initialStationId: 'faro_sync_test',
        initialPin: '130315',
      );
      addTearDown(station2.dispose);
      await station2.start(enableWebcam: false);
      expect(station2.pairingInvite!.stationId, 'faro_sync_test');
      expect(station2.pairingInvite!.pin, '130315');

      final joyInvite2 = VrDesktopBeaconPairingInvite(
        stationId: 'faro_sync_test',
        stationName: station2.stationName,
        host: '127.0.0.1',
        port: station2.boundPort!,
        pin: '130315',
        role: 'joystick',
      );
      final joyLink = await VrBeaconJoystickControllerLink.connect(joyInvite2);
      addTearDown(joyLink.close);
      expect(station2.isDirectJoystickConnected, isTrue);
    },
  );

  test(
    'Hands-free facial expressions pack into 140-byte VRB1 datagram with 0-latency pass-through and Estación Faro relays Spanish voice commands over HMAC UDP',
    () async {
      final station = VrDesktopBeaconStation(
        discoveryPort: 0,
        enableMediaVault: false,
      );
      final visorClient = VrDesktopBeaconClient(discoveryPort: 0);
      addTearDown(() {
        visorClient.dispose();
        station.dispose();
      });

      await station.start(enableWebcam: false);
      final invite = station.pairingInvite!;
      final loopbackInvite = VrDesktopBeaconPairingInvite(
        stationId: invite.stationId,
        stationName: invite.stationName,
        host: '127.0.0.1',
        port: invite.port,
        pin: invite.pin,
      );
      expect(await visorClient.connectWithInvite(loopbackInvite), isTrue);

      final receivedVoices = <VrBeaconVoiceCommandPacket>[];
      final voiceSub = visorClient.voiceCommands.listen(receivedVoices.add);
      addTearDown(voiceSub.cancel);

      // 1) Pulse eyebrowsRaised on Estación Faro -> verify immediate 140-byte VRB1 delivery
      station.pulseSimulatedFacialExpression(VrFacialExpression.eyebrowsRaised);
      await Future<void>.delayed(const Duration(milliseconds: 65));
      expect(
        visorClient.latestSolution?.facialExpression,
        VrFacialExpression.eyebrowsRaised,
      );
      expect(
        visorClient.latestSolution?.facialConfidence ?? 0.0,
        greaterThan(0.85),
      );

      // 2) Submit natural Spanish voice commands on Estación Faro -> verify HMAC relay & deduplication
      final mallGuidance = station.submitVoiceCommand('Abre Plaza Artz');
      expect(mallGuidance.actionType, VrVoiceActionType.launchExperience);
      expect(mallGuidance.targetDemoId, 'mall_plaza_artz_mx');

      await Future<void>.delayed(const Duration(milliseconds: 80));
      expect(receivedVoices, hasLength(1));
      expect(receivedVoices.single.transcript, 'Abre Plaza Artz');

      // 3) Verify Itzli Spanish locomotion, world & UI routing without Joystick
      final itzli = const VrDefaultSpanishVoiceRouter();
      final walk = itzli.interpretRequest('camina hacia adelante bastante');
      expect(walk.actionType, VrVoiceActionType.moveInSpace);
      expect(walk.motionKind, VrVoiceMotionKind.walkForward);
      expect(walk.motionDurationSeconds, closeTo(4.0, 0.01));

      final turn = itzli.interpretRequest('date media vuelta');
      expect(turn.actionType, VrVoiceActionType.moveInSpace);
      expect(turn.motionKind, VrVoiceMotionKind.turnAround);

      final stop = itzli.interpretRequest('detente');
      expect(stop.actionType, VrVoiceActionType.moveInSpace);
      expect(stop.motionKind, VrVoiceMotionKind.stop);

      final menuOpen = itzli.interpretRequest('abre el menú');
      expect(menuOpen.actionType, VrVoiceActionType.uiCommand);
      expect(menuOpen.uiCommand, VrVoiceUiCommand.openMenu);

      final solar = itzli.interpretRequest('llévame a Ciudad Solar');
      expect(solar.actionType, VrVoiceActionType.launchExperience);
      expect(solar.targetDemoId, 'ecosystems_solar_city');

      // 4) Verify Visor IMU head-nod fusion when webcam face is occluded (neutral)
      final solver = VrTriNodeDepthSolver();
      final pitches = [0.0, 0.12, -0.10, 0.13, -0.09, 0.11, 0.0];
      VrTriNodeDepthSolution? fused;
      for (var i = 0; i < pitches.length; i++) {
        final q = vm.Quaternion.axisAngle(vm.Vector3(1, 0, 0), pitches[i]);
        fused = solver.solve(
          nowMicros: 100000 + i * 45000,
          vision: null,
          visorFeedback: VrVisorBeaconFeedback(
            timestampMicros: 100000 + i * 45000,
            visorOrientation: q,
          ),
        );
      }
      expect(fused?.facialExpression, VrFacialExpression.headNod);
    },
  );

  test(
    'SAIA-FARO-006: Visor-first mode replay on Joystick helloAck + ultra-short 1ms A tap preserves DOWN/UP edge order over UDP',
    () async {
      final station = VrDesktopBeaconStation(
        discoveryPort: 0,
        enableMediaVault: false,
      );
      final visorClient = VrDesktopBeaconClient(discoveryPort: 0);
      final visorRemote = VrRemoteControllerService();
      VrBeaconJoystickControllerLink? joystickLink;

      addTearDown(() async {
        await joystickLink?.close();
        visorRemote.dispose();
        visorClient.dispose();
        station.dispose();
      });

      await station.start(enableWebcam: false);
      final baseInvite = station.pairingInvite!;
      final loopbackInvite = VrDesktopBeaconPairingInvite(
        stationId: baseInvite.stationId,
        stationName: baseInvite.stationName,
        host: '127.0.0.1',
        port: baseInvite.port,
        pin: baseInvite.pin,
      );

      // 1) Visor connects FIRST and requests driving BEFORE Joystick joins!
      expect(await visorClient.connectWithInvite(loopbackInvite), isTrue);
      expect(
        visorRemote.attachControllerLink(
          visorClient.coordinatedControllerLink,
          sessionToken: visorRemote.sessionToken,
        ),
        isTrue,
      );
      visorRemote.requestControllerMode(
        RemoteControllerMode.driving,
        actions: const {'A': 'Acelerar'},
      );
      await Future<void>.delayed(const Duration(milliseconds: 50));

      // 2) Now Joystick joins role=joystick -> station replays active driving mode!
      var fakeJoyClockUs = 1000000;
      joystickLink = await VrBeaconJoystickControllerLink.connect(
        loopbackInvite,
        monotonicMicroseconds: () => fakeJoyClockUs,
      );
      final receivedModesOnJoystick = <VrControllerModeRequest>[];
      final joySub = joystickLink.messages.listen((msg) {
        final req = VrControllerModeRequest.tryParse(msg);
        if (req != null) receivedModesOnJoystick.add(req);
      });
      addTearDown(joySub.cancel);

      await Future<void>.delayed(const Duration(milliseconds: 80));
      expect(
        receivedModesOnJoystick.any(
          (r) => r.mode == RemoteControllerMode.driving,
        ),
        isTrue,
      );

      // Switch back to joystick mode and ACK revision 2 so discrete A taps are observed.
      visorRemote.requestControllerMode(RemoteControllerMode.joystick);
      await Future<void>.delayed(const Duration(milliseconds: 60));
      final joyModeRev = receivedModesOnJoystick.last.revision;

      final observedBtnAEdges = <bool>[];
      final stateSub = visorRemote.onState.listen((s) {
        if (observedBtnAEdges.isEmpty || observedBtnAEdges.last != s.btnA) {
          observedBtnAEdges.add(s.btnA);
        }
      });
      addTearDown(stateSub.cancel);

      // Neutral ACK for active joystick revision
      joystickLink.add(
        VrCompactGamepadFrame(
          sequence: 1,
          senderTimestampMicroseconds: fakeJoyClockUs,
          orientation: vm.Quaternion.identity(),
          angularVelocity: vm.Vector3.zero(),
          hostModeRevision: joyModeRev,
        ),
      );
      await Future<void>.delayed(const Duration(milliseconds: 30));

      // 3) Ultra-short 1 ms synthetic tap: DOWN at t=0, UP at t=+1000 us!
      fakeJoyClockUs += 16000;
      joystickLink.add(
        VrCompactGamepadFrame(
          sequence: 2,
          senderTimestampMicroseconds: fakeJoyClockUs,
          orientation: vm.Quaternion.identity(),
          angularVelocity: vm.Vector3.zero(),
          btnA: true,
          hostModeRevision: joyModeRev,
        ),
      );
      fakeJoyClockUs += 1000; // 1 ms later!
      joystickLink.add(
        VrCompactGamepadFrame(
          sequence: 3,
          senderTimestampMicroseconds: fakeJoyClockUs,
          orientation: vm.Quaternion.identity(),
          angularVelocity: vm.Vector3.zero(),
          btnA: false,
          hostModeRevision: joyModeRev,
        ),
      );

      // Wait >24 ms so deferred release completes cleanly after DOWN!
      await Future<void>.delayed(const Duration(milliseconds: 70));
      expect(observedBtnAEdges, containsAllInOrder(<bool>[false, true, false]));
    },
  );

  test(
    'SAIA-FARO-007b: Lossy UDP proxy dropping 1st DOWN (seq 2) and 1st UP (seq 3) recovers full A tap via idempotent same-sequence retransmits without double-clicks',
    () async {
      final station = VrDesktopBeaconStation(
        discoveryPort: 0,
        enableMediaVault: false,
      );
      final visorClient = VrDesktopBeaconClient(discoveryPort: 0);
      final visorRemote = VrRemoteControllerService();
      VrBeaconJoystickControllerLink? joystickLink;

      await station.start(enableWebcam: false);
      final stationPort = station.boundPort!;
      final proxySocket = await RawDatagramSocket.bind(
        InternetAddress.loopbackIPv4,
        0,
      );
      InternetAddress? joystickAddr;
      int? joystickPort;
      final droppedFirstSequences = <int>{};
      final forwardedSequences = <int>[];

      final proxySub = proxySocket.listen((event) {
        if (event != RawSocketEvent.read) return;
        Datagram? dg;
        while ((dg = proxySocket.receive()) != null) {
          final data = dg!.data;
          if (dg.port == stationPort) {
            if (joystickAddr != null && joystickPort != null) {
              proxySocket.send(data, joystickAddr!, joystickPort!);
            }
          } else {
            joystickAddr = dg.address;
            joystickPort = dg.port;
            if (data.length == VrDesktopBeaconWire.compactGamepadPacketBytes &&
                data[4] == VrDesktopBeaconWire.joystickControllerFrameKind &&
                data[5] == VrDesktopBeaconWire.relayCompactGamepadFrameType) {
              final decoded = const VrCompactGamepadCodec().tryDecode(
                Uint8List.sublistView(data, 6, 52),
              );
              if (decoded != null) {
                if ((decoded.sequence == 2 || decoded.sequence == 3) &&
                    !droppedFirstSequences.contains(decoded.sequence)) {
                  droppedFirstSequences.add(decoded.sequence);
                  continue; // Deliberately drop first DOWN and first UP!
                }
                forwardedSequences.add(decoded.sequence);
              }
            }
            proxySocket.send(data, InternetAddress.loopbackIPv4, stationPort);
          }
        }
      });

      addTearDown(() async {
        await joystickLink?.close();
        await proxySub.cancel();
        proxySocket.close();
        visorRemote.dispose();
        visorClient.dispose();
        station.dispose();
      });

      final baseInvite = station.pairingInvite!;
      final visorInvite = VrDesktopBeaconPairingInvite(
        stationId: baseInvite.stationId,
        stationName: baseInvite.stationName,
        host: '127.0.0.1',
        port: stationPort,
        pin: baseInvite.pin,
      );
      final proxiedJoyInvite = VrDesktopBeaconPairingInvite(
        stationId: baseInvite.stationId,
        stationName: baseInvite.stationName,
        host: '127.0.0.1',
        port: proxySocket.port,
        pin: baseInvite.pin,
        role: 'joystick',
      );

      expect(await visorClient.connectWithInvite(visorInvite), isTrue);
      expect(
        visorRemote.attachControllerLink(
          visorClient.coordinatedControllerLink,
          sessionToken: visorRemote.sessionToken,
        ),
        isTrue,
      );

      var fakeJoyClockUs = 1000000;
      joystickLink = await VrBeaconJoystickControllerLink.connect(
        proxiedJoyInvite,
        monotonicMicroseconds: () => fakeJoyClockUs,
      );

      final receivedModesOnJoystick = <VrControllerModeRequest>[];
      final joySub = joystickLink.messages.listen((msg) {
        final req = VrControllerModeRequest.tryParse(msg);
        if (req != null) receivedModesOnJoystick.add(req);
      });
      addTearDown(joySub.cancel);

      visorRemote.requestControllerMode(RemoteControllerMode.joystick);
      await Future<void>.delayed(const Duration(milliseconds: 60));
      final joyModeRev = receivedModesOnJoystick.last.revision;

      final observedBtnAEdges = <bool>[];
      final stateSub = visorRemote.onState.listen((s) {
        if (observedBtnAEdges.isEmpty || observedBtnAEdges.last != s.btnA) {
          observedBtnAEdges.add(s.btnA);
        }
      });
      addTearDown(stateSub.cancel);

      // Neutral ACK (sequence 1)
      joystickLink.add(
        VrCompactGamepadFrame(
          sequence: 1,
          senderTimestampMicroseconds: fakeJoyClockUs,
          orientation: vm.Quaternion.identity(),
          angularVelocity: vm.Vector3.zero(),
          hostModeRevision: joyModeRev,
        ),
      );
      await Future<void>.delayed(const Duration(milliseconds: 30));

      // Ultra-short 1 ms tap where BOTH the 1st DOWN (seq 2) AND 1st UP (seq 3) are dropped on Wi-Fi!
      fakeJoyClockUs += 16000;
      joystickLink.add(
        VrCompactGamepadFrame(
          sequence: 2,
          senderTimestampMicroseconds: fakeJoyClockUs,
          orientation: vm.Quaternion.identity(),
          angularVelocity: vm.Vector3.zero(),
          btnA: true,
          hostModeRevision: joyModeRev,
        ),
      );
      fakeJoyClockUs += 1000;
      joystickLink.add(
        VrCompactGamepadFrame(
          sequence: 3,
          senderTimestampMicroseconds: fakeJoyClockUs,
          orientation: vm.Quaternion.identity(),
          angularVelocity: vm.Vector3.zero(),
          btnA: false,
          hostModeRevision: joyModeRev,
        ),
      );

      // Wait ~90 ms so +7/+15 ms backups for seq 2 AND deferred +23 ms (+7/+15 ms backups) for seq 3 complete!
      await Future<void>.delayed(const Duration(milliseconds: 95));

      expect(droppedFirstSequences, equals(<int>{2, 3}));
      // End-to-end Visor sequence ACKs (`compactSeqAckKind`) acknowledge the 1st
      // retry of seq 2 and 1st retry of seq 3 without blind post-ACK spam.
      expect(joystickLink.redundantEdgeDatagramsSent, greaterThanOrEqualTo(2));
      expect(forwardedSequences, containsAllInOrder(<int>[1, 2, 3]));
      // Station & Visor deduplicate same-sequence backups: exact 3 logical compact frames and exact single [false, true, false] tap!
      expect(station.compactJoystickFramesCoordinated, 3);
      expect(observedBtnAEdges, equals(<bool>[false, true, false]));
    },
  );

  test(
    'SAIA-FARO-008 #1: Consecutive ultra-short A taps recover exact [false, true, false, true, false] FIFO edges under 1st UP3 loss on both Joystick->PC and PC->Visor legs',
    () async {
      int? tryPeekCompactSeq(Uint8List data) {
        if (data.length == VrDesktopBeaconWire.compactGamepadPacketBytes &&
            data[4] == VrDesktopBeaconWire.joystickControllerFrameKind &&
            data[5] == VrDesktopBeaconWire.relayCompactGamepadFrameType) {
          return const VrCompactGamepadCodec()
              .tryDecode(Uint8List.sublistView(data, 6, 52))
              ?.sequence;
        }
        return null;
      }

      for (final dropLeg in <String>['joystick_to_pc', 'pc_to_visor']) {
        final station = VrDesktopBeaconStation(
          discoveryPort: 0,
          enableMediaVault: false,
        );
        final visorClient = VrDesktopBeaconClient(discoveryPort: 0);
        final visorRemote = VrRemoteControllerService();
        VrBeaconJoystickControllerLink? joystickLink;

        await station.start(enableWebcam: false);
        final stationPort = station.boundPort!;
        final joyProxy = await RawDatagramSocket.bind(
          InternetAddress.loopbackIPv4,
          0,
        );
        final visorProxy = await RawDatagramSocket.bind(
          InternetAddress.loopbackIPv4,
          0,
        );
        InternetAddress? joyAddr;
        int? joyPort;
        InternetAddress? visorAddr;
        int? visorPort;
        var droppedFirstUp3 = false;

        final joyProxySub = joyProxy.listen((event) {
          if (event != RawSocketEvent.read) return;
          Datagram? dg;
          while ((dg = joyProxy.receive()) != null) {
            final data = dg!.data;
            if (dg.port == stationPort) {
              if (joyAddr != null && joyPort != null) {
                joyProxy.send(data, joyAddr!, joyPort!);
              }
            } else {
              joyAddr = dg.address;
              joyPort = dg.port;
              if (dropLeg == 'joystick_to_pc' &&
                  tryPeekCompactSeq(data) == 3 &&
                  !droppedFirstUp3) {
                droppedFirstUp3 = true;
                continue;
              }
              joyProxy.send(data, InternetAddress.loopbackIPv4, stationPort);
            }
          }
        });

        final visorProxySub = visorProxy.listen((event) {
          if (event != RawSocketEvent.read) return;
          Datagram? dg;
          while ((dg = visorProxy.receive()) != null) {
            final data = dg!.data;
            if (dg.port == stationPort) {
              if (dropLeg == 'pc_to_visor' &&
                  tryPeekCompactSeq(data) == 3 &&
                  !droppedFirstUp3) {
                droppedFirstUp3 = true;
                continue;
              }
              if (visorAddr != null && visorPort != null) {
                visorProxy.send(data, visorAddr!, visorPort!);
              }
            } else {
              visorAddr = dg.address;
              visorPort = dg.port;
              visorProxy.send(data, InternetAddress.loopbackIPv4, stationPort);
            }
          }
        });

        try {
          final baseInvite = station.pairingInvite!;
          final visorInvite = VrDesktopBeaconPairingInvite(
            stationId: baseInvite.stationId,
            stationName: baseInvite.stationName,
            host: '127.0.0.1',
            port: visorProxy.port,
            pin: baseInvite.pin,
          );
          final joyInvite = VrDesktopBeaconPairingInvite(
            stationId: baseInvite.stationId,
            stationName: baseInvite.stationName,
            host: '127.0.0.1',
            port: joyProxy.port,
            pin: baseInvite.pin,
            role: 'joystick',
          );

          expect(await visorClient.connectWithInvite(visorInvite), isTrue);
          expect(
            visorRemote.attachControllerLink(
              visorClient.coordinatedControllerLink,
              sessionToken: visorRemote.sessionToken,
            ),
            isTrue,
          );

          var fakeJoyClockUs = 1000000;
          joystickLink = await VrBeaconJoystickControllerLink.connect(
            joyInvite,
            monotonicMicroseconds: () => fakeJoyClockUs,
          );

          final receivedModes = <VrControllerModeRequest>[];
          final joySub = joystickLink.messages.listen((msg) {
            final req = VrControllerModeRequest.tryParse(msg);
            if (req != null) receivedModes.add(req);
          });
          visorRemote.requestControllerMode(RemoteControllerMode.joystick);
          await Future<void>.delayed(const Duration(milliseconds: 60));
          final joyModeRev = receivedModes.last.revision;

          final observedBtnAEdges = <bool>[];
          final stateSub = visorRemote.onState.listen((s) {
            if (observedBtnAEdges.isEmpty || observedBtnAEdges.last != s.btnA) {
              observedBtnAEdges.add(s.btnA);
            }
          });

          // seq 1: neutral
          joystickLink.add(
            VrCompactGamepadFrame(
              sequence: 1,
              senderTimestampMicroseconds: fakeJoyClockUs,
              orientation: vm.Quaternion.identity(),
              angularVelocity: vm.Vector3.zero(),
              hostModeRevision: joyModeRev,
            ),
          );
          await Future<void>.delayed(const Duration(milliseconds: 25));

          // seq 2: DOWN_A at t=0
          fakeJoyClockUs += 16000;
          joystickLink.add(
            VrCompactGamepadFrame(
              sequence: 2,
              senderTimestampMicroseconds: fakeJoyClockUs,
              orientation: vm.Quaternion.identity(),
              angularVelocity: vm.Vector3.zero(),
              btnA: true,
              hostModeRevision: joyModeRev,
            ),
          );
          // seq 3: UP_A at t=+1ms
          fakeJoyClockUs += 1000;
          joystickLink.add(
            VrCompactGamepadFrame(
              sequence: 3,
              senderTimestampMicroseconds: fakeJoyClockUs,
              orientation: vm.Quaternion.identity(),
              angularVelocity: vm.Vector3.zero(),
              btnA: false,
              hostModeRevision: joyModeRev,
            ),
          );
          // seq 4: DOWN_A at t=+10ms (while 1st UP3 is dropped!)
          await Future<void>.delayed(const Duration(milliseconds: 10));
          fakeJoyClockUs += 9000;
          joystickLink.add(
            VrCompactGamepadFrame(
              sequence: 4,
              senderTimestampMicroseconds: fakeJoyClockUs,
              orientation: vm.Quaternion.identity(),
              angularVelocity: vm.Vector3.zero(),
              btnA: true,
              hostModeRevision: joyModeRev,
            ),
          );
          // Final UP_A seq 5 after 95ms
          await Future<void>.delayed(const Duration(milliseconds: 95));
          fakeJoyClockUs += 95000;
          joystickLink.add(
            VrCompactGamepadFrame(
              sequence: 5,
              senderTimestampMicroseconds: fakeJoyClockUs,
              orientation: vm.Quaternion.identity(),
              angularVelocity: vm.Vector3.zero(),
              btnA: false,
              hostModeRevision: joyModeRev,
            ),
          );
          await Future<void>.delayed(const Duration(milliseconds: 50));

          await stateSub.cancel();
          await joySub.cancel();
          expect(droppedFirstUp3, isTrue);
          expect(joystickLink.pendingDiscreteEdgeCount, 0);
          expect(
            joystickLink.redundantEdgeDatagramsSent,
            greaterThanOrEqualTo(1),
          );
          expect(station.compactJoystickFramesCoordinated, 5);
          expect(
            observedBtnAEdges,
            equals(<bool>[false, true, false, true, false]),
            reason: 'Expected exact FIFO two-tap edges on leg=$dropLeg',
          );
        } finally {
          await joystickLink?.close();
          await joyProxySub.cancel();
          await visorProxySub.cancel();
          joyProxy.close();
          visorProxy.close();
          visorRemote.dispose();
          visorClient.dispose();
          station.dispose();
        }
      }
    },
  );

  test(
    'SAIA-FARO-008 #2: Retained UP3 injected after DOWN4 never overwrites newer state on Station or Visor, and close() with pending discrete queue cancels retries cleanly',
    () async {
      int? tryPeekCompactSeq(Uint8List data) {
        if (data.length == VrDesktopBeaconWire.compactGamepadPacketBytes &&
            data[4] == VrDesktopBeaconWire.joystickControllerFrameKind &&
            data[5] == VrDesktopBeaconWire.relayCompactGamepadFrameType) {
          return const VrCompactGamepadCodec()
              .tryDecode(Uint8List.sublistView(data, 6, 52))
              ?.sequence;
        }
        return null;
      }

      for (final retainLeg in <String>['joystick_to_pc', 'pc_to_visor']) {
        final station = VrDesktopBeaconStation(
          discoveryPort: 0,
          enableMediaVault: false,
        );
        final visorClient = VrDesktopBeaconClient(discoveryPort: 0);
        final visorRemote = VrRemoteControllerService();
        VrBeaconJoystickControllerLink? joystickLink;

        await station.start(enableWebcam: false);
        final stationPort = station.boundPort!;
        final joyProxy = await RawDatagramSocket.bind(
          InternetAddress.loopbackIPv4,
          0,
        );
        final visorProxy = await RawDatagramSocket.bind(
          InternetAddress.loopbackIPv4,
          0,
        );
        InternetAddress? joyAddr;
        int? joyPort;
        InternetAddress? visorAddr;
        int? visorPort;
        Uint8List? retainedUp3OnJoyLeg;
        Uint8List? retainedUp3OnVisorLeg;
        var injectedLateUp3 = false;
        var blackholeJoystickOutbound = false;
        var joyPacketsAfterClose = 0;
        var linkClosedInTest = false;

        final joyProxySub = joyProxy.listen((event) {
          if (event != RawSocketEvent.read) return;
          Datagram? dg;
          while ((dg = joyProxy.receive()) != null) {
            final data = dg!.data;
            if (dg.port == stationPort) {
              if (joyAddr != null && joyPort != null) {
                joyProxy.send(data, joyAddr!, joyPort!);
              }
            } else {
              joyAddr = dg.address;
              joyPort = dg.port;
              if (linkClosedInTest) {
                joyPacketsAfterClose++;
              }
              if (blackholeJoystickOutbound) {
                continue;
              }
              final seq = tryPeekCompactSeq(data);
              if (retainLeg == 'joystick_to_pc') {
                if (seq == 3 && retainedUp3OnJoyLeg == null) {
                  retainedUp3OnJoyLeg = Uint8List.fromList(data);
                  continue; // Retain 1st UP3 until after DOWN4!
                }
                joyProxy.send(data, InternetAddress.loopbackIPv4, stationPort);
                if (seq == 4 &&
                    retainedUp3OnJoyLeg != null &&
                    !injectedLateUp3) {
                  injectedLateUp3 = true;
                  joyProxy.send(
                    retainedUp3OnJoyLeg!,
                    InternetAddress.loopbackIPv4,
                    stationPort,
                  );
                }
                continue;
              }
              joyProxy.send(data, InternetAddress.loopbackIPv4, stationPort);
            }
          }
        });

        final visorProxySub = visorProxy.listen((event) {
          if (event != RawSocketEvent.read) return;
          Datagram? dg;
          while ((dg = visorProxy.receive()) != null) {
            final data = dg!.data;
            if (dg.port == stationPort) {
              final seq = tryPeekCompactSeq(data);
              if (retainLeg == 'pc_to_visor' &&
                  visorAddr != null &&
                  visorPort != null) {
                if (seq == 3 && retainedUp3OnVisorLeg == null) {
                  retainedUp3OnVisorLeg = Uint8List.fromList(data);
                  continue; // Retain 1st UP3 on PC->Visor until after DOWN4!
                }
                visorProxy.send(data, visorAddr!, visorPort!);
                if (seq == 4 &&
                    retainedUp3OnVisorLeg != null &&
                    !injectedLateUp3) {
                  injectedLateUp3 = true;
                  visorProxy.send(
                    retainedUp3OnVisorLeg!,
                    visorAddr!,
                    visorPort!,
                  );
                }
                continue;
              }
              if (visorAddr != null && visorPort != null) {
                visorProxy.send(data, visorAddr!, visorPort!);
              }
            } else {
              visorAddr = dg.address;
              visorPort = dg.port;
              visorProxy.send(data, InternetAddress.loopbackIPv4, stationPort);
            }
          }
        });

        try {
          final baseInvite = station.pairingInvite!;
          final visorInvite = VrDesktopBeaconPairingInvite(
            stationId: baseInvite.stationId,
            stationName: baseInvite.stationName,
            host: '127.0.0.1',
            port: visorProxy.port,
            pin: baseInvite.pin,
          );
          final joyInvite = VrDesktopBeaconPairingInvite(
            stationId: baseInvite.stationId,
            stationName: baseInvite.stationName,
            host: '127.0.0.1',
            port: joyProxy.port,
            pin: baseInvite.pin,
            role: 'joystick',
          );

          expect(await visorClient.connectWithInvite(visorInvite), isTrue);
          expect(
            visorRemote.attachControllerLink(
              visorClient.coordinatedControllerLink,
              sessionToken: visorRemote.sessionToken,
            ),
            isTrue,
          );

          var fakeJoyClockUs = 1000000;
          joystickLink = await VrBeaconJoystickControllerLink.connect(
            joyInvite,
            monotonicMicroseconds: () => fakeJoyClockUs,
          );

          final receivedModes = <VrControllerModeRequest>[];
          final joySub = joystickLink.messages.listen((msg) {
            final req = VrControllerModeRequest.tryParse(msg);
            if (req != null) receivedModes.add(req);
          });
          visorRemote.requestControllerMode(RemoteControllerMode.joystick);
          await Future<void>.delayed(const Duration(milliseconds: 60));
          final joyModeRev = receivedModes.last.revision;

          final observedBtnAEdges = <bool>[];
          final stateSub = visorRemote.onState.listen((s) {
            if (observedBtnAEdges.isEmpty || observedBtnAEdges.last != s.btnA) {
              observedBtnAEdges.add(s.btnA);
            }
          });

          // seq 1: neutral
          joystickLink.add(
            VrCompactGamepadFrame(
              sequence: 1,
              senderTimestampMicroseconds: fakeJoyClockUs,
              orientation: vm.Quaternion.identity(),
              angularVelocity: vm.Vector3.zero(),
              hostModeRevision: joyModeRev,
            ),
          );
          await Future<void>.delayed(const Duration(milliseconds: 25));

          // seq 2: DOWN_A t=0, seq 3: UP_A t=+1ms, seq 4: DOWN_A t=+10ms
          fakeJoyClockUs += 16000;
          joystickLink.add(
            VrCompactGamepadFrame(
              sequence: 2,
              senderTimestampMicroseconds: fakeJoyClockUs,
              orientation: vm.Quaternion.identity(),
              angularVelocity: vm.Vector3.zero(),
              btnA: true,
              hostModeRevision: joyModeRev,
            ),
          );
          fakeJoyClockUs += 1000;
          joystickLink.add(
            VrCompactGamepadFrame(
              sequence: 3,
              senderTimestampMicroseconds: fakeJoyClockUs,
              orientation: vm.Quaternion.identity(),
              angularVelocity: vm.Vector3.zero(),
              btnA: false,
              hostModeRevision: joyModeRev,
            ),
          );
          await Future<void>.delayed(const Duration(milliseconds: 10));
          fakeJoyClockUs += 9000;
          joystickLink.add(
            VrCompactGamepadFrame(
              sequence: 4,
              senderTimestampMicroseconds: fakeJoyClockUs,
              orientation: vm.Quaternion.identity(),
              angularVelocity: vm.Vector3.zero(),
              btnA: true,
              hostModeRevision: joyModeRev,
            ),
          );

          // Wait 45 ms so retry of UP3 succeeds, DOWN4 is accepted, AND stale UP3 is injected right after DOWN4!
          await Future<void>.delayed(const Duration(milliseconds: 45));
          expect(injectedLateUp3, isTrue);
          // Stale UP3 MUST NOT overwrite DOWN4 on either Station or Visor!
          expect(visorRemote.latestState.btnA, isTrue);
          expect(station.coordinatorJoystickMirror.latestState.btnA, isTrue);
          if (retainLeg == 'pc_to_visor') {
            expect(
              visorRemote.telemetry.rejectedStates,
              greaterThanOrEqualTo(1),
            );
          }

          // Final UP5 after remaining 50ms (~95ms total)
          await Future<void>.delayed(const Duration(milliseconds: 50));
          fakeJoyClockUs += 95000;
          joystickLink.add(
            VrCompactGamepadFrame(
              sequence: 5,
              senderTimestampMicroseconds: fakeJoyClockUs,
              orientation: vm.Quaternion.identity(),
              angularVelocity: vm.Vector3.zero(),
              btnA: false,
              hostModeRevision: joyModeRev,
            ),
          );
          await Future<void>.delayed(const Duration(milliseconds: 45));
          expect(
            observedBtnAEdges,
            equals(<bool>[false, true, false, true, false]),
            reason: 'Late UP3 must not corrupt two-tap edges on leg=$retainLeg',
          );

          // Now blackhole outbound UDP, enqueue pending discrete edges (seq 6 DOWN, seq 7 UP), and close() mid-retry!
          blackholeJoystickOutbound = true;
          fakeJoyClockUs += 16000;
          joystickLink.add(
            VrCompactGamepadFrame(
              sequence: 6,
              senderTimestampMicroseconds: fakeJoyClockUs,
              orientation: vm.Quaternion.identity(),
              angularVelocity: vm.Vector3.zero(),
              btnA: true,
              hostModeRevision: joyModeRev,
            ),
          );
          fakeJoyClockUs += 2000;
          joystickLink.add(
            VrCompactGamepadFrame(
              sequence: 7,
              senderTimestampMicroseconds: fakeJoyClockUs,
              orientation: vm.Quaternion.identity(),
              angularVelocity: vm.Vector3.zero(),
              btnA: false,
              hostModeRevision: joyModeRev,
            ),
          );
          expect(joystickLink.pendingDiscreteEdgeCount, 2);

          await joystickLink.close();
          linkClosedInTest = true;
          expect(joystickLink.isOpen, isFalse);
          expect(joystickLink.pendingDiscreteEdgeCount, 0);

          await Future<void>.delayed(const Duration(milliseconds: 35));
          expect(joyPacketsAfterClose, 0);

          await stateSub.cancel();
          await joySub.cancel();
        } finally {
          await joystickLink?.close();
          await joyProxySub.cancel();
          await visorProxySub.cancel();
          joyProxy.close();
          visorProxy.close();
          visorRemote.dispose();
          visorClient.dispose();
          station.dispose();
        }
      }
    },
  );

  test(
    'SAIA-FARO-009: Legacy v2 Visor without compactSeqAckKind negotiates legacyBestEffort without fake ACKs, stalling, or relaxed anti-replay',
    () async {
      final station = VrDesktopBeaconStation(
        discoveryPort: 0,
        enableMediaVault: false,
      );
      final legacyVisorClient = VrDesktopBeaconClient(
        discoveryPort: 0,
        advertiseCompactAckCapability: false,
      );
      final visorRemote = VrRemoteControllerService();
      VrBeaconJoystickControllerLink? joystickLink;

      await station.start(enableWebcam: false);
      final stationPort = station.boundPort!;
      final joyProxy = await RawDatagramSocket.bind(
        InternetAddress.loopbackIPv4,
        0,
      );
      final visorProxy = await RawDatagramSocket.bind(
        InternetAddress.loopbackIPv4,
        0,
      );
      InternetAddress? joyAddr;
      int? joyPort;
      InternetAddress? visorAddr;
      int? visorPort;
      var stationToJoyCompactAcks = 0;
      Uint8List? capturedFrame10;
      var relayedCount = 0;

      final joyProxySub = joyProxy.listen((event) {
        if (event != RawSocketEvent.read) return;
        Datagram? dg;
        while ((dg = joyProxy.receive()) != null) {
          final data = dg!.data;
          if (dg.port == stationPort) {
            if (VrDesktopBeaconWire.hasMagic(data) &&
                data[4] == VrDesktopBeaconWire.compactSeqAckKind) {
              stationToJoyCompactAcks++;
            }
            if (joyAddr != null && joyPort != null) {
              joyProxy.send(data, joyAddr!, joyPort!);
            }
          } else {
            joyAddr = dg.address;
            joyPort = dg.port;
            joyProxy.send(data, InternetAddress.loopbackIPv4, stationPort);
          }
        }
      });

      final visorProxySub = visorProxy.listen((event) {
        if (event != RawSocketEvent.read) return;
        Datagram? dg;
        while ((dg = visorProxy.receive()) != null) {
          final data = dg!.data;
          if (dg.port == stationPort) {
            if (VrDesktopBeaconWire.hasMagic(data) &&
                data[4] == VrDesktopBeaconWire.joystickControllerFrameKind) {
              relayedCount++;
              if (relayedCount == 10 && capturedFrame10 == null) {
                capturedFrame10 = Uint8List.fromList(data);
              }
            }
            if (visorAddr != null && visorPort != null) {
              visorProxy.send(data, visorAddr!, visorPort!);
            }
          } else {
            visorAddr = dg.address;
            visorPort = dg.port;
            visorProxy.send(data, InternetAddress.loopbackIPv4, stationPort);
          }
        }
      });

      addTearDown(() async {
        await joystickLink?.close();
        await joyProxySub.cancel();
        await visorProxySub.cancel();
        joyProxy.close();
        visorProxy.close();
        visorRemote.dispose();
        legacyVisorClient.dispose();
        station.dispose();
      });

      final baseInvite = station.pairingInvite!;
      final visorInvite = VrDesktopBeaconPairingInvite(
        stationId: baseInvite.stationId,
        stationName: baseInvite.stationName,
        host: '127.0.0.1',
        port: visorProxy.port,
        pin: baseInvite.pin,
      );
      final joyInvite = VrDesktopBeaconPairingInvite(
        stationId: baseInvite.stationId,
        stationName: baseInvite.stationName,
        host: '127.0.0.1',
        port: joyProxy.port,
        pin: baseInvite.pin,
        role: 'joystick',
      );

      expect(await legacyVisorClient.connectWithInvite(visorInvite), isTrue);
      expect(station.visorSupportsEndToEndCompactAck, isFalse);
      expect(
        visorRemote.attachControllerLink(
          legacyVisorClient.coordinatedControllerLink,
          sessionToken: visorRemote.sessionToken,
        ),
        isTrue,
      );

      var fakeJoyClockUs = 1000000;
      joystickLink = await VrBeaconJoystickControllerLink.connect(
        joyInvite,
        monotonicMicroseconds: () => fakeJoyClockUs,
      );
      expect(joystickLink.isEndToEndVisorAckActive, isFalse);
      expect(
        joystickLink.discreteDeliveryMode,
        VrFaroDiscreteDeliveryMode.legacyBestEffort,
      );

      final receivedModes = <VrControllerModeRequest>[];
      final joySub = joystickLink.messages.listen((msg) {
        final req = VrControllerModeRequest.tryParse(msg);
        if (req != null) receivedModes.add(req);
      });
      addTearDown(joySub.cancel);

      visorRemote.requestControllerMode(RemoteControllerMode.joystick);
      await Future<void>.delayed(const Duration(milliseconds: 60));
      final joyModeRev = receivedModes.last.revision;

      final observedBtnAEdges = <bool>[];
      final observedStickX = <double>[];
      final stateSub = visorRemote.onState.listen((s) {
        if (observedBtnAEdges.isEmpty || observedBtnAEdges.last != s.btnA) {
          observedBtnAEdges.add(s.btnA);
        }
        if (observedStickX.isEmpty ||
            (observedStickX.last - s.stickX).abs() >= 0.015) {
          observedStickX.add(s.stickX);
        }
      });
      addTearDown(stateSub.cancel);

      joystickLink.add(
        VrCompactGamepadFrame(
          sequence: 1,
          senderTimestampMicroseconds: fakeJoyClockUs,
          orientation: vm.Quaternion.identity(),
          angularVelocity: vm.Vector3.zero(),
          hostModeRevision: joyModeRev,
        ),
      );
      await Future<void>.delayed(const Duration(milliseconds: 20));

      for (var tick = 0; tick < 25; tick++) {
        fakeJoyClockUs += 8000;
        final seq = 2 + tick;
        final btnAPressed =
            (tick >= 0 && tick < 7) || (tick >= 15 && tick < 23);
        final stickX = ((tick + 1) / 25.0).clamp(0.0, 1.0);
        joystickLink.add(
          VrCompactGamepadFrame(
            sequence: seq,
            senderTimestampMicroseconds: fakeJoyClockUs,
            orientation: vm.Quaternion.identity(),
            angularVelocity: vm.Vector3.zero(),
            btnA: btnAPressed,
            stickX: stickX,
            hostModeRevision: joyModeRev,
          ),
        );
        await Future<void>.delayed(const Duration(milliseconds: 8));
      }
      await Future<void>.delayed(const Duration(milliseconds: 35));

      final rejectedBefore = visorRemote.telemetry.rejectedStates;
      visorProxy.send(capturedFrame10!, visorAddr!, visorPort!);
      await Future<void>.delayed(const Duration(milliseconds: 15));

      expect(stationToJoyCompactAcks, 0);
      expect(joystickLink.pendingDiscreteEdgeCount, 0);
      expect(joystickLink.unacknowledgedEdgesExpired, 0);
      expect(
        observedBtnAEdges,
        equals(<bool>[false, true, false, true, false]),
      );
      expect(observedStickX.length, greaterThanOrEqualTo(20));
      expect(visorRemote.latestState.stickX, closeTo(1.0, 0.02));
      expect(visorRemote.telemetry.rejectedStates, greaterThan(rejectedBefore));
    },
  );

  testWidgets(
    'SAIA-FARO-010/010b & POINTER-022b: Idle Faro UI and /api/pose.json suppress unobserved seeds, fixed <3mm, and unmeasured 60 FPS; show on valid observation and hide on expiry',
    (tester) async {
      await tester.binding.setSurfaceSize(const Size(1280, 960));
      addTearDown(() => tester.binding.setSurfaceSize(null));

      final fakeCam = _FakeUptimeWebcamPlatform();
      var clockUs = 100000;
      final station = VrDesktopBeaconStation(
        visionPlatform: fakeCam,
        discoveryPort: 0,
        enableMediaVault: true,
        monotonicMicroseconds: () => clockUs,
      );
      final visorClient = VrDesktopBeaconClient(discoveryPort: 0);
      station.setSyntheticAssistEnabled(false);
      station.setMonitorFiducialEnabled(false);

      await tester.runAsync(() async {
        await station.start(enableWebcam: false);
        final base = station.pairingInvite!;
        await visorClient.connectWithInvite(
          VrDesktopBeaconPairingInvite(
            stationId: base.stationId,
            stationName: base.stationName,
            host: '127.0.0.1',
            port: base.port,
            pin: base.pin,
          ),
        );
        await Future<void>.delayed(const Duration(milliseconds: 60));
      });

      await tester.pumpWidget(
        MaterialApp(home: VrDesktopBeaconPage(station: station)),
      );
      await tester.pump();

      try {
        expect(station.latestSolution?.confidence ?? 0.0, 0.0);
        expect(find.textContaining('Sin observación'), findsWidgets);
        expect(find.text('—'), findsWidgets);
        expect(find.textContaining('<3mm'), findsNothing);
        expect(find.text('1.56 m'), findsNothing);
        expect(find.text('1.50 m'), findsNothing);
        expect(find.textContaining('60 FPS'), findsNothing);
        expect(find.textContaining('FPS sin medir'), findsOneWidget);

        late Map<String, dynamic> idlePoseMap;
        await tester.runAsync(() async {
          final poseUri = station.mediaVault.lanBaseUri!.resolve(
            'api/pose.json',
          );
          final client = HttpClient();
          try {
            final req = await client.getUrl(poseUri);
            final resp = await req.close();
            idlePoseMap =
                jsonDecode(await utf8.decodeStream(resp))
                    as Map<String, dynamic>;
          } finally {
            client.close(force: true);
          }
        });
        expect(idlePoseMap['spatialObserved'], isFalse);
        expect(idlePoseMap['head'], isNull);
        expect(idlePoseMap['deskClearanceM'], isNull);
        expect(
          (idlePoseMap['threeDeviceSet'] as Map)['visorFpsMeasured'],
          isFalse,
        );
        expect((idlePoseMap['threeDeviceSet'] as Map)['visorFps'], isNull);

        await tester.runAsync(() async {
          await station.setWebcamEnabled(true);
          clockUs += 32000;
          fakeCam.pushFrame(
            VrDesktopVisionFrame(
              timestampMicros: clockUs,
              mirrored: true,
              inferenceMs: 8.5,
              nose: const VrDesktopKeypoint2D(0.52, 0.34, 0.95),
              leftShoulder: const VrDesktopKeypoint2D(0.38, 0.54, 0.94),
              rightShoulder: const VrDesktopKeypoint2D(0.66, 0.54, 0.94),
              rightWrist: const VrDesktopKeypoint2D(0.70, 0.28, 0.92),
              leftWrist: const VrDesktopKeypoint2D(0.34, 0.28, 0.91),
            ),
          );
          await Future<void>.delayed(const Duration(milliseconds: 50));
        });
        await tester.pump();
        expect(station.latestSolution?.hasValidSpatialObservation, isTrue);
        expect(find.text('Sin observación'), findsNothing);

        await tester.runAsync(() async {
          clockUs += 1500000;
          await Future<void>.delayed(const Duration(milliseconds: 50));
        });
        await tester.pump();
        expect(station.latestSolution?.hasValidSpatialObservation, isFalse);
        expect(find.text('Sin observación'), findsOneWidget);
      } finally {
        await tester.pumpWidget(const SizedBox.shrink());
        await tester.runAsync(() async {
          visorClient.dispose();
          station.dispose();
          await fakeCam.dispose();
          await Future<void>.delayed(const Duration(milliseconds: 20));
        });
      }
    },
  );

  test(
    'SAIA-FARO-010b consumer: confidence=0 beacon solutions preserve heartbeat without accumulating pose delta, obstacle clearance, or focus shift',
    () async {
      final service = VrSpatialTrackingService();
      final customServer = await RawDatagramSocket.bind(
        InternetAddress.loopbackIPv4,
        0,
      );
      addTearDown(() async {
        customServer.close();
        service.dispose();
      });

      const stationId = 'faro_zero_conf_unit';
      const pin = '654321';
      final handshakeKey = VrDesktopBeaconWire.deriveHandshakeKey(
        stationId,
        pin,
      );
      List<int>? sessionKey;
      InternetAddress? visorAddr;
      int? visorPort;

      final sub = customServer.listen((event) {
        if (event != RawSocketEvent.read) return;
        Datagram? dg;
        while ((dg = customServer.receive()) != null) {
          final map = VrDesktopBeaconWire.decodeAuthenticatedJson(
            expectedKind: VrDesktopBeaconWire.helloKind,
            key: handshakeKey,
            datagram: dg!.data,
          );
          if (map != null) {
            final cn = map['cn'] as String;
            const sn = 'serverNonce_010b_unit';
            sessionKey = VrDesktopBeaconWire.deriveSessionKey(
              stationId: stationId,
              pin: pin,
              clientNonce: cn,
              serverNonce: sn,
            );
            visorAddr = dg.address;
            visorPort = dg.port;
            final ack = VrDesktopBeaconWire.encodeAuthenticatedJson(
              kind: VrDesktopBeaconWire.helloAckKind,
              key: handshakeKey,
              payload: {
                'sn': sn,
                'cn': cn,
                'id': stationId,
                'name': 'ZeroConfFaro',
                'role': 'visor',
                'caps': VrTriNodeDepthSolution.modalityWifiUdp,
              },
            );
            customServer.send(ack, dg.address, dg.port);
          }
        }
      });
      addTearDown(sub.cancel);

      final initialObstacle = service.visorObstacleDistanceMeters;
      expect(
        await service.beaconClient.connect(
          stationId: stationId,
          host: '127.0.0.1',
          port: customServer.port,
          pin: pin,
        ),
        isTrue,
      );

      for (final (seq, pos) in <(int, vm.Vector3)>[
        (1, vm.Vector3(0.0, 1.60, 0.0)),
        (2, vm.Vector3(0.20, 1.70, -0.18)),
      ]) {
        final sol = VrTriNodeDepthSolution(
          timestampMicros: seq * 16000,
          sequence: seq,
          computerToVisorMeters: 2.05,
          computerToJoystickMeters: 1.10,
          visorToJoystickMeters: 0.50,
          visorRoomPosition: pos,
          joystickRoomPosition: vm.Vector3(0.0, 1.18, -0.38),
          deskClearanceMeters: 0.40,
          visorAzimuthRadians: 0.0,
          yawAnchorCorrectionRadians: 0.0,
          leftHandRelHead: vm.Vector3(-0.18, -0.22, -0.56),
          rightHandRelHead: vm.Vector3(0.18, -0.22, -0.56),
          leftHandConfidence: 0.0,
          rightHandConfidence: 0.0,
          leftHandRaised: false,
          rightHandRaised: false,
          leftPinchStrength: 0.0,
          rightPinchStrength: 0.0,
          gesture: VrBeaconHandGesture.none,
          confidence: 0.0,
        );
        customServer.send(
          VrDesktopBeaconWire.encodeBeaconState(
            sessionKey: sessionKey!,
            solution: sol,
          ),
          visorAddr!,
          visorPort!,
        );
        await Future<void>.delayed(const Duration(milliseconds: 20));
      }

      final delta = service.syncRigPoseAndSampleLocomotion(
        rigEyeCenter: vm.Vector3(0.0, 1.60, 0.0),
        rigYaw: 0.0,
        rigPitch: 0.0,
        dtSeconds: 0.016,
        nativeVioTracking: false,
      );
      expect(service.beaconConnected, isTrue);
      expect(service.rfBeaconSamples, greaterThanOrEqualTo(2));
      expect(delta.length, closeTo(0.0, 1e-6));
      expect(
        service.visorObstacleDistanceMeters,
        closeTo(initialObstacle, 1e-6),
      );
      expect(service.recommendedFocusDistanceMeters, closeTo(1.35, 1e-6));
    },
  );

  test(
    'SAIA-FARO-011: pointerPoseReady=false scrubs FIFO discrete backlog and reaches Visor in <150 ms under 100% lost return ACKs without sequence skipping or fake ACKs',
    () async {
      final station = VrDesktopBeaconStation(
        discoveryPort: 0,
        enableMediaVault: false,
      );
      final visorClient = VrDesktopBeaconClient(discoveryPort: 0);
      final visorRemote = VrRemoteControllerService();
      VrBeaconJoystickControllerLink? joystickLink;

      await station.start(enableWebcam: false);
      final stationPort = station.boundPort!;
      final joyProxy = await RawDatagramSocket.bind(
        InternetAddress.loopbackIPv4,
        0,
      );
      final visorProxy = await RawDatagramSocket.bind(
        InternetAddress.loopbackIPv4,
        0,
      );
      InternetAddress? joyAddr;
      int? joyPort;
      InternetAddress? visorAddr;
      int? visorPort;
      var dropAllCompactAcks = false;
      var stationSynthesizedAcks = 0;
      final acceptedVisorSeqs = <int>[];

      final joyProxySub = joyProxy.listen((event) {
        if (event != RawSocketEvent.read) return;
        Datagram? dg;
        while ((dg = joyProxy.receive()) != null) {
          final data = dg!.data;
          if (dg.port == stationPort) {
            if (VrDesktopBeaconWire.hasMagic(data) &&
                data[4] == VrDesktopBeaconWire.compactSeqAckKind &&
                dropAllCompactAcks) {
              stationSynthesizedAcks++;
            }
            if (joyAddr != null && joyPort != null) {
              joyProxy.send(data, joyAddr!, joyPort!);
            }
          } else {
            joyAddr = dg.address;
            joyPort = dg.port;
            joyProxy.send(data, InternetAddress.loopbackIPv4, stationPort);
          }
        }
      });

      final visorProxySub = visorProxy.listen((event) {
        if (event != RawSocketEvent.read) return;
        Datagram? dg;
        while ((dg = visorProxy.receive()) != null) {
          final data = dg!.data;
          if (dg.port == stationPort) {
            if (visorAddr != null && visorPort != null) {
              visorProxy.send(data, visorAddr!, visorPort!);
            }
          } else {
            visorAddr = dg.address;
            visorPort = dg.port;
            if (dropAllCompactAcks &&
                VrDesktopBeaconWire.hasMagic(data) &&
                data[4] == VrDesktopBeaconWire.compactSeqAckKind) {
              continue; // Drop 100% of Visor -> PC compact sequence ACKs!
            }
            visorProxy.send(data, InternetAddress.loopbackIPv4, stationPort);
          }
        }
      });

      addTearDown(() async {
        await joystickLink?.close();
        await joyProxySub.cancel();
        await visorProxySub.cancel();
        joyProxy.close();
        visorProxy.close();
        visorRemote.dispose();
        visorClient.dispose();
        station.dispose();
      });

      final baseInvite = station.pairingInvite!;
      final visorInvite = VrDesktopBeaconPairingInvite(
        stationId: baseInvite.stationId,
        stationName: baseInvite.stationName,
        host: '127.0.0.1',
        port: visorProxy.port,
        pin: baseInvite.pin,
      );
      final joyInvite = VrDesktopBeaconPairingInvite(
        stationId: baseInvite.stationId,
        stationName: baseInvite.stationName,
        host: '127.0.0.1',
        port: joyProxy.port,
        pin: baseInvite.pin,
        role: 'joystick',
      );

      expect(await visorClient.connectWithInvite(visorInvite), isTrue);
      final wrappedVisorLink = VrBeaconVisorControllerLink(
        onSendControl: visorClient.sendVisorControllerControl,
        onAcknowledgeCompactSequence: (seq) {
          acceptedVisorSeqs.add(seq);
          visorClient.sendVisorCompactSequenceAck(seq);
        },
      );
      visorClient.coordinatedControllerLink.messages.listen((m) {
        if (m != null) wrappedVisorLink.deliverFromStation(m);
      });
      expect(
        visorRemote.attachControllerLink(
          wrappedVisorLink,
          sessionToken: visorRemote.sessionToken,
        ),
        isTrue,
      );

      var fakeJoyClockUs = 1000000;
      joystickLink = await VrBeaconJoystickControllerLink.connect(
        joyInvite,
        monotonicMicroseconds: () => fakeJoyClockUs,
      );
      expect(joystickLink.isEndToEndVisorAckActive, isTrue);

      final receivedModes = <VrControllerModeRequest>[];
      final joySub = joystickLink.messages.listen((msg) {
        final req = VrControllerModeRequest.tryParse(msg);
        if (req != null) receivedModes.add(req);
      });
      addTearDown(joySub.cancel);

      visorRemote.requestControllerMode(
        RemoteControllerMode.joystick,
        horizontalLook: true,
      );
      await Future<void>.delayed(const Duration(milliseconds: 60));
      final joyModeRev = receivedModes.last.revision;

      // Baseline seq 1: pointerPoseReady = true
      joystickLink.add(
        VrCompactGamepadFrame(
          sequence: 1,
          senderTimestampMicroseconds: fakeJoyClockUs,
          orientation: vm.Quaternion.identity(),
          angularVelocity: vm.Vector3.zero(),
          modeIndex: 0,
          pointerPoseReady: true,
          hostModeRevision: joyModeRev,
        ),
      );
      await Future<void>.delayed(const Duration(milliseconds: 25));
      expect(visorRemote.latestState.pointerPoseReady, isTrue);

      // Blackhole 100% of return ACKs and enqueue 12 discrete button edges
      // (seq 2..13) all captured while pointerPoseReady == true!
      dropAllCompactAcks = true;
      for (var i = 0; i < 12; i++) {
        fakeJoyClockUs += 2000;
        joystickLink.add(
          VrCompactGamepadFrame(
            sequence: 2 + i,
            senderTimestampMicroseconds: fakeJoyClockUs,
            orientation: vm.Quaternion.identity(),
            angularVelocity: vm.Vector3.zero(),
            modeIndex: 0,
            btnA: i.isEven,
            pointerPoseReady: true,
            hostModeRevision: joyModeRev,
          ),
        );
      }
      await Future<void>.delayed(const Duration(milliseconds: 12));
      expect(visorRemote.latestState.pointerPoseReady, isTrue);
      expect(joystickLink.pendingDiscreteEdgeCount, 12);

      // Now user lowers phone -> pointerPoseReady: false (seq 14, continuous)!
      final sw = Stopwatch()..start();
      fakeJoyClockUs += 8000;
      joystickLink.add(
        VrCompactGamepadFrame(
          sequence: 14,
          senderTimestampMicroseconds: fakeJoyClockUs,
          orientation: vm.Quaternion.identity(),
          angularVelocity: vm.Vector3.zero(),
          modeIndex: 0,
          btnA: false,
          pointerPoseReady: false,
          hostModeRevision: joyModeRev,
        ),
      );

      int? poseClearedElapsedMs;
      while (sw.elapsedMilliseconds < 220) {
        if (!visorRemote.latestState.pointerPoseReady) {
          poseClearedElapsedMs = sw.elapsedMilliseconds;
          break;
        }
        await Future<void>.delayed(const Duration(milliseconds: 4));
      }

      expect(
        poseClearedElapsedMs,
        isNotNull,
        reason:
            'Visor must receive pointerPoseReady=false without hanging on lost ACKs',
      );
      expect(
        poseClearedElapsedMs!,
        lessThan(150),
        reason:
            'pointerPoseReady=false must reach Visor within <150 ms freshness budget',
      );
      expect(stationSynthesizedAcks, 0);
      // Verify zero sequence skipping: seq 3 was accepted right after seq 2!
      expect(acceptedVisorSeqs, containsAllInOrder(<int>[1, 2, 3]));

      // Re-enable return ACKs and verify E2E ACK mode self-heals automatically!
      await Future<void>.delayed(const Duration(milliseconds: 90));
      dropAllCompactAcks = false;
      fakeJoyClockUs += 100000;
      joystickLink.add(
        VrCompactGamepadFrame(
          sequence: 15,
          senderTimestampMicroseconds: fakeJoyClockUs,
          orientation: vm.Quaternion.identity(),
          angularVelocity: vm.Vector3.zero(),
          modeIndex: 0,
          btnA: true,
          pointerPoseReady: false,
          hostModeRevision: joyModeRev,
        ),
      );
      await Future<void>.delayed(const Duration(milliseconds: 30));
      expect(joystickLink.isEndToEndVisorAckActive, isTrue);
    },
  );

  test(
    'SAIA-POINTER-022b: VrDesktopBeaconClient re-entering connectWithInvite during active timeout never clobbers live session and auto-recovers after station restart',
    () async {
      var clientClockUs = 1000000;
      final deadSocket = await RawDatagramSocket.bind(
        InternetAddress.loopbackIPv4,
        0,
      );
      final deadPort = deadSocket.port;
      deadSocket.close();

      final station = VrDesktopBeaconStation(
        discoveryPort: 0,
        enableMediaVault: false,
      );
      await station.start(enableWebcam: false);
      final visorClient = VrDesktopBeaconClient(
        discoveryPort: 0,
        monotonicMicroseconds: () => clientClockUs,
      );

      addTearDown(() {
        visorClient.dispose();
        station.dispose();
      });

      final baseInvite = station.pairingInvite!;
      final badInvite = VrDesktopBeaconPairingInvite(
        stationId: baseInvite.stationId,
        stationName: baseInvite.stationName,
        host: '127.0.0.1',
        port: deadPort,
        pin: baseInvite.pin,
      );
      final goodInvite = VrDesktopBeaconPairingInvite(
        stationId: baseInvite.stationId,
        stationName: baseInvite.stationName,
        host: '127.0.0.1',
        port: baseInvite.port,
        pin: baseInvite.pin,
      );

      // Start a failing connect attempt with a 250 ms timeout, then immediately
      // re-enter connectWithInvite with the live invite while the first is pending!
      final firstAttempt = visorClient.connect(
        stationId: badInvite.stationId,
        host: badInvite.host,
        port: badInvite.port,
        pin: badInvite.pin,
        timeout: const Duration(milliseconds: 250),
      );
      await Future<void>.delayed(const Duration(milliseconds: 20));
      final secondOk = await visorClient.connectWithInvite(goodInvite);
      expect(secondOk, isTrue);
      expect(visorClient.isConnected, isTrue);

      // Let the first attempt's 250 ms timeout fire -> must NOT disconnect the live session!
      expect(await firstAttempt, isFalse);
      await Future<void>.delayed(const Duration(milliseconds: 60));
      expect(visorClient.isConnected, isTrue);
      expect(visorClient.lastError, isNull);

      // Advance client clock past 3.5 s packet timeout -> auto-reconnect watchdog
      // (2.0 s interval) automatically re-authenticates without manual restart!
      clientClockUs += 5000000;
      expect(visorClient.isConnected, isFalse);
      await Future<void>.delayed(const Duration(milliseconds: 2450));
      expect(visorClient.isConnected, isTrue);
    },
  );

  test(
    'SAIA-REVIEW-ARGON-019a P2: delayed bridgeDirective(epoch N, e2eAck=true) and delayed compactSeqAck replayed after bridgeDirective(epoch N+1, e2eAck=false) never revive E2E ACK mode on legacy-v2 Visor',
    () async {
      final station = VrDesktopBeaconStation(
        discoveryPort: 0,
        enableMediaVault: false,
      );
      final visor1Client = VrDesktopBeaconClient(discoveryPort: 0);
      final legacyVisor2Client = VrDesktopBeaconClient(
        discoveryPort: 0,
        advertiseCompactAckCapability: false,
      );
      final visor1Remote = VrRemoteControllerService();
      final visor2Remote = VrRemoteControllerService();
      VrBeaconJoystickControllerLink? joystickLink;

      await station.start(enableWebcam: false);
      final stationPort = station.boundPort!;
      final joyProxy = await RawDatagramSocket.bind(
        InternetAddress.loopbackIPv4,
        0,
      );
      InternetAddress? joyAddr;
      int? joyPort;

      var retainNextTrueDirective = false;
      var retainNextCompactAck = false;
      Uint8List? retainedTrueDirectiveDatagram;
      Uint8List? retainedCompactAckDatagram;

      final joyProxySub = joyProxy.listen((event) {
        if (event != RawSocketEvent.read) return;
        Datagram? dg;
        while ((dg = joyProxy.receive()) != null) {
          final data = Uint8List.fromList(dg!.data);
          if (dg.port == stationPort) {
            if (VrDesktopBeaconWire.hasMagic(data)) {
              final kind = data[4];
              if (retainNextTrueDirective &&
                  kind == VrDesktopBeaconWire.bridgeDirectiveKind &&
                  retainedTrueDirectiveDatagram == null) {
                retainedTrueDirectiveDatagram = data;
                retainNextTrueDirective = false;
                continue;
              }
              if (retainNextCompactAck &&
                  kind == VrDesktopBeaconWire.compactSeqAckKind &&
                  retainedCompactAckDatagram == null) {
                retainedCompactAckDatagram = data;
                retainNextCompactAck = false;
                continue;
              }
            }
            if (joyAddr != null && joyPort != null) {
              joyProxy.send(data, joyAddr!, joyPort!);
            }
          } else {
            joyAddr = dg.address;
            joyPort = dg.port;
            joyProxy.send(data, InternetAddress.loopbackIPv4, stationPort);
          }
        }
      });

      addTearDown(() async {
        await joystickLink?.close();
        await joyProxySub.cancel();
        joyProxy.close();
        visor1Remote.dispose();
        visor2Remote.dispose();
        visor1Client.dispose();
        legacyVisor2Client.dispose();
        station.dispose();
      });

      final baseInvite = station.pairingInvite!;
      final visorInvite = VrDesktopBeaconPairingInvite(
        stationId: baseInvite.stationId,
        stationName: baseInvite.stationName,
        host: '127.0.0.1',
        port: stationPort,
        pin: baseInvite.pin,
      );
      final joyInvite = VrDesktopBeaconPairingInvite(
        stationId: baseInvite.stationId,
        stationName: baseInvite.stationName,
        host: '127.0.0.1',
        port: joyProxy.port,
        pin: baseInvite.pin,
        role: 'joystick',
      );

      expect(await visor1Client.connectWithInvite(visorInvite), isTrue);
      expect(
        visor1Remote.attachControllerLink(
          visor1Client.coordinatedControllerLink,
          sessionToken: visor1Remote.sessionToken,
        ),
        isTrue,
      );

      var fakeJoyClockUs = 1000000;
      joystickLink = await VrBeaconJoystickControllerLink.connect(
        joyInvite,
        monotonicMicroseconds: () => fakeJoyClockUs,
      );
      expect(joystickLink.isEndToEndVisorAckActive, isTrue);

      final receivedModes = <VrControllerModeRequest>[];
      final joySub = joystickLink.messages.listen((msg) {
        final req = VrControllerModeRequest.tryParse(msg);
        if (req != null) receivedModes.add(req);
      });
      addTearDown(joySub.cancel);

      visor1Remote.requestControllerMode(
        RemoteControllerMode.joystick,
        horizontalLook: true,
      );
      await Future<void>.delayed(const Duration(milliseconds: 50));
      var joyModeRev = receivedModes.last.revision;

      joystickLink.add(
        VrCompactGamepadFrame(
          sequence: 1,
          senderTimestampMicroseconds: fakeJoyClockUs,
          orientation: vm.Quaternion.identity(),
          angularVelocity: vm.Vector3.zero(),
          hostModeRevision: joyModeRev,
        ),
      );
      await Future<void>.delayed(const Duration(milliseconds: 25));

      retainNextTrueDirective = true;
      retainNextCompactAck = true;
      station.setCastToDesktopEnabled(!station.castToDesktopEnabled);
      fakeJoyClockUs += 10000;
      joystickLink.add(
        VrCompactGamepadFrame(
          sequence: 2,
          senderTimestampMicroseconds: fakeJoyClockUs,
          orientation: vm.Quaternion.identity(),
          angularVelocity: vm.Vector3.zero(),
          btnA: true,
          hostModeRevision: joyModeRev,
        ),
      );
      await Future<void>.delayed(const Duration(milliseconds: 45));
      expect(retainedTrueDirectiveDatagram, isNotNull);
      expect(retainedCompactAckDatagram, isNotNull);

      // Swap to legacy-v2 Visor without 'cack' -> epoch N+1 sets e2eAck=false!
      visor1Client.disconnect();
      expect(await legacyVisor2Client.connectWithInvite(visorInvite), isTrue);
      expect(
        visor2Remote.attachControllerLink(
          legacyVisor2Client.coordinatedControllerLink,
          sessionToken: visor2Remote.sessionToken,
        ),
        isTrue,
      );
      visor2Remote.requestControllerMode(
        RemoteControllerMode.joystick,
        horizontalLook: true,
      );
      await Future<void>.delayed(const Duration(milliseconds: 60));
      joyModeRev = receivedModes.last.revision;
      expect(joystickLink.isEndToEndVisorAckActive, isFalse);

      // Replay delayed compactSeqAck(seq 2) and delayed bridgeDirective(epoch N, true)
      joyProxy.send(retainedCompactAckDatagram!, joyAddr!, joyPort!);
      await Future<void>.delayed(const Duration(milliseconds: 5));
      expect(joystickLink.isEndToEndVisorAckActive, isFalse);

      station.setCastToDesktopEnabled(!station.castToDesktopEnabled);
      await Future<void>.delayed(const Duration(milliseconds: 30));
      expect(joystickLink.isEndToEndVisorAckActive, isFalse);

      joyProxy.send(retainedTrueDirectiveDatagram!, joyAddr!, joyPort!);
      await Future<void>.delayed(const Duration(milliseconds: 5));
      expect(joystickLink.isEndToEndVisorAckActive, isFalse);
    },
  );

  test(
    'SAIA-REVIEW-ARGON-019a TTL: advancing injected clock >= 150000 us with retained pointerPoseReady=true and zero false frames scrubs stale posture on drain',
    () async {
      final station = VrDesktopBeaconStation(
        discoveryPort: 0,
        enableMediaVault: false,
      );
      final visorClient = VrDesktopBeaconClient(discoveryPort: 0);
      final visorRemote = VrRemoteControllerService();
      VrBeaconJoystickControllerLink? joystickLink;

      await station.start(enableWebcam: false);
      final stationPort = station.boundPort!;
      final visorProxy = await RawDatagramSocket.bind(
        InternetAddress.loopbackIPv4,
        0,
      );
      InternetAddress? visorAddr;
      int? visorPort;
      var holdCompactAcks = false;
      final heldAckDatagrams = <Uint8List>[];

      final visorProxySub = visorProxy.listen((event) {
        if (event != RawSocketEvent.read) return;
        Datagram? dg;
        while ((dg = visorProxy.receive()) != null) {
          final data = Uint8List.fromList(dg!.data);
          if (dg.port == stationPort) {
            if (visorAddr != null && visorPort != null) {
              visorProxy.send(data, visorAddr!, visorPort!);
            }
          } else {
            visorAddr = dg.address;
            visorPort = dg.port;
            if (holdCompactAcks &&
                VrDesktopBeaconWire.hasMagic(data) &&
                data[4] == VrDesktopBeaconWire.compactSeqAckKind) {
              heldAckDatagrams.add(data);
              continue;
            }
            visorProxy.send(data, InternetAddress.loopbackIPv4, stationPort);
          }
        }
      });

      addTearDown(() async {
        await joystickLink?.close();
        await visorProxySub.cancel();
        visorProxy.close();
        visorRemote.dispose();
        visorClient.dispose();
        station.dispose();
      });

      final baseInvite = station.pairingInvite!;
      final visorInvite = VrDesktopBeaconPairingInvite(
        stationId: baseInvite.stationId,
        stationName: baseInvite.stationName,
        host: '127.0.0.1',
        port: visorProxy.port,
        pin: baseInvite.pin,
      );
      final joyInvite = VrDesktopBeaconPairingInvite(
        stationId: baseInvite.stationId,
        stationName: baseInvite.stationName,
        host: '127.0.0.1',
        port: stationPort,
        pin: baseInvite.pin,
        role: 'joystick',
      );

      expect(await visorClient.connectWithInvite(visorInvite), isTrue);
      expect(
        visorRemote.attachControllerLink(
          visorClient.coordinatedControllerLink,
          sessionToken: visorRemote.sessionToken,
        ),
        isTrue,
      );

      var fakeJoyClockUs = 1000000;
      joystickLink = await VrBeaconJoystickControllerLink.connect(
        joyInvite,
        monotonicMicroseconds: () => fakeJoyClockUs,
      );

      final receivedModes = <VrControllerModeRequest>[];
      final joySub = joystickLink.messages.listen((msg) {
        final req = VrControllerModeRequest.tryParse(msg);
        if (req != null) receivedModes.add(req);
      });
      addTearDown(joySub.cancel);

      visorRemote.requestControllerMode(
        RemoteControllerMode.joystick,
        horizontalLook: true,
      );
      await Future<void>.delayed(const Duration(milliseconds: 50));
      final joyModeRev = receivedModes.last.revision;

      joystickLink.add(
        VrCompactGamepadFrame(
          sequence: 1,
          senderTimestampMicroseconds: fakeJoyClockUs,
          orientation: vm.Quaternion.identity(),
          angularVelocity: vm.Vector3.zero(),
          pointerPoseReady: true,
          hostModeRevision: joyModeRev,
        ),
      );
      await Future<void>.delayed(const Duration(milliseconds: 25));
      expect(visorRemote.latestState.pointerPoseReady, isTrue);

      holdCompactAcks = true;
      fakeJoyClockUs += 25000;
      joystickLink.add(
        VrCompactGamepadFrame(
          sequence: 2,
          senderTimestampMicroseconds: fakeJoyClockUs,
          orientation: vm.Quaternion.identity(),
          angularVelocity: vm.Vector3.zero(),
          btnA: true,
          pointerPoseReady: true,
          hostModeRevision: joyModeRev,
        ),
      );
      fakeJoyClockUs += 25000;
      joystickLink.add(
        VrCompactGamepadFrame(
          sequence: 3,
          senderTimestampMicroseconds: fakeJoyClockUs,
          orientation: vm.Quaternion.identity(),
          angularVelocity: vm.Vector3.zero(),
          btnA: false,
          pointerPoseReady: true,
          hostModeRevision: joyModeRev,
        ),
      );
      fakeJoyClockUs += 4000;
      joystickLink.add(
        VrCompactGamepadFrame(
          sequence: 4,
          senderTimestampMicroseconds: fakeJoyClockUs,
          orientation: vm.Quaternion.identity(),
          angularVelocity: vm.Vector3.zero(),
          stickX: 0.65,
          btnA: false,
          pointerPoseReady: true,
          hostModeRevision: joyModeRev,
        ),
      );
      await Future<void>.delayed(const Duration(milliseconds: 15));
      expect(joystickLink.pendingDiscreteEdgeCount, 2);

      // Advance clock past 150 ms TTL without sending any false frame!
      fakeJoyClockUs += 160000;
      holdCompactAcks = false;
      for (final ack in heldAckDatagrams) {
        visorProxy.send(ack, InternetAddress.loopbackIPv4, stationPort);
      }
      await Future<void>.delayed(const Duration(milliseconds: 50));

      expect(joystickLink.pendingDiscreteEdgeCount, 0);
      expect(visorRemote.latestState.btnA, isFalse);
      expect(visorRemote.latestState.stickX, closeTo(0.65, 0.02));
      expect(visorRemote.latestState.pointerPoseReady, isFalse);
    },
  );

  test(
    'VrTriNodeDepthSolver gates duplicate 62.5 Hz webcam frames without false spike rejection and VrSpatialTrackingService rotates Faro 6-DoF parallax by rigBodyYaw (C-FARO-02)',
    () async {
      final solver = VrTriNodeDepthSolver();
      const frame1 = VrDesktopVisionFrame(
        timestampMicros: 100000,
        mirrored: true,
        inferenceMs: 9.0,
        nose: VrDesktopKeypoint2D(0.50, 0.30, 0.95),
        leftShoulder: VrDesktopKeypoint2D(0.62, 0.45, 0.92),
        rightShoulder: VrDesktopKeypoint2D(0.38, 0.45, 0.92),
        rightWrist: VrDesktopKeypoint2D(0.40, 0.52, 0.94),
      );
      // Frame 1 arrives at t=100ms, then two duplicate 62.5 Hz station ticks at 116ms and 132ms:
      final sol0 = solver.solve(nowMicros: 100000, vision: frame1);
      final solDup1 = solver.solve(nowMicros: 116000, vision: frame1);
      final solDup2 = solver.solve(nowMicros: 132000, vision: frame1);
      expect(sol0.rightHandConfidence, greaterThan(0.6));
      expect(
        solDup2.rightHandConfidence,
        closeTo(solDup1.rightHandConfidence, 0.02),
      );
      expect(solver.lastHandObservationSpikeHeld, isFalse);

      // Next real 20 fps webcam frame arrives 50ms after frame1 with natural ~10 cm hand motion
      // (~2.0 m/s over 50ms — previously seen as >6.0 m/s over 16ms and spike-clamped!).
      const frame2 = VrDesktopVisionFrame(
        timestampMicros: 150000,
        mirrored: true,
        inferenceMs: 9.0,
        nose: VrDesktopKeypoint2D(0.50, 0.30, 0.95),
        leftShoulder: VrDesktopKeypoint2D(0.62, 0.45, 0.92),
        rightShoulder: VrDesktopKeypoint2D(0.38, 0.45, 0.92),
        rightWrist: VrDesktopKeypoint2D(0.30, 0.50, 0.94),
      );
      final solNext = solver.solve(nowMicros: 150000, vision: frame2);
      expect(solver.lastHandObservationSpikeHeld, isFalse);
      expect(solNext.rightHandConfidence, greaterThan(0.7));
      expect(
        (solNext.rightHandRelHead - sol0.rightHandRelHead).length,
        greaterThan(0.03),
      );

      // Verify rigBodyYaw = pi rotates Faro room-space horizontal delta by 180 degrees end-to-end
      final fakeCam = _FakeUptimeWebcamPlatform();
      final station = VrDesktopBeaconStation(
        visionPlatform: fakeCam,
        discoveryPort: 0,
        enableMediaVault: false,
      );
      final tracking = VrSpatialTrackingService();
      addTearDown(() async {
        tracking.dispose();
        station.dispose();
        await fakeCam.dispose();
      });
      await station.start(enableWebcam: true);
      final baseInvite = station.pairingInvite!;
      final loopbackInvite = VrDesktopBeaconPairingInvite(
        stationId: baseInvite.stationId,
        stationName: baseInvite.stationName,
        host: '127.0.0.1',
        port: baseInvite.port,
        pin: baseInvite.pin,
      );
      expect(
        await tracking.beaconClient.connectWithInvite(loopbackInvite),
        isTrue,
      );

      // In mirrored webcam space (user-right = +u -> +roomPos.x):
      // At bodyYaw=0, +dx maps to -rx; turned 180 deg (bodyYaw=pi), cos(pi)=-1 flips it to +world X!
      fakeCam.pushFrame(
        const VrDesktopVisionFrame(
          timestampMicros: 200000,
          mirrored: true,
          inferenceMs: 8.0,
          nose: VrDesktopKeypoint2D(0.45, 0.30, 0.96),
          leftShoulder: VrDesktopKeypoint2D(0.33, 0.45, 0.94),
          rightShoulder: VrDesktopKeypoint2D(0.57, 0.45, 0.94),
        ),
      );
      await Future<void>.delayed(const Duration(milliseconds: 50));
      // Drain initial baseline sample at bodyYaw = pi (3.141592653589793)
      tracking.syncRigPoseAndSampleLocomotion(
        rigEyeCenter: vm.Vector3(0, 1.6, 0),
        rigYaw: 3.141592653589793,
        rigBodyYaw: 3.141592653589793,
        rigPitch: 0.0,
        dtSeconds: 0.016,
        nativeVioTracking: false,
      );

      fakeCam.pushFrame(
        const VrDesktopVisionFrame(
          timestampMicros: 260000,
          mirrored: true,
          inferenceMs: 8.0,
          nose: VrDesktopKeypoint2D(0.62, 0.30, 0.96),
          leftShoulder: VrDesktopKeypoint2D(0.50, 0.45, 0.94),
          rightShoulder: VrDesktopKeypoint2D(0.74, 0.45, 0.94),
        ),
      );
      await Future<void>.delayed(const Duration(milliseconds: 80));
      final deltaTurned180 = tracking.syncRigPoseAndSampleLocomotion(
        rigEyeCenter: vm.Vector3(0, 1.6, 0),
        rigYaw: 3.141592653589793,
        rigBodyYaw: 3.141592653589793,
        rigPitch: 0.0,
        dtSeconds: 0.016,
        nativeVioTracking: false,
      );
      expect(deltaTurned180.x, greaterThan(0.05));
    },
  );
}
