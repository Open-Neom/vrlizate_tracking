import 'dart:async';
import 'dart:collection';
import 'dart:convert';
import 'dart:io';
import 'dart:math' as math;

import 'package:flutter/foundation.dart';
import 'package:vrlizate/vrlizate.dart' show VrCompactGamepadFrame;
import 'package:vrlizate_joystick/vrlizate_joystick.dart';

import 'vr_desktop_autonomous_bridge.dart';
import 'vr_desktop_beacon_wire.dart';

/// In-memory duplex pipe used by [VrDesktopBeaconStation] to mirror live
/// Joystick snapshots and supply neutral mode ACKs before a Visor joins.
class VrInMemoryCoordinatorControllerPipe {
  VrInMemoryCoordinatorControllerPipe() {
    _stationSide = _PipeEndpoint(
      incoming: _toStation.stream,
      onSend: (data) {
        if (!_closed && !_toJoystick.isClosed) {
          _toJoystick.add(data);
        }
      },
      onClose: close,
      isOpenGetter: () => !_closed,
    );
    _joystickSide = _PipeEndpoint(
      incoming: _toJoystick.stream,
      onSend: (data) {
        if (!_closed && !_toStation.isClosed) {
          _toStation.add(data);
        }
      },
      onClose: close,
      isOpenGetter: () => !_closed,
    );
  }

  final StreamController<Object?> _toStation =
      StreamController<Object?>.broadcast();
  final StreamController<Object?> _toJoystick =
      StreamController<Object?>.broadcast();
  bool _closed = false;
  late final VrControllerLink _stationSide;
  late final VrControllerLink _joystickSide;

  VrControllerLink get stationSide => _stationSide;
  VrControllerLink get joystickSide => _joystickSide;

  Future<void> close() async {
    if (_closed) return;
    _closed = true;
    await _toStation.close();
    await _toJoystick.close();
  }
}

class _PipeEndpoint implements VrControllerLink, VrCompactGamepadCapableLink {
  _PipeEndpoint({
    required this.incoming,
    required this.onSend,
    required this.onClose,
    required this.isOpenGetter,
  });

  final Stream<Object?> incoming;
  final void Function(Object data) onSend;
  final Future<void> Function() onClose;
  final bool Function() isOpenGetter;

  @override
  bool get supportsCompactGamepadFrames => true;

  @override
  Stream<Object?> get messages => incoming;

  @override
  bool get isOpen => isOpenGetter();

  @override
  void add(Object data) => onSend(data);

  @override
  Future<void> close([int? code, String? reason]) => onClose();
}

class _PendingDiscreteEdge {
  _PendingDiscreteEdge({
    required this.frame,
    required this.holdActive,
    required this.enqueuedAtUs,
  });

  VrCompactGamepadFrame frame;
  bool holdActive;
  final int enqueuedAtUs;
  int attempts = 0;
}

/// Negotiated discrete button-edge delivery mode on the Joystick -> PC -> Visor
/// Faro UDP link (`SAIA-FARO-009`).
enum VrFaroDiscreteDeliveryMode {
  /// Used when no Visor is paired, with `v1` Visors, or with legacy `v2` Visors
  /// that do not advertise `'cack': true` in `helloKind`.
  /// Bounded latency (`<= 24 ms` minimum pulse-width hold only), non-blocking
  /// continuous `120 Hz` analog stream, and guarded same-sequence best-effort
  /// backups (`+7 ms` / `+15 ms`) that cancel automatically whenever a newer
  /// sequence transmits so anti-replay never drops subsequent frames.
  legacyBestEffort,

  /// Used only when the paired Visor explicitly negotiates end-to-end
  /// `compactSeqAckKind` support (`'cack': true` -> `'e2eAck': true`).
  /// Discrete button edges use the bounded Stop-and-Wait FIFO queue confirmed
  /// exclusively by authentic end-to-end Visor sequence ACKs.
  endToEndVisorAck,
}

/// Authenticated LAN `VrControllerLink` connecting a Smartphone in **Modo
/// Joystick** directly to a **VRlizate Desktop (Estación Faro)** `3D Set`
/// coordinator (`Joystick ↔ PC ↔ Visor`).
class VrBeaconJoystickControllerLink
    implements VrControllerLink, VrCompactGamepadCapableLink {
  /// Maximum number of discrete button/edge transitions queued awaiting Visor
  /// sequence ACK (`32 * 68 B ~= 2.1 KiB`).
  static const int maxPendingDiscreteFrames = 32;

  /// Retry cadence while the FIFO head discrete edge awaits Visor ACK.
  static const Duration discreteRetryInterval = Duration(milliseconds: 7);

  /// Maximum same-sequence retransmissions per discrete edge (`12 * 7 ms ~=
  /// 84 ms`, ~91 ms expiry) before degrading to best-effort under a hard ACK
  /// partition so queued frames and `pointerPoseReady: false` transitions never
  /// stall past the `150 ms` posture freshness window (`SAIA-FARO-011`).
  static const int maxRetriesPerDiscreteEdge = 12;

  /// Maximum age (`150 ms`, matching `VrPhonePointerPosture.staleAfterUs`) for
  /// which a queued `pointerPoseReady: true` flag remains valid before being
  /// scrubbed to `false` (`SAIA-FARO-011`).
  static const int maxQueuedPostureFreshnessUs = 150000;

  VrBeaconJoystickControllerLink._(
    this._socket,
    this._stationAddress,
    this._stationPort,
    this.stationId,
    this.stationName,
    this._sessionKey,
    this._clockUs,
    this._visorEndToEndAckActive,
  ) : _stationAdvertisesVisorEndToEndAck = _visorEndToEndAckActive {
    _lastServerSeenUs = _clockUs();
    _telemetryTimer = Timer.periodic(
      const Duration(milliseconds: 500),
      (_) => _sendTelemetryHeartbeat(),
    );
    _sendTelemetryHeartbeat();
  }

  void _attachSocketSubscription(StreamSubscription<RawSocketEvent> sub) {
    _socketSub = sub;
  }

  final RawDatagramSocket _socket;
  final InternetAddress _stationAddress;
  final int _stationPort;
  final String stationId;
  final String stationName;
  final List<int> _sessionKey;
  final int Function() _clockUs;
  bool _stationAdvertisesVisorEndToEndAck;
  bool _visorEndToEndAckActive;
  int _lastDirectiveEpoch = 0;

  late final StreamController<Object?> _incoming =
      StreamController<Object?>.broadcast(onListen: _flushPendingIncoming);
  final List<Object?> _pendingIncoming = <Object?>[];
  final Queue<_PendingDiscreteEdge> _pendingDiscreteQueue =
      Queue<_PendingDiscreteEdge>();
  StreamSubscription<RawSocketEvent>? _socketSub;
  Timer? _telemetryTimer;
  Timer? _discreteRetryTimer;
  Timer? _ultraShortHoldTimer;
  Timer? _legacyBackupTimer1;
  Timer? _legacyBackupTimer2;
  VrCompactGamepadFrame? _deferredContinuousFrame;
  int _deferredContinuousEnqueuedAtUs = 0;
  int _lastQueuedButtonMask = 0;
  int? _lastQueuedPressUs;
  int? _lastTransmittedCompactSeq;
  bool _latestSenderPointerPoseReady = false;
  bool _lastTransmittedPointerPoseReady = false;
  int _unacknowledgedEdgesExpired = 0;
  int _consecutiveExpiredEdges = 0;

  bool _closed = false;
  int _lastServerSeenUs = 0;
  int _bridgeRevision = 0;
  int _recommendedHz = 60;
  bool _castOffloadAck = true;
  bool _visionOffloadAck = false;
  int _framesSent = 0;
  int _compactBinaryFramesSent = 0;
  int _redundantEdgeDatagramsSent = 0;

  int get framesSent => _framesSent;
  int get compactBinaryFramesSent => _compactBinaryFramesSent;
  int get redundantEdgeDatagramsSent => _redundantEdgeDatagramsSent;
  int get pendingDiscreteEdgeCount => _pendingDiscreteQueue.length;
  int get unacknowledgedEdgesExpired => _unacknowledgedEdgesExpired;
  bool get isEndToEndVisorAckActive => _visorEndToEndAckActive;
  VrFaroDiscreteDeliveryMode get discreteDeliveryMode => _visorEndToEndAckActive
      ? VrFaroDiscreteDeliveryMode.endToEndVisorAck
      : VrFaroDiscreteDeliveryMode.legacyBestEffort;

  @override
  bool get supportsCompactGamepadFrames => true;

  @override
  Stream<Object?> get messages => _incoming.stream;

  @override
  bool get isOpen => !_closed;

  /// Connects and authenticates with [invite] as the `joystick` role in the
  /// Computer's `3D Set`.
  static Future<VrBeaconJoystickControllerLink> connect(
    VrDesktopBeaconPairingInvite invite, {
    Duration timeout = const Duration(seconds: 3),
    int Function()? monotonicMicroseconds,
  }) async {
    if (kIsWeb) {
      throw const VrControllerConnectionException(
        'El enlace UDP del 3D Set requiere VRlizate nativo.',
      );
    }
    if (!VrDesktopBeaconWire.isValidPin(invite.pin)) {
      throw const VrControllerConnectionException(
        'El PIN de la Estación Faro debe tener 6 dígitos.',
      );
    }
    final clock =
        monotonicMicroseconds ??
        (() {
          final sw = Stopwatch()..start();
          return () => sw.elapsedMicroseconds;
        })();

    final resolved =
        InternetAddress.tryParse(invite.host) ??
        (await InternetAddress.lookup(
          invite.host,
        )).firstWhere((a) => a.type == InternetAddressType.IPv4);

    final socket = await RawDatagramSocket.bind(InternetAddress.anyIPv4, 0);
    socket.broadcastEnabled = true;
    var targetAddr = resolved;
    var targetPort = invite.port;
    final rng = math.Random.secure();
    final clientNonce = base64UrlEncode(
      List<int>.generate(16, (_) => rng.nextInt(256)),
    );
    final handshakeKey = VrDesktopBeaconWire.deriveHandshakeKey(
      invite.stationId,
      invite.pin,
    );
    final helloPacket = VrDesktopBeaconWire.encodeAuthenticatedJson(
      kind: VrDesktopBeaconWire.helloKind,
      key: handshakeKey,
      payload: {'cn': clientNonce, 'role': 'joystick', 'ts': clock()},
    );
    final discoveryProbe = VrDesktopBeaconWire.encodeDiscoveryProbe();

    final completer =
        Completer<({List<int> sessionKey, bool visorEndToEndAck})>();
    void Function(RawSocketEvent)? activeHandler;
    late final StreamSubscription<RawSocketEvent> sub;
    Timer? retryTimer;
    var helloAttempts = 0;

    void sendHello() {
      helloAttempts++;
      try {
        socket.send(helloPacket, targetAddr, targetPort);
      } catch (_) {}
      // If cached station port/IP changed after a Desktop restart, simultaneously
      // probe LAN discovery so the Joystick resolves the new port automatically!
      if (helloAttempts >= 2) {
        try {
          socket.send(
            discoveryProbe,
            targetAddr,
            VrDesktopBeaconWire.defaultDiscoveryPort,
          );
        } catch (_) {}
        try {
          socket.send(
            discoveryProbe,
            InternetAddress('255.255.255.255'),
            VrDesktopBeaconWire.defaultDiscoveryPort,
          );
        } catch (_) {}
      }
    }

    activeHandler = (event) {
      if (event != RawSocketEvent.read) return;
      Datagram? dg;
      while ((dg = socket.receive()) != null) {
        final raw = dg!.data;
        if (!VrDesktopBeaconWire.hasMagic(raw)) continue;
        if (raw[4] == VrDesktopBeaconWire.discoverAnnounceKind) {
          final info = VrDesktopBeaconWire.decodeDiscoveryAnnounce(
            raw,
            senderHost: dg.address.address,
            nowMicros: clock(),
          );
          if (info != null && info.stationId == invite.stationId) {
            targetAddr = dg.address;
            targetPort = info.port;
            try {
              socket.send(helloPacket, targetAddr, targetPort);
            } catch (_) {}
          }
          continue;
        }
        final map = VrDesktopBeaconWire.decodeAuthenticatedJson(
          expectedKind: VrDesktopBeaconWire.helloAckKind,
          key: handshakeKey,
          datagram: raw,
        );
        if (map == null) continue;
        final sn = map['sn'] as String?;
        final cn = map['cn'] as String?;
        if (sn == null || cn != clientNonce) continue;
        targetAddr = dg.address;
        targetPort = dg.port;
        final sessionKey = VrDesktopBeaconWire.deriveSessionKey(
          stationId: invite.stationId,
          pin: invite.pin,
          clientNonce: clientNonce,
          serverNonce: sn,
        );
        final initialE2eAck = map['e2eAck'] == true;
        if (!completer.isCompleted) {
          completer.complete((
            sessionKey: sessionKey,
            visorEndToEndAck: initialE2eAck,
          ));
        }
        return;
      }
    };
    sub = socket.listen((event) => activeHandler?.call(event));

    sendHello();
    retryTimer = Timer.periodic(
      const Duration(milliseconds: 350),
      (_) => sendHello(),
    );

    try {
      final handshake = await completer.future.timeout(timeout);
      retryTimer.cancel();
      final link = VrBeaconJoystickControllerLink._(
        socket,
        targetAddr,
        targetPort,
        invite.stationId,
        invite.stationName,
        handshake.sessionKey,
        clock,
        handshake.visorEndToEndAck,
      );
      activeHandler = link._onSocketEvent;
      link._attachSocketSubscription(sub);
      // Drain any UDP datagrams that arrived in the same socket read burst
      // immediately after helloAck (such as replayed controllerMode).
      link._onSocketEvent(RawSocketEvent.read);
      // Emit initial connected Desktop Bridge state immediately so the
      // Joystick HUD reflects 3D Set coordination right away.
      Future<void>.microtask(() {
        if (!link._closed && !link._incoming.isClosed) {
          link._emitIncoming(
            VrDesktopBridgeState(
              revision: ++link._bridgeRevision,
              stationConnected: true,
              stationName: invite.stationName,
              castOffloadedToDesktop: true,
              visionOffloadedToDesktop: true,
              recommendedJoystickHz: 120,
              activeDecisionSummary:
                  '3D Set activo · Coordinado por ${invite.stationName}',
            ).toJsonMessage(),
          );
        }
      });
      return link;
    } catch (_) {
      retryTimer.cancel();
      await sub.cancel();
      socket.close();
      throw const VrControllerConnectionException(
        'No se pudo vincular el mando con la Estación Faro (verifica IP/PIN).',
      );
    }
  }

  void _onSocketEvent(RawSocketEvent event) {
    if (_closed || event != RawSocketEvent.read) return;
    Datagram? dg;
    while ((dg = _socket.receive()) != null) {
      final data = dg!.data;
      if (!VrDesktopBeaconWire.hasMagic(data)) continue;
      final kind = data[4];
      if (kind == VrDesktopBeaconWire.compactSeqAckKind) {
        final ackSeq = VrDesktopBeaconWire.decodeCompactSeqAck(
          key: _sessionKey,
          datagram: data,
        );
        if (ackSeq != null) {
          _lastServerSeenUs = _clockUs();
          final poppedAny = _handleCompactSeqAck(ackSeq);
          final matchesRecentSeq =
              poppedAny ||
              (_lastTransmittedCompactSeq != null &&
                  ackSeq == _lastTransmittedCompactSeq);
          if (matchesRecentSeq &&
              (_lastDirectiveEpoch == 0 ||
                  _stationAdvertisesVisorEndToEndAck)) {
            _consecutiveExpiredEdges = 0;
            _setEndToEndVisorAckActive(true);
          }
        }
      } else if (kind == VrDesktopBeaconWire.visorControllerControlKind) {
        final frame = VrDesktopBeaconWire.decodeAuthenticatedRelayFrame(
          expectedKind: VrDesktopBeaconWire.visorControllerControlKind,
          key: _sessionKey,
          datagram: data,
        );
        if (frame != null && !_incoming.isClosed) {
          _lastServerSeenUs = _clockUs();
          final modeReq = VrControllerModeRequest.tryParse(frame);
          if (modeReq != null) {
            _pruneObsoleteModeRevisionFrames(modeReq.revision);
          }
          _emitIncoming(frame);
        }
      } else if (kind == VrDesktopBeaconWire.bridgeDirectiveKind) {
        final map = VrDesktopBeaconWire.decodeAuthenticatedJson(
          expectedKind: VrDesktopBeaconWire.bridgeDirectiveKind,
          key: _sessionKey,
          datagram: data,
        );
        if (map == null) continue;
        final directive = VrDesktopBridgeDirective.fromJson(map);
        if (directive == null) continue;
        if (directive.decisionEpoch <= _lastDirectiveEpoch) continue;
        _lastDirectiveEpoch = directive.decisionEpoch;
        _lastServerSeenUs = _clockUs();
        _recommendedHz = directive.recommendedJoystickHz;
        _castOffloadAck = directive.offloadCastToDesktop;
        _visionOffloadAck = directive.offloadJoystickVisionToDesktop;
        _stationAdvertisesVisorEndToEndAck = directive.visorEndToEndCompactAck;
        _setEndToEndVisorAckActive(directive.visorEndToEndCompactAck);
        if (!_incoming.isClosed) {
          _emitIncoming(
            directive
                .toJoystickBridgeState(revision: ++_bridgeRevision)
                .toJsonMessage(),
          );
        }
      }
    }
    // Keepalive check: if PC station disappears for >12s, close link so
    // PhoneControllerPage's automatic reconnect kicks in cleanly.
    if ((_clockUs() - _lastServerSeenUs) > 12000000) {
      unawaited(close());
    }
  }

  void _emitIncoming(Object? message) {
    if (_closed || _incoming.isClosed) return;
    if (_incoming.hasListener) {
      _incoming.add(message);
      return;
    }
    if (_pendingIncoming.length >= 8) {
      _pendingIncoming.removeAt(0);
    }
    _pendingIncoming.add(message);
  }

  void _flushPendingIncoming() {
    if (_pendingIncoming.isEmpty || _closed || _incoming.isClosed) return;
    final queued = List<Object?>.of(_pendingIncoming);
    _pendingIncoming.clear();
    Future<void>.microtask(() {
      if (_closed || _incoming.isClosed || !_incoming.hasListener) return;
      for (final item in queued) {
        _incoming.add(item);
      }
    });
  }

  void _sendTelemetryHeartbeat() {
    if (_closed) return;
    // Proactive dead-station watchdog on the 500 ms heartbeat timer (even when
    // zero UDP packets arrive from a restarted station) so PhoneControllerPage
    // triggers automatic reconnect within ~4.5 s!
    if ((_clockUs() - _lastServerSeenUs) > 4500000) {
      unawaited(close());
      return;
    }
    final packet = VrDesktopBeaconWire.encodeAuthenticatedJson(
      kind: VrDesktopBeaconWire.joystickTelemetryKind,
      key: _sessionKey,
      payload: VrJoystickDirectBridgeTelemetry(
        timestampMicros: _clockUs(),
        linkHz: _recommendedHz,
        castOffloadedAck: _castOffloadAck,
        visionOffloadedAck: _visionOffloadAck,
      ).toJson(),
    );
    try {
      _socket.send(packet, _stationAddress, _stationPort);
    } catch (_) {}
  }

  static int _compactButtonsMask(VrCompactGamepadFrame f) {
    var mask = 0;
    if (f.trigger) mask |= 0x0001;
    if (f.action) mask |= 0x0002;
    if (f.btnA) mask |= 0x0004;
    if (f.btnB) mask |= 0x0008;
    if (f.btnX) mask |= 0x0010;
    if (f.btnY) mask |= 0x0020;
    if (f.btnL) mask |= 0x0040;
    if (f.btnR) mask |= 0x0080;
    if (f.btnGrip) mask |= 0x0100;
    if (f.stickClick) mask |= 0x0200;
    if (f.recenter) mask |= 0x0400;
    if (f.btnZL) mask |= 0x0800;
    if (f.btnZR) mask |= 0x1000;
    return mask;
  }

  bool _handleCompactSeqAck(int ackSeq) {
    if (_closed || _pendingDiscreteQueue.isEmpty) return false;
    var poppedAny = false;
    while (_pendingDiscreteQueue.isNotEmpty) {
      final headSeq = _pendingDiscreteQueue.first.frame.sequence;
      final diff = (ackSeq - headSeq) & 0xFFFFFFFF;
      if (diff < 0x80000000) {
        _pendingDiscreteQueue.removeFirst();
        poppedAny = true;
      } else {
        break;
      }
    }
    if (poppedAny) {
      _discreteRetryTimer?.cancel();
      _discreteRetryTimer = null;
      _pumpDiscreteQueue();
    }
    return poppedAny;
  }

  void _pruneObsoleteModeRevisionFrames(int minModeRevision) {
    if (_closed) return;
    final deferredRev = _deferredContinuousFrame?.hostModeRevision;
    if (deferredRev != null && deferredRev < minModeRevision) {
      _deferredContinuousFrame = null;
    }
    var removedHead = false;
    while (_pendingDiscreteQueue.isNotEmpty) {
      final rev = _pendingDiscreteQueue.first.frame.hostModeRevision;
      if (rev != null && rev < minModeRevision) {
        _pendingDiscreteQueue.removeFirst();
        removedHead = true;
      } else {
        break;
      }
    }
    _pendingDiscreteQueue.removeWhere(
      (e) =>
          e.frame.hostModeRevision != null &&
          e.frame.hostModeRevision! < minModeRevision,
    );
    if (removedHead) {
      _discreteRetryTimer?.cancel();
      _discreteRetryTimer = null;
      _pumpDiscreteQueue();
    }
  }

  void _cancelLegacyBackupTimers() {
    _legacyBackupTimer1?.cancel();
    _legacyBackupTimer1 = null;
    _legacyBackupTimer2?.cancel();
    _legacyBackupTimer2 = null;
  }

  void _setEndToEndVisorAckActive(bool active) {
    if (_visorEndToEndAckActive == active) return;
    _visorEndToEndAckActive = active;
    if (active) {
      _consecutiveExpiredEdges = 0;
      _cancelLegacyBackupTimers();
    } else {
      _discreteRetryTimer?.cancel();
      _discreteRetryTimer = null;
      _pumpDiscreteQueue();
    }
  }

  void _armLegacyBackupsForFrame(
    VrCompactGamepadFrame frame, {
    required bool fastOnly,
  }) {
    _cancelLegacyBackupTimers();
    if (fastOnly) {
      _legacyBackupTimer1 = Timer(const Duration(milliseconds: 3), () {
        _legacyBackupTimer1 = null;
        if (_closed || _lastTransmittedCompactSeq != frame.sequence) return;
        _sendRawRelayFrame(frame, isRetry: true);
      });
      return;
    }
    _legacyBackupTimer1 = Timer(const Duration(milliseconds: 7), () {
      _legacyBackupTimer1 = null;
      if (_closed || _lastTransmittedCompactSeq != frame.sequence) return;
      _sendRawRelayFrame(frame, isRetry: true);
    });
    _legacyBackupTimer2 = Timer(const Duration(milliseconds: 15), () {
      _legacyBackupTimer2 = null;
      if (_closed || _lastTransmittedCompactSeq != frame.sequence) return;
      _sendRawRelayFrame(frame, isRetry: true);
    });
  }

  static VrCompactGamepadFrame _copyWithPointerPoseFalse(
    VrCompactGamepadFrame f,
  ) {
    if (!f.pointerPoseReady) return f;
    return VrCompactGamepadFrame(
      sequence: f.sequence,
      senderTimestampMicroseconds: f.senderTimestampMicroseconds,
      orientation: f.orientation,
      angularVelocity: f.angularVelocity,
      stickX: f.stickX,
      stickY: f.stickY,
      lookX: f.lookX,
      lookY: f.lookY,
      laserX: f.laserX,
      laserY: f.laserY,
      steering: f.steering,
      drivingTilt: f.drivingTilt,
      throttle: f.throttle,
      brake: f.brake,
      modeIndex: f.modeIndex,
      hostModeRevision: f.hostModeRevision,
      trigger: f.trigger,
      action: f.action,
      btnA: f.btnA,
      btnB: f.btnB,
      btnX: f.btnX,
      btnY: f.btnY,
      btnL: f.btnL,
      btnR: f.btnR,
      btnZL: f.btnZL,
      btnZR: f.btnZR,
      btnGrip: f.btnGrip,
      stickClick: f.stickClick,
      recenter: f.recenter,
      moveAtEdge: f.moveAtEdge,
      laserSlideActive: f.laserSlideActive,
      pointerPoseReady: false,
      controllerVisible: f.controllerVisible,
      drivingPaused: f.drivingPaused,
      motionAvailable: f.motionAvailable,
      passthroughActive: f.passthroughActive,
      spatialHudVisible: f.spatialHudVisible,
      passthroughMix: f.passthroughMix,
      rangeMeters: f.rangeMeters,
    );
  }

  VrCompactGamepadFrame _sanitizePostureFreshness(
    VrCompactGamepadFrame f, {
    required int capturedAtUs,
  }) {
    if (!f.pointerPoseReady) return f;
    if (!_latestSenderPointerPoseReady ||
        (_clockUs() - capturedAtUs) >= maxQueuedPostureFreshnessUs) {
      return _copyWithPointerPoseFalse(f);
    }
    return f;
  }

  void _scrubQueuedPostureToFalse() {
    for (final entry in _pendingDiscreteQueue) {
      if (entry.frame.pointerPoseReady) {
        entry.frame = _copyWithPointerPoseFalse(entry.frame);
      }
    }
    final deferred = _deferredContinuousFrame;
    if (deferred != null && deferred.pointerPoseReady) {
      _deferredContinuousFrame = _copyWithPointerPoseFalse(deferred);
    }
  }

  void _sendRawRelayFrame(Object data, {required bool isRetry}) {
    if (_closed) return;
    final packet = VrDesktopBeaconWire.encodeAuthenticatedRelayFrame(
      kind: VrDesktopBeaconWire.joystickControllerFrameKind,
      key: _sessionKey,
      frame: data,
    );
    if (packet == null) return;
    try {
      _socket.send(packet, _stationAddress, _stationPort);
      if (isRetry) {
        _redundantEdgeDatagramsSent++;
      } else {
        _framesSent++;
        if (data is VrCompactGamepadFrame) {
          _cancelLegacyBackupTimers();
          _compactBinaryFramesSent++;
          _lastTransmittedCompactSeq = data.sequence;
          _lastTransmittedPointerPoseReady = data.pointerPoseReady;
        }
      }
    } catch (_) {}
  }

  void _armDiscreteRetryTimer() {
    _discreteRetryTimer?.cancel();
    if (_closed || _pendingDiscreteQueue.isEmpty) {
      _discreteRetryTimer = null;
      return;
    }
    _discreteRetryTimer = Timer(discreteRetryInterval, () {
      _discreteRetryTimer = null;
      if (_closed || _pendingDiscreteQueue.isEmpty) return;
      final head = _pendingDiscreteQueue.first;
      if (head.holdActive) return;
      if (head.attempts > maxRetriesPerDiscreteEdge) {
        _pendingDiscreteQueue.removeFirst();
        _unacknowledgedEdgesExpired++;
        _consecutiveExpiredEdges++;
        if (_consecutiveExpiredEdges >= 1) {
          _setEndToEndVisorAckActive(false);
          return;
        }
        _pumpDiscreteQueue();
        return;
      }
      head.attempts++;
      head.frame = _sanitizePostureFreshness(
        head.frame,
        capturedAtUs: head.enqueuedAtUs,
      );
      _sendRawRelayFrame(head.frame, isRetry: true);
      _armDiscreteRetryTimer();
    });
  }

  void _pumpDiscreteQueue() {
    if (_closed) return;
    if (!_visorEndToEndAckActive) {
      _discreteRetryTimer?.cancel();
      _discreteRetryTimer = null;
      while (_pendingDiscreteQueue.isNotEmpty) {
        final head = _pendingDiscreteQueue.first;
        if (head.holdActive) {
          return;
        }
        _pendingDiscreteQueue.removeFirst();
        head.frame = _sanitizePostureFreshness(
          head.frame,
          capturedAtUs: head.enqueuedAtUs,
        );
        final hasFollowingDiscrete = _pendingDiscreteQueue.isNotEmpty;
        _sendRawRelayFrame(head.frame, isRetry: false);
        _armLegacyBackupsForFrame(head.frame, fastOnly: hasFollowingDiscrete);
        if (hasFollowingDiscrete) {
          final next = _pendingDiscreteQueue.first;
          if (!next.holdActive) {
            next.holdActive = true;
            _ultraShortHoldTimer?.cancel();
            _ultraShortHoldTimer = Timer(const Duration(milliseconds: 6), () {
              _ultraShortHoldTimer = null;
              if (_closed) return;
              next.holdActive = false;
              _pumpDiscreteQueue();
            });
          }
          return;
        }
      }
    } else {
      while (_pendingDiscreteQueue.isNotEmpty) {
        final head = _pendingDiscreteQueue.first;
        if (head.holdActive) {
          return;
        }
        if (head.attempts == 0) {
          head.attempts = 1;
          head.frame = _sanitizePostureFreshness(
            head.frame,
            capturedAtUs: head.enqueuedAtUs,
          );
          _sendRawRelayFrame(head.frame, isRetry: false);
          _armDiscreteRetryTimer();
          return;
        }
        if (_discreteRetryTimer == null || !_discreteRetryTimer!.isActive) {
          _armDiscreteRetryTimer();
        }
        return;
      }
      _discreteRetryTimer?.cancel();
      _discreteRetryTimer = null;
    }
    final trailing = _deferredContinuousFrame;
    _deferredContinuousFrame = null;
    if (trailing != null) {
      final lastSeq = _lastTransmittedCompactSeq;
      final advance = lastSeq == null
          ? 1
          : ((trailing.sequence - lastSeq) & 0xFFFFFFFF);
      if (advance > 0 && advance < 0x80000000) {
        final sanitizedTrailing = _sanitizePostureFreshness(
          trailing,
          capturedAtUs: _deferredContinuousEnqueuedAtUs,
        );
        final droppedPosture =
            _lastTransmittedPointerPoseReady &&
            !sanitizedTrailing.pointerPoseReady;
        _sendRawRelayFrame(sanitizedTrailing, isRetry: false);
        if (droppedPosture) {
          _armLegacyBackupsForFrame(sanitizedTrailing, fastOnly: false);
        }
      }
    }
  }

  @override
  void add(Object data) {
    if (_closed) return;
    if (data is! VrCompactGamepadFrame) {
      _sendRawRelayFrame(data, isRetry: false);
      return;
    }

    final nowUs = _clockUs();
    final prevSenderPoseReady = _latestSenderPointerPoseReady;
    _latestSenderPointerPoseReady = data.pointerPoseReady;
    if (!data.pointerPoseReady) {
      _scrubQueuedPostureToFalse();
    }

    final nextMask = _compactButtonsMask(data);
    final newlyPressed = (~_lastQueuedButtonMask & nextMask) != 0;
    final newlyReleased = (_lastQueuedButtonMask & ~nextMask) != 0;
    final hasDiscreteEdge = newlyPressed || newlyReleased;

    if (!hasDiscreteEdge) {
      if (_pendingDiscreteQueue.isEmpty) {
        _deferredContinuousFrame = null;
        final droppedPosture =
            (prevSenderPoseReady || _lastTransmittedPointerPoseReady) &&
            !data.pointerPoseReady;
        _sendRawRelayFrame(data, isRetry: false);
        if (droppedPosture) {
          _armLegacyBackupsForFrame(data, fastOnly: false);
        }
      } else {
        // Coalesce latest continuous snapshot behind pending discrete edges so
        // its sequence never jumps ahead of an unacknowledged button transition.
        _deferredContinuousFrame = data;
        _deferredContinuousEnqueuedAtUs = nowUs;
      }
      return;
    }

    _deferredContinuousFrame = null;
    if (_pendingDiscreteQueue.isNotEmpty) {
      // A newer discrete edge arrived: lift any remaining ultra-short release
      // pulse hold on earlier queued edges so FIFO flushes without delay.
      _ultraShortHoldTimer?.cancel();
      _ultraShortHoldTimer = null;
      for (final queued in _pendingDiscreteQueue) {
        queued.holdActive = false;
      }
    }

    var holdDelay = Duration.zero;
    if (newlyPressed) {
      _lastQueuedPressUs = nowUs;
    } else if (newlyReleased && _lastQueuedPressUs != null) {
      final elapsedUs = nowUs - _lastQueuedPressUs!;
      if (elapsedUs >= 0 && elapsedUs < 24000) {
        holdDelay = Duration(microseconds: 24000 - elapsedUs);
      }
    }
    _lastQueuedButtonMask = nextMask;

    while (_pendingDiscreteQueue.length >= maxPendingDiscreteFrames) {
      _pendingDiscreteQueue.removeFirst();
      _unacknowledgedEdgesExpired++;
      _discreteRetryTimer?.cancel();
      _discreteRetryTimer = null;
    }

    final entry = _PendingDiscreteEdge(
      frame: data,
      holdActive: holdDelay > Duration.zero,
      enqueuedAtUs: nowUs,
    );
    _pendingDiscreteQueue.addLast(entry);
    if (entry.holdActive) {
      _ultraShortHoldTimer?.cancel();
      _ultraShortHoldTimer = Timer(holdDelay, () {
        _ultraShortHoldTimer = null;
        if (_closed) return;
        entry.holdActive = false;
        _pumpDiscreteQueue();
      });
    }
    _pumpDiscreteQueue();
  }

  @override
  Future<void> close([int? code, String? reason]) async {
    if (_closed) return;
    _closed = true;
    _cancelLegacyBackupTimers();
    _discreteRetryTimer?.cancel();
    _discreteRetryTimer = null;
    _ultraShortHoldTimer?.cancel();
    _ultraShortHoldTimer = null;
    _pendingDiscreteQueue.clear();
    _deferredContinuousFrame = null;
    _telemetryTimer?.cancel();
    _telemetryTimer = null;
    await _socketSub?.cancel();
    _socketSub = null;
    _socket.close();
    await _incoming.close();
  }
}

/// Duplex `VrControllerLink` exposed on the **Visor** (`VrDesktopBeaconClient`)
/// so [VrRemoteControllerService.attachControllerLink] receives Joystick inputs
/// coordinated by the Computer (`Estación Faro`).
class VrBeaconVisorControllerLink
    implements
        VrControllerLink,
        VrCompactGamepadCapableLink,
        VrCompactGamepadAckSink {
  VrBeaconVisorControllerLink({
    required this.onSendControl,
    this.onAcknowledgeCompactSequence,
  });

  final void Function(Object data) onSendControl;
  final void Function(int sequence)? onAcknowledgeCompactSequence;
  final StreamController<Object?> _incoming =
      StreamController<Object?>.broadcast();
  bool _open = true;

  @override
  bool get supportsCompactGamepadFrames => true;

  @override
  Stream<Object?> get messages => _incoming.stream;

  @override
  bool get isOpen => _open;

  void deliverFromStation(Object frame) {
    if (!_open || _incoming.isClosed) return;
    _incoming.add(frame);
  }

  @override
  void acknowledgeCompactGamepadSequence(int sequence) {
    if (!_open) return;
    onAcknowledgeCompactSequence?.call(sequence);
  }

  @override
  void add(Object data) {
    if (!_open) return;
    onSendControl(data);
  }

  @override
  Future<void> close([int? code, String? reason]) async {
    if (!_open) return;
    _open = false;
    await _incoming.close();
  }
}
