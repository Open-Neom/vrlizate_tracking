import 'dart:async';
import 'dart:convert';

import 'package:flutter/foundation.dart';
import 'package:flutter/services.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:vrlizate/vrlizate.dart' show VrDirectDeviceBridge;
import 'package:vrlizate_scene/vrlizate_scene.dart';

export 'package:vrlizate_scene/vrlizate_scene.dart'
    show
        VrDeviceCapabilities,
        VrDeviceClass,
        VrDeviceFeatureProfile,
        VrDisplayMode,
        VrPopularDeviceCatalog;

/// Catalog-based initial quality preference; this is not a hardware benchmark.
/// Actual scene timings and physical headset calibration remain necessary.
class VrDeviceProfiler {
  static const String _prefKey = 'vrlizate_quality_tier';
  static const _channel = MethodChannel('vrlizate/device_capabilities');
  static Future<void>? _initialization;
  static VrDeviceCapabilities? _capabilities;
  static VrDevicePerformanceProfile? _profile;
  static VrDeviceFeatureProfile? _featureProfile;

  static VrDeviceCapabilities? get capabilities => _capabilities;
  static VrDevicePerformanceProfile? get profile => _profile;
  static VrDeviceFeatureProfile get featureProfile =>
      _featureProfile ?? VrPopularDeviceCatalog.resolve(_capabilities);

  /// Reads Android once, before runApp. Missing/slow native hosts keep defaults.
  /// Does not select a display mode or load/overwrite a user's saved preset.
  static Future<void> initialize() => _initialization ??= _readCapabilities();

  static bool get _isDirectNativeHostMatch {
    if (kIsWeb) return false;
    final direct = VrDirectDeviceBridge.instance;
    if (!direct.isAvailable) return false;
    return switch (defaultTargetPlatform) {
      TargetPlatform.android => direct.androidOpenXrHandlesSync() != null,
      TargetPlatform.iOS || TargetPlatform.macOS => true,
      _ => false,
    };
  }

  /// Synchronously reads OS thermal severity (`0..6`) via Direct Native Interop
  /// (`NSProcessInfo.thermalState` on iOS/macOS, `PowerManager.getCurrentThermalStatus`
  /// via JNI on Android) with zero `MethodChannel` round-trips, or `null` when
  /// unavailable.
  static int? readThermalStatusSync() {
    if (!_isDirectNativeHostMatch) return null;
    final status = VrDirectDeviceBridge.instance.readThermalStatusSync();
    return status != null && status >= 0 && status <= 6 ? status : null;
  }

  /// Reads current OS thermal severity without falling back to a cached
  /// device snapshot. An unavailable reading is represented by null.
  static Future<int?> readThermalStatus() async {
    final directStatus = readThermalStatusSync();
    if (directStatus != null) return directStatus;
    if (kIsWeb || defaultTargetPlatform != TargetPlatform.android) return null;
    try {
      final response = await _channel
          .invokeMethod<Object?>('readRuntimeEnvironment')
          .timeout(const Duration(milliseconds: 800));
      final status = response is Map ? response['thermalStatus'] : null;
      return status is int && status >= 0 && status <= 6 ? status : null;
    } on MissingPluginException {
      return null;
    } on PlatformException {
      return null;
    } on TimeoutException {
      return null;
    }
  }

  /// Refreshes live Wi-Fi band, audio route, display mode, and thermal state
  /// without resetting the startup quality profile.
  static Future<VrDeviceCapabilities?> refreshRuntimeEnvironment() async {
    if (kIsWeb) return _capabilities;
    final current = _capabilities;
    if (current == null) return null;
    if (_isDirectNativeHostMatch) {
      final direct = VrDirectDeviceBridge.instance.readDeviceCapabilitiesSync();
      if (direct != null) {
        _capabilities = current.copyWithRuntimeEnvironment(direct);
        _featureProfile = VrPopularDeviceCatalog.resolve(_capabilities);
        return _capabilities;
      }
    }
    if (defaultTargetPlatform != TargetPlatform.android) {
      return _capabilities;
    }
    try {
      final response = await _channel
          .invokeMethod<Object?>('readRuntimeEnvironment')
          .timeout(const Duration(milliseconds: 800));
      if (response is Map) {
        _capabilities = current.copyWithRuntimeEnvironment(response);
        _featureProfile = VrPopularDeviceCatalog.resolve(_capabilities);
      }
    } catch (_) {}
    return _capabilities;
  }

  static Future<void> _readCapabilities() async {
    if (kIsWeb) return;
    final supportsDirect = _isDirectNativeHostMatch;
    if (!supportsDirect && defaultTargetPlatform != TargetPlatform.android) {
      return;
    }
    var status = 'unavailable';
    double? directCapsUs;
    double? directThermalUs;
    double? channelThermalUs;
    try {
      Object? response;
      if (supportsDirect) {
        final swDirectCaps = Stopwatch()..start();
        response = VrDirectDeviceBridge.instance.readDeviceCapabilitiesSync();
        swDirectCaps.stop();
        directCapsUs = swDirectCaps.elapsedMicroseconds.toDouble();
        // Warm 64-iteration micro-probe of direct thermal status vs channel.
        for (var i = 0; i < 8; i++) {
          VrDirectDeviceBridge.instance.readThermalStatusSync();
        }
        final swDirectThermal = Stopwatch()..start();
        for (var i = 0; i < 64; i++) {
          VrDirectDeviceBridge.instance.readThermalStatusSync();
        }
        swDirectThermal.stop();
        directThermalUs = swDirectThermal.elapsedMicroseconds / 64.0;
        if (defaultTargetPlatform == TargetPlatform.android) {
          try {
            await _channel
                .invokeMethod<Object?>('readRuntimeEnvironment')
                .timeout(const Duration(milliseconds: 250));
            final swChan = Stopwatch()..start();
            for (var i = 0; i < 16; i++) {
              await _channel
                  .invokeMethod<Object?>('readRuntimeEnvironment')
                  .timeout(const Duration(milliseconds: 250));
            }
            swChan.stop();
            channelThermalUs = swChan.elapsedMicroseconds / 16.0;
          } catch (_) {}
        }
      }
      if (response == null && defaultTargetPlatform == TargetPlatform.android) {
        response = await _channel
            .invokeMethod<Object?>('read')
            .timeout(const Duration(seconds: 2));
      }
      _capabilities = VrDeviceCapabilities.fromPlatform(response);
      final device = _capabilities;
      if (device != null) {
        _profile = VrDevicePerformanceCatalog.match(
          manufacturer: device.manufacturer,
          model: device.model,
        );
        _featureProfile = VrPopularDeviceCatalog.resolve(device);
        status = _profile != null
            ? 'catalog_match'
            : (_featureProfile!.isExplicitOrHardwareClassified
                  ? 'feature_classified'
                  : 'unknown_model');
      } else {
        status = 'invalid_response';
      }
    } on MissingPluginException {
      status = 'missing_plugin';
    } on PlatformException {
      status = 'platform_error';
    } on TimeoutException {
      status = 'timeout';
    } catch (_) {
      status = 'invalid_response';
    }
    debugPrint(
      '[VR_DEVICE] ${jsonEncode({'status': status, 'capabilities': _capabilities?.toJson(), 'profile': _profile?.id, 'featureLine': _featureProfile?.popularLineId, 'deviceClass': _featureProfile?.deviceClass.name, 'initialTier': detectInitialPreset().tier.name, 'catalogDisplayMaxHz': _profile?.displayMaxHz, 'directCapsUs': directCapsUs, 'directThermalUs': directThermalUs, 'channelThermalUs': channelThermalUs, 'measuredFps': null})}',
    );
  }

  /// Chooses a starting budget; supported Hz do not predict rendered FPS.
  static VrQualityPreset detectInitialPreset() {
    if (kIsWeb) return VrQualityPreset.medium;
    return switch (defaultTargetPlatform) {
      TargetPlatform.android =>
        _profile != null
            ? VrQualityPreset.forTier(_profile!.initialTier)
            : (_featureProfile != null &&
                      _featureProfile!.isExplicitOrHardwareClassified
                  ? VrQualityPreset.forTier(
                      _featureProfile!.recommendedVisorTier,
                    )
                  : VrQualityPreset.medium),
      TargetPlatform.iOS => VrQualityPreset.high,
      TargetPlatform.macOS ||
      TargetPlatform.windows ||
      TargetPlatform.linux => VrQualityPreset.ultra,
      TargetPlatform.fuchsia => VrQualityPreset.medium,
    };
  }

  /// Loads the persisted user quality preset preference if available.
  static Future<VrQualityPreset> loadPreferredPreset() async {
    try {
      final prefs = await SharedPreferences.getInstance();
      final tierName = prefs.getString(_prefKey);
      if (tierName != null) {
        for (final tier in VrQualityTier.values) {
          if (tier.name == tierName) {
            return VrQualityPreset.forTier(tier);
          }
        }
      }
    } catch (_) {}
    return detectInitialPreset();
  }

  /// Persists the user quality preset preference.
  static Future<void> savePreferredPreset(VrQualityPreset preset) async {
    try {
      final prefs = await SharedPreferences.getInstance();
      await prefs.setString(_prefKey, preset.tier.name);
    } catch (_) {}
  }

  /// Clears startup state between tests; never needed during an app session.
  @visibleForTesting
  static void resetForTesting() {
    _initialization = null;
    _capabilities = null;
    _profile = null;
    _featureProfile = null;
  }
}
