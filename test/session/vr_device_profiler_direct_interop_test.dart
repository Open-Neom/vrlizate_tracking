import 'dart:io';

import 'package:flutter/foundation.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:vrlizate_tracking/vrlizate_tracking.dart';

void main() {
  TestWidgetsFlutterBinding.ensureInitialized();

  tearDown(() {
    debugDefaultTargetPlatformOverride = null;
    VrDeviceProfiler.resetForTesting();
    TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger
        .setMockMethodCallHandler(
          const MethodChannel('vrlizate/device_capabilities'),
          null,
        );
  });

  test('VrDeviceProfiler reads live macOS capabilities and thermal status via Direct Native Interop', () async {
    if (!Platform.isMacOS) return;
    debugDefaultTargetPlatformOverride = TargetPlatform.macOS;
    VrDeviceProfiler.resetForTesting();

    final syncThermal = VrDeviceProfiler.readThermalStatusSync();
    expect(syncThermal, isNotNull);
    expect(syncThermal, inInclusiveRange(0, 4));

    final asyncThermal = await VrDeviceProfiler.readThermalStatus();
    expect(asyncThermal, equals(syncThermal));

    await VrDeviceProfiler.initialize();
    final caps = VrDeviceProfiler.capabilities;
    expect(caps, isNotNull);
    expect(caps!.manufacturer, equals('Apple'));
    expect(caps.totalRamMb, greaterThanOrEqualTo(1024));
  });

  test('VrDeviceProfiler preserves Android MethodChannel mock fallback in tests', () async {
    debugDefaultTargetPlatformOverride = TargetPlatform.android;
    VrDeviceProfiler.resetForTesting();

    const channel = MethodChannel('vrlizate/device_capabilities');
    TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger
        .setMockMethodCallHandler(channel, (call) async {
          if (call.method == 'read') {
            return <String, Object?>{
              'manufacturer': 'samsung',
              'model': 'SM-S928B',
              'brand': 'samsung',
              'device': 'e3q',
              'hardware': 'qcom',
              'sdkInt': 35,
              'mediaPerformanceClass': 35,
              'totalRamMb': 12288,
              'hasHardwareGyroscope': true,
              'hasConcurrentCameras': true,
              'supportsWifi5Ghz': true,
              'supportedModes': <Map<String, Object?>>[
                <String, Object?>{
                  'id': 1,
                  'width': 3120,
                  'height': 1440,
                  'refreshRateHz': 120.0,
                },
              ],
            };
          }
          if (call.method == 'readRuntimeEnvironment') {
            return <String, Object?>{'thermalStatus': 2};
          }
          return null;
        });

    await VrDeviceProfiler.initialize();
    expect(VrDeviceProfiler.capabilities?.model, equals('SM-S928B'));
    expect(await VrDeviceProfiler.readThermalStatus(), equals(2));
  });
}
