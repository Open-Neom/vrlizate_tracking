/// VRlizate SDK 6DoF optical camera tracking, Estación Faro 3-device spatial
/// fusion, passthrough camera feeds, tri-camera HUDs, and device profiling.
library;

export 'package:vrlizate/vrlizate.dart'
    show
        CalibratedCameraView,
        CameraDistortionIntrinsics,
        CausalLandmarkStabilizer3D,
        ForwardKinematicChainRigidifier,
        KabschHorizonAligner,
        MultiViewDltTriangulator,
        MultiViewTriangulationResult,
        RigidTransform3D,
        RollingMedianBoneEstimator,
        StabilizedLandmark3D,
        VelocitySpikeGate3D,
        VrAnthropometricSegmentRatios;

export 'beacon/vr_beacon_dead_reckoning.dart';
export 'beacon/vr_beacon_joystick_link.dart';
export 'beacon/vr_desktop_autonomous_bridge.dart';
export 'beacon/vr_desktop_beacon_client.dart';
export 'beacon/vr_desktop_beacon_models.dart';
export 'beacon/vr_desktop_beacon_page.dart';
export 'beacon/vr_desktop_beacon_station.dart';
export 'beacon/vr_desktop_beacon_wire.dart';
export 'beacon/vr_desktop_capabilities.dart';
export 'beacon/vr_desktop_media_vault.dart';
export 'beacon/vr_desktop_vision_platform.dart';
export 'beacon/vr_fiducial_tag_codec.dart';
export 'beacon/vr_private_lan_loopback_relay.dart';
export 'beacon/vr_tri_node_depth_solver.dart';
export 'session/vr_device_profiler.dart';
export 'session/vr_joystick_mini_frame_painter.dart';
export 'session/vr_marker_orientation_resolver.dart';
export 'session/vr_passthrough_camera_feed.dart';
export 'session/vr_spatial_tracking_service.dart';
export 'session/vr_spatial_tracking_settings.dart';
export 'session/vr_stereo_spatial_inspector_hud.dart';
export 'session/vr_tri_camera_frustum_painter.dart';
export 'session/vr_tri_camera_live_inspector_card.dart';
export 'session/vr_visual_marker_pose.dart';
export 'session/vr_visual_tracking_panel.dart';
export 'session/vr_visual_tracking_session.dart';
export 'utils/constants/vr_constants.dart';
export 'utils/constants/vr_translation_constants.dart';
