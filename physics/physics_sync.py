import time

import bpy

from .physics_world import PhysicsWorld, capture_initial_snapshot, restore_last_initial_snapshot


_controller = None
_REGISTRY_KEY = "pmx_physics.controllers"


def _controller_registry():
    registry = bpy.app.driver_namespace.get(_REGISTRY_KEY)
    if not isinstance(registry, list):
        registry = []
        bpy.app.driver_namespace[_REGISTRY_KEY] = registry
    return registry


def _known_controllers():
    controllers = []
    seen = set()
    if _controller is not None:
        controllers.append(_controller)
        seen.add(id(_controller))
    for controller in list(_controller_registry()):
        if id(controller) in seen:
            continue
        controllers.append(controller)
        seen.add(id(controller))
    return controllers


def _register_controller(controller):
    registry = _controller_registry()
    if all(item is not controller for item in registry):
        registry.append(controller)


def _unregister_controller(controller):
    registry = _controller_registry()
    registry[:] = [
        item
        for item in registry
        if item is not controller and bool(getattr(item, "running", False))
    ]


def _clear_current_controller(controller):
    global _controller
    if _controller is controller:
        _controller = None


def _disable_blender_rigidbody_world(scene):
    rigidbody_world = getattr(scene, "rigidbody_world", None)
    if rigidbody_world is None:
        return None, None
    previous_enabled = bool(rigidbody_world.enabled)
    rigidbody_world.enabled = False
    return rigidbody_world, previous_enabled


def _restore_blender_rigidbody_world(rigidbody_world, previous_enabled):
    if rigidbody_world is not None and previous_enabled is not None:
        rigidbody_world.enabled = previous_enabled


def is_active():
    return any(bool(getattr(controller, "running", False)) for controller in _known_controllers())


def _time_scale(settings):
    return max(0.0, float(getattr(settings, "time_scale", 1.0)))


def _fixed_timestep(settings):
    resolver = getattr(settings, "effective_fixed_timestep", None)
    if resolver is not None:
        return float(resolver())
    return float(settings.fixed_timestep)


def _effective_gravity(settings):
    resolver = getattr(settings, "effective_gravity", None)
    if resolver is not None:
        return resolver()
    scale = float(getattr(settings, "gravity_scale", 1.0))
    return tuple(float(value) * scale for value in settings.gravity)


def _publish_performance(settings, world):
    if not bool(getattr(settings, "show_performance_stats", False)):
        return
    perf = getattr(world, "performance", None)
    if not isinstance(perf, dict):
        return
    settings.perf_body_count = int(perf.get("body_count", 0))
    settings.perf_joint_count = int(perf.get("joint_count", 0))
    settings.perf_pair_count = int(perf.get("pair_count", 0))
    settings.perf_last_step_ms = float(perf.get("last_step_ms", 0.0))
    settings.perf_avg_step_ms = float(perf.get("avg_step_ms", 0.0))
    settings.perf_max_step_ms = float(perf.get("max_step_ms", 0.0))
    settings.perf_step_count = int(perf.get("step_count", 0))
    settings.perf_last_tick_ms = float(perf.get("last_tick_ms", 0.0))
    settings.perf_avg_tick_ms = float(perf.get("avg_tick_ms", 0.0))
    settings.perf_max_tick_ms = float(perf.get("max_tick_ms", 0.0))
    settings.perf_last_flush_ms = float(perf.get("last_flush_ms", 0.0))
    settings.perf_avg_flush_ms = float(perf.get("avg_flush_ms", 0.0))
    settings.perf_max_flush_ms = float(perf.get("max_flush_ms", 0.0))
    settings.perf_last_collect_ms = float(perf.get("last_collect_ms", 0.0))
    settings.perf_avg_collect_ms = float(perf.get("avg_collect_ms", 0.0))
    settings.perf_max_collect_ms = float(perf.get("max_collect_ms", 0.0))
    settings.perf_last_native_ms = float(perf.get("last_native_ms", 0.0))
    settings.perf_avg_native_ms = float(perf.get("avg_native_ms", 0.0))
    settings.perf_max_native_ms = float(perf.get("max_native_ms", 0.0))
    settings.perf_last_readback_ms = float(perf.get("last_readback_ms", 0.0))
    settings.perf_avg_readback_ms = float(perf.get("avg_readback_ms", 0.0))
    settings.perf_max_readback_ms = float(perf.get("max_readback_ms", 0.0))
    settings.perf_last_apply_ms = float(perf.get("last_apply_ms", 0.0))
    settings.perf_avg_apply_ms = float(perf.get("avg_apply_ms", 0.0))
    settings.perf_max_apply_ms = float(perf.get("max_apply_ms", 0.0))
    settings.perf_last_tick_steps = int(perf.get("last_tick_steps", 0))
    settings.perf_last_smoothing_segments = int(perf.get("last_smoothing_segments", 1))
    settings.perf_max_smoothing_segments = int(perf.get("max_smoothing_segments", 1))
    settings.perf_last_bone_writes = int(perf.get("last_bone_writes", 0))
    settings.perf_last_object_writes = int(perf.get("last_object_writes", 0))


def _scene_fps(scene):
    fps_base = float(getattr(scene.render, "fps_base", 1.0))
    if fps_base <= 0.0:
        fps_base = 1.0
    return max(1.0, float(getattr(scene.render, "fps", 30)) / fps_base)


def _scene_frame_start(scene):
    return int(getattr(scene, "frame_start", 1))


def _timeline_warmup_limit(settings):
    return max(0, min(10000, int(getattr(settings, "bake_preroll", 30))))


def _startup_sync_steps(settings):
    return max(0, min(300, int(getattr(settings, "startup_sync_steps", 30))))


def _realtime_apply_options(settings):
    return {
        "update_rigid_objects": bool(getattr(settings, "realtime_update_rigid_objects", False)),
        "skip_unchanged_bones": bool(getattr(settings, "realtime_skip_unchanged_bones", True)),
        "write_location_threshold": max(0.0, float(getattr(settings, "realtime_write_location_threshold", 0.00015))),
        "write_rotation_threshold": max(0.0, float(getattr(settings, "realtime_write_rotation_threshold", 0.02))),
    }


def _full_apply_options():
    return {
        "update_rigid_objects": True,
        "skip_unchanged_bones": False,
        "write_location_threshold": 0.0,
        "write_rotation_threshold": 0.0,
    }


def _substeps_per_frame(scene, settings):
    seconds_per_frame = 1.0 / _scene_fps(scene)
    fixed_step = _fixed_timestep(settings)
    substeps = max(1, round(seconds_per_frame / fixed_step))
    return substeps, seconds_per_frame / substeps


def _advance_frame_span(world, scene, settings, frame_delta):
    if frame_delta <= 0:
        return
    substeps_per_frame, timestep = _substeps_per_frame(scene, settings)
    for _ in range(frame_delta * substeps_per_frame):
        world.step(timestep * _time_scale(settings), 1)


def _advance_scene_frames(world, scene, settings, last_frame, target_frame):
    if target_frame <= last_frame:
        return
    for frame in range(last_frame + 1, target_frame + 1):
        if int(scene.frame_current) != frame:
            scene.frame_set(frame)
        world.flush_depsgraph()
        _advance_frame_span(world, scene, settings, 1)
        world.flush_depsgraph()


def _initialize_timeline_world(context, settings, world, root):
    scene = context.scene
    target_frame = int(scene.frame_current)
    start_frame = min(target_frame, _scene_frame_start(scene))
    root_for_snapshot = root
    if root_for_snapshot is None:
        from . import pmx_data_reader

        root_for_snapshot = pmx_data_reader.find_root_object(context.active_object)
    original_snapshot = None
    if root_for_snapshot is not None:
        from . import pmx_data_reader

        try:
            original_snapshot = capture_initial_snapshot(pmx_data_reader.read_model(context, root_for_snapshot))
        except Exception:
            original_snapshot = None

    scene.frame_set(start_frame)
    _initialize_world(context, settings, world, root, apply_options=_full_apply_options())
    if original_snapshot is not None:
        world.set_initial_snapshot(original_snapshot)

    world.flush_depsgraph()
    preroll = min(_timeline_warmup_limit(settings), max(0, target_frame - start_frame))
    if preroll > 0:
        _advance_frame_span(world, scene, settings, preroll)
        world.flush_depsgraph()

    _advance_scene_frames(world, scene, settings, start_frame, target_frame)

    if scene.frame_current != target_frame:
        scene.frame_set(target_frame)
        world.flush_depsgraph()
    _publish_performance(settings, world)


def _has_armature_action(world):
    model = getattr(world, "model", None)
    armature = getattr(model, "armature", None)
    if armature is None:
        return False
    animation_data = getattr(armature, "animation_data", None)
    if animation_data is None:
        return False
    if animation_data.action is not None:
        return True
    return any(not track.mute and any(strip.action is not None for strip in track.strips) for track in animation_data.nla_tracks)


def _initialize_world(context, settings, world, root, startup_sync=True, apply_options=None):
    if apply_options is None:
        apply_options = _realtime_apply_options(settings)
    world.initialize(
        context,
        root,
        settings.resolved_dll_path(),
        _effective_gravity(settings),
        settings.solver_iterations,
        settings.use_frame_offset,
        settings.joint_stop_erp,
        settings.joint_stop_cfm,
        settings.locked_joint_stop_erp,
        settings.locked_joint_stop_cfm,
        settings.prewarm_steps,
        settings.kinematic_smoothing,
        settings.kinematic_smoothing_steps,
        settings.kinematic_smoothing_move,
        settings.kinematic_smoothing_angle,
        _startup_sync_steps(settings) if startup_sync else 0,
        settings.locked_joint_pullback,
        settings.resting_body_stabilization,
        apply_options,
    )
    _publish_performance(settings, world)


class TimerController:
    def __init__(self, world, settings, scene):
        self.world = world
        self.settings = settings
        self.scene = scene
        self.running = True
        self.stopped = False
        self.timer_callback = self.tick
        self.last_time = time.perf_counter()
        self.accumulator = 0.0
        self.last_frame = int(scene.frame_current)
        self.has_armature_action = _has_armature_action(world)
        self._internal_frame_set = False
        self.rigidbody_world, self.previous_rigidbody_enabled = _disable_blender_rigidbody_world(scene)

    def tick(self):
        if not self.running:
            return None

        tick_start = time.perf_counter()
        try:
            now = time.perf_counter()
            elapsed = max(0.0, min(now - self.last_time, 0.25))
            self.last_time = now
            self.accumulator += elapsed

            fixed_step = _fixed_timestep(self.settings)
            if self._sync_changed_scene_frame(fixed_step):
                return max(0.001, fixed_step * 0.5)

            steps = 0
            max_steps = int(self.settings.max_substeps)
            if self.accumulator < fixed_step:
                return max(0.001, min(fixed_step * 0.5, fixed_step - self.accumulator))

            self.world.configure_apply_options(_full_apply_options())
            flush_start = time.perf_counter()
            self.world.flush_depsgraph()
            flush_ms = (time.perf_counter() - flush_start) * 1000.0
            while self.accumulator >= fixed_step and steps < max_steps:
                next_steps = steps + 1
                apply_results = self.accumulator < fixed_step * 2.0 or next_steps >= max_steps
                self.world.step(fixed_step * _time_scale(self.settings), 1, apply_results=apply_results)
                self.accumulator -= fixed_step
                steps += 1
            if steps:
                flush_start = time.perf_counter()
                self.world.flush_depsgraph()
                flush_ms += (time.perf_counter() - flush_start) * 1000.0
                self.world.record_flush_time(flush_ms)
                self.world.record_tick_time((time.perf_counter() - tick_start) * 1000.0, steps)
                _publish_performance(self.settings, self.world)

            return max(0.001, fixed_step * 0.5)
        except Exception as exc:
            self.settings.status = str(exc)
            self.settings.is_running = False
            self.stop()
            return None

    def _sync_changed_scene_frame(self, fixed_step):
        if self._internal_frame_set:
            return False
        frame = int(self.scene.frame_current)
        if frame == self.last_frame:
            return False

        frame_delta = frame - self.last_frame
        self.world.flush_depsgraph()
        if frame_delta <= 0:
            self.world.reset_to_current_pose()
        else:
            if frame_delta > 1:
                self.settings.status = f"{frame_delta} {bpy.app.translations.pgettext_iface('frame catch-up')}"
            try:
                self._internal_frame_set = True
                _advance_scene_frames(self.world, self.scene, self.settings, self.last_frame, frame)
            finally:
                self._internal_frame_set = False

        self.last_frame = frame
        self.accumulator = 0.0
        _publish_performance(self.settings, self.world)
        return True

    def stop(self, restore_initial=False):
        if self.stopped:
            return
        self.running = False
        self.stopped = True
        try:
            if bpy.app.timers.is_registered(self.timer_callback):
                bpy.app.timers.unregister(self.timer_callback)
        except Exception:
            pass
        try:
            self.world.destroy(restore_initial=restore_initial)
        finally:
            _restore_blender_rigidbody_world(self.rigidbody_world, self.previous_rigidbody_enabled)
            _unregister_controller(self)
            _clear_current_controller(self)


class TimelineController:
    def __init__(self, world, settings, scene):
        self.world = world
        self.settings = settings
        self.running = True
        self.stopped = False
        self.last_frame = int(scene.frame_current)
        self.handler = self.frame_change
        self._internal_frame_set = False
        self.rigidbody_world, self.previous_rigidbody_enabled = _disable_blender_rigidbody_world(scene)

    def frame_change(self, scene, depsgraph=None):
        if not self.running:
            return
        if self._internal_frame_set:
            return

        try:
            frame = int(scene.frame_current)
            if frame == self.last_frame:
                return
            if frame < self.last_frame:
                self.world.reset_to_current_pose()
                self.last_frame = frame
                return

            self.world.configure_apply_options(_realtime_apply_options(self.settings))
            self.world.flush_depsgraph()
            frame_delta = frame - self.last_frame
            if frame_delta > 1:
                self.settings.status = f"{frame_delta} {bpy.app.translations.pgettext_iface('frame catch-up')}"
            try:
                self._internal_frame_set = True
                _advance_scene_frames(self.world, scene, self.settings, self.last_frame, frame)
            finally:
                self._internal_frame_set = False
            _publish_performance(self.settings, self.world)

            self.last_frame = frame
        except Exception as exc:
            self.settings.status = str(exc)
            self.settings.is_running = False
            self.stop()

    def stop(self, restore_initial=False):
        if self.stopped:
            return
        self.running = False
        self.stopped = True
        handlers = bpy.app.handlers.frame_change_post
        if self.handler in handlers:
            handlers.remove(self.handler)
        try:
            self.world.destroy(restore_initial=restore_initial)
        finally:
            _restore_blender_rigidbody_world(self.rigidbody_world, self.previous_rigidbody_enabled)
            _unregister_controller(self)
            _clear_current_controller(self)


def start(context, settings):
    global _controller
    force_stop()
    world = PhysicsWorld()
    root = settings.model_root
    if root is None:
        from . import pmx_data_reader

        root = pmx_data_reader.find_root_object(context.active_object)
    try:
        if settings.timeline_mode:
            _initialize_timeline_world(context, settings, world, root)
        else:
            _initialize_world(context, settings, world, root)
    except Exception:
        world.destroy()
        raise
    if settings.timeline_mode:
        _controller = TimelineController(world, settings, context.scene)
        _register_controller(_controller)
        bpy.app.handlers.frame_change_post.append(_controller.handler)
    else:
        _controller = TimerController(world, settings, context.scene)
        _register_controller(_controller)
        bpy.app.timers.register(_controller.timer_callback, first_interval=0.0)


def stop():
    return force_stop()


def reset(context, settings):
    controllers = _known_controllers()
    if not controllers:
        settings.is_running = False
        return restore_last_initial_snapshot(getattr(context, "view_layer", None))

    for controller in controllers:
        controller.stop(restore_initial=True)
        controller.settings.is_running = False
    settings.is_running = False
    return True


def force_stop():
    count = 0
    for controller in _known_controllers():
        if bool(getattr(controller, "running", False)):
            count += 1
        controller.stop()
        controller.settings.is_running = False
    _controller_registry()[:] = []
    return count


def step_once(context, settings):
    controllers = _known_controllers()
    controller = controllers[0] if controllers else None
    if controller is None:
        world = PhysicsWorld()
        root = settings.model_root
        if root is None:
            from . import pmx_data_reader

            root = pmx_data_reader.find_root_object(context.active_object)
        try:
            _initialize_world(context, settings, world, root, startup_sync=False)
            world.flush_depsgraph()
            world.step(_fixed_timestep(settings) * _time_scale(settings), settings.max_substeps)
            world.flush_depsgraph()
            _publish_performance(settings, world)
        finally:
            world.destroy()
    else:
        controller.world.flush_depsgraph()
        previous_options = controller.world.apply_options.copy()
        try:
            controller.world.configure_apply_options(_full_apply_options())
            controller.world.step(_fixed_timestep(settings) * _time_scale(settings), settings.max_substeps)
        finally:
            controller.world.apply_options = previous_options
        controller.world.flush_depsgraph()
        _publish_performance(settings, controller.world)
