import copy
import math
import time

from mathutils import Matrix, Vector

from . import pmx_data_reader
from .bullet_native import BulletNative
from .physics_world import (
    PhysicsWorld,
    _new_performance,
    capture_initial_snapshot,
    remember_initial_snapshot,
    restore_snapshot,
)
from .types import MODE_DYNAMIC, MODE_DYNAMIC_BONE, MODE_STATIC, SHAPE_BOX, SHAPE_CAPSULE


class SharedPhysicsWorld:
    def __init__(self):
        self.models = []
        self.native = None
        self._view_layer = None
        self._model_worlds = []
        self._offsets = []
        self._initial_snapshots = []
        self._last_root_worlds = []
        self._last_kinematic_matrices = {}
        self._last_shared_body_matrices = {}
        self._model_active_cooldowns = []
        self._shared_unit_scale = 1.0
        self._kinematic_smoothing_enabled = True
        self._kinematic_smoothing_steps = 12
        self._kinematic_smoothing_move = 0.03
        self._kinematic_smoothing_angle = 8.0
        self._shared_activation_margin = 0.008
        self._shared_activity_cooldown = 0
        self._contact_gate_enabled = False
        self._last_contact_pairs = set()
        self._last_enabled_cross_body_pairs = set()
        self._last_applied_enabled_cross_body_pairs = None
        self._last_disabled_model_pairs = set()
        self._last_changed_models = set()
        self._last_contact_models = set()
        self._last_writeback_models = set()
        self._prewarm_steps = 0
        self.apply_options = {}
        self.performance = _new_performance()

    def configure_contact_gate(self, enabled):
        self._contact_gate_enabled = bool(enabled)

    def initialize(
        self,
        context,
        roots,
        dll_path,
        gravity,
        solver_iterations=20,
        use_frame_offset=True,
        joint_stop_erp=-1.0,
        joint_stop_cfm=-1.0,
        locked_joint_stop_erp=0.2,
        locked_joint_stop_cfm=0.0002,
        prewarm_steps=0,
        kinematic_smoothing=True,
        kinematic_smoothing_steps=12,
        kinematic_smoothing_move=0.03,
        kinematic_smoothing_angle=8.0,
        startup_sync_steps=0,
        locked_joint_pullback=True,
        resting_body_stabilization=False,
        apply_options=None,
        initial_body_matrices_by_root=None,
    ):
        roots = [root for root in roots if root is not None]
        if not roots:
            raise RuntimeError("No mmd_tools model root selected")

        self._view_layer = getattr(context, "view_layer", None)
        self._shared_unit_scale = self._shared_reference_scale(roots)
        self.apply_options = apply_options or {}
        self._flush_depsgraph()
        self._prewarm_steps = int(prewarm_steps)
        initial_body_matrices_by_root = initial_body_matrices_by_root or {}
        self._kinematic_smoothing_enabled = bool(kinematic_smoothing)
        self._kinematic_smoothing_steps = max(1, min(64, int(kinematic_smoothing_steps)))
        self._kinematic_smoothing_move = max(0.001, float(kinematic_smoothing_move))
        self._kinematic_smoothing_angle = math.radians(max(0.1, float(kinematic_smoothing_angle)))

        self.native = BulletNative(dll_path)
        self.native.create_world()
        self.native.set_cross_model_body_pair_filter_enabled(self._contact_gate_enabled)
        self.native.set_solver_iterations(solver_iterations)
        self.native.set_joint_quality(
            use_frame_offset,
            joint_stop_erp,
            joint_stop_cfm,
            locked_joint_stop_erp,
            locked_joint_stop_cfm,
        )
        self.native.set_stabilization(locked_joint_pullback, resting_body_stabilization)
        self.native.set_gravity(gravity)

        initial_all = {}
        body_offset = 0
        for root in roots:
            model = pmx_data_reader.read_model(context, root)
            self.models.append(model)
            self._initial_snapshots.append(capture_initial_snapshot(model))
            remember_initial_snapshot(self._initial_snapshots[-1])
            model_world = PhysicsWorld()
            model_world.model = model
            model_world.native = self.native
            model_world._view_layer = self._view_layer
            model_world.apply_options = self.apply_options
            model_world._capture_initial_state()
            model_world._mute_mmd_tools_physics_constraints()
            model_world._disconnect_physics_bones(context)
            model_world.model = pmx_data_reader.read_model(context, root)
            model = model_world.model
            self.models[-1] = model
            model_world.bone_driver_rigid_indices = model_world._build_bone_driver_rigid_indices()
            model_world._build_runtime_cache()
            model_world._mute_physics_bone_action_curves()
            self._model_worlds.append(model_world)
            self._offsets.append(body_offset)

            shifted_model = self._shift_model_to_shared(model, body_offset)
            source_matrices = initial_body_matrices_by_root.get(root.name)
            if source_matrices:
                target_matrices = self._shift_body_matrices_to_shared(model_world, body_offset, source_matrices)
            else:
                target_matrices = self._current_body_matrices(model_world, body_offset, include_dynamic=True)
            self.native.add_rigid_bodies(shifted_model, initial_matrices=target_matrices)
            self.native.set_body_model_ids(body_offset, len(model.rigid_bodies), len(self.models) - 1)
            self.native.add_non_collision_pairs(shifted_model)
            self.native.add_joints(shifted_model)
            initial_all.update(target_matrices)
            body_offset += len(model.rigid_bodies)

        self.native.temporal_kinematic_init(initial_all)
        self._last_shared_body_matrices = self._copy_matrices(initial_all)
        self._last_kinematic_matrices = self._copy_matrices(self._current_kinematic_matrices())
        self._last_disabled_model_pairs = set()
        self._apply_contact_gate_pairs(force=True)
        self._prewarm(self._prewarm_steps)
        self._last_shared_body_matrices = self._copy_matrices(self.native.get_body_transforms(self._body_count()))
        self._last_root_worlds = [model.root.matrix_world.copy() for model in self.models]
        self._model_active_cooldowns = [0 for _model in self.models]
        self._record_counts()
        self.flush_depsgraph()

    def destroy(self, restore_initial=False):
        if self.native is not None:
            self.native.destroy()
            self.native = None
        for model_world in self._model_worlds:
            model_world.native = None
            model_world._restore_physics_bone_action_curves()
            model_world._restore_mmd_tools_physics_constraints()
            model_world._restore_physics_bones()
        if restore_initial:
            for snapshot in self._initial_snapshots:
                restore_snapshot(snapshot, self._view_layer)
        self.models = []
        self._model_worlds = []
        self._offsets = []
        self._initial_snapshots = []
        self._last_root_worlds = []
        self._last_kinematic_matrices = {}
        self._last_shared_body_matrices = {}
        self._model_active_cooldowns = []
        self._shared_unit_scale = 1.0
        self._last_contact_pairs = set()
        self._last_enabled_cross_body_pairs = set()
        self._last_applied_enabled_cross_body_pairs = None
        self._last_disabled_model_pairs = set()
        self._last_changed_models = set()
        self._last_contact_models = set()
        self._last_writeback_models = set()
        self.performance = _new_performance()

    def configure_apply_options(self, options):
        self.apply_options = options or {}
        for model_world in self._model_worlds:
            model_world.configure_apply_options(options)

    def step(self, timestep, max_substeps, apply_results=True):
        start_time = time.perf_counter()
        self.flush_depsgraph()
        self._follow_root_motion_for_model_bodies()
        collect_start = time.perf_counter()
        kinematic = self._current_kinematic_matrices()
        gate_models = self._active_model_indices(kinematic)
        self._apply_contact_gate_pairs()
        physics_models = set(range(len(self.models)))
        inactive_models = set(range(len(self.models))) - physics_models
        if inactive_models:
            self._freeze_model_indices(inactive_models)
        step_kinematic = (
            self._filter_kinematic_matrices(kinematic, physics_models)
            if self._contact_gate_enabled
            else kinematic
        )
        collect_ms = (time.perf_counter() - collect_start) * 1000.0
        native_start = time.perf_counter()
        self._step_with_kinematic_smoothing(step_kinematic, timestep, max_substeps)
        native_ms = (time.perf_counter() - native_start) * 1000.0
        self._last_kinematic_matrices = self._copy_matrices(kinematic)

        readback_ms = 0.0
        apply_ms = 0.0
        if apply_results:
            readback_start = time.perf_counter()
            body_matrices = self.native.get_body_transforms(self._body_count())
            readback_ms = (time.perf_counter() - readback_start) * 1000.0
            apply_start = time.perf_counter()
            self._apply_body_matrices(body_matrices, physics_models)
            self._last_writeback_models = set(physics_models)
            apply_ms = (time.perf_counter() - apply_start) * 1000.0
            if self._contact_gate_enabled:
                self._refresh_kinematic_baseline(physics_models)
                self._refresh_inactive_body_cache(inactive_models, body_matrices)

        self._last_root_worlds = [model.root.matrix_world.copy() for model in self.models]
        self._decay_model_activity(gate_models)
        self.performance["last_shared_active_models"] = len(gate_models)
        self.performance["last_shared_writeback_models"] = len(physics_models)
        self.performance["last_contact_pairs"] = len(self._last_contact_pairs)
        self.performance["last_enabled_cross_body_pairs"] = len(self._last_enabled_cross_body_pairs)
        self.performance["last_disabled_model_pairs"] = len(self._last_disabled_model_pairs)
        self.performance["last_changed_models"] = len(self._last_changed_models)
        self.performance["last_contact_models"] = len(self._last_contact_models)
        self.performance["last_writeback_models"] = len(self._last_writeback_models)
        self.performance["last_contact_detect_ms"] = 0.0
        self._record_step_time(
            (time.perf_counter() - start_time) * 1000.0,
            {
                "collect_ms": collect_ms,
                "native_ms": native_ms,
                "readback_ms": readback_ms,
                "apply_ms": apply_ms,
            },
        )

    def reset_to_current_pose(self, prewarm_steps=0):
        if self.native is None:
            return
        for model_world in self._model_worlds:
            model_world._restore_unanimated_physics_bone_basis()
        self.flush_depsgraph()
        self.native.reset()
        matrices = self._current_body_matrices_all(include_dynamic=True)
        self.native.temporal_kinematic_init(matrices)
        self._last_shared_body_matrices = self._copy_matrices(matrices)
        self._last_kinematic_matrices = self._copy_matrices(self._current_kinematic_matrices())
        self._prewarm(prewarm_steps)
        self._last_shared_body_matrices = self._copy_matrices(self.native.get_body_transforms(self._body_count()))
        self.flush_depsgraph()

    def flush_depsgraph(self):
        if self._view_layer is None:
            return
        try:
            self._view_layer.update()
        except ReferenceError:
            self._view_layer = None
        except RuntimeError:
            pass

    def record_tick_time(self, elapsed_ms, steps):
        PhysicsWorld.record_tick_time(self, elapsed_ms, steps)

    def record_flush_time(self, elapsed_ms):
        PhysicsWorld.record_flush_time(self, elapsed_ms)

    def _flush_depsgraph(self):
        self.flush_depsgraph()

    def _body_count(self):
        return sum(len(model.rigid_bodies) for model in self.models)

    def _record_counts(self):
        self.performance = _new_performance()
        self.performance.update(
            {
                "body_count": self._body_count(),
                "joint_count": sum(len(model.joints) for model in self.models),
                "pair_count": sum(len(model.non_collision_pairs) for model in self.models),
            }
        )

    def _record_step_time(self, elapsed_ms, stages=None):
        PhysicsWorld._record_step_time(self, elapsed_ms, stages)

    def _prewarm(self, steps):
        steps = int(steps)
        if steps <= 0:
            return
        kinematic = self._current_kinematic_matrices()
        for _ in range(steps):
            self.native.set_kinematic_transforms(kinematic)
            self.native.step(1.0 / 30.0, 1)
        body_matrices = self.native.get_body_transforms(self._body_count())
        self._apply_body_matrices(body_matrices)

    def _current_body_matrices_all(self, include_dynamic):
        matrices = {}
        for model_world, offset in zip(self._model_worlds, self._offsets):
            matrices.update(self._current_body_matrices(model_world, offset, include_dynamic))
        return matrices

    def _current_kinematic_matrices(self):
        return self._current_body_matrices_all(include_dynamic=False)

    def _filter_kinematic_matrices(self, kinematic_matrices, model_indices):
        selected = set(model_indices)
        if not selected:
            return {}
        filtered = {}
        for model_index in selected:
            if model_index < 0 or model_index >= len(self.models):
                continue
            offset = self._offsets[model_index]
            for rigid in self.models[model_index].rigid_bodies:
                if rigid.mode != MODE_STATIC:
                    continue
                body_index = offset + rigid.index
                matrix = kinematic_matrices.get(body_index)
                if matrix is not None:
                    filtered[body_index] = matrix
        return filtered

    def _current_body_matrices(self, model_world, offset, include_dynamic):
        matrices = {}
        model = model_world.model
        for rigid in model.rigid_bodies:
            if rigid.mode == MODE_STATIC or include_dynamic:
                local_matrix = model_world._current_body_matrix(rigid)
                matrices[offset + rigid.index] = self._model_to_shared_matrix(model, local_matrix)
        return matrices

    def _shift_body_matrices_to_shared(self, model_world, offset, source_matrices):
        matrices = {}
        model = model_world.model
        for rigid in model.rigid_bodies:
            matrix = source_matrices.get(rigid.index)
            if matrix is None:
                matrix = model_world._current_body_matrix(rigid)
            matrices[offset + rigid.index] = self._model_to_shared_matrix(model, matrix)
        return matrices

    def _apply_body_matrices(self, body_matrices, model_indices=None):
        total_bone_writes = 0
        total_object_writes = 0
        selected = None if model_indices is None else set(model_indices)
        for model_index, (model_world, offset) in enumerate(zip(self._model_worlds, self._offsets)):
            if selected is not None and model_index not in selected:
                continue
            local_matrices = {}
            for rigid in model_world.model.rigid_bodies:
                body_index = offset + rigid.index
                matrix = body_matrices.get(body_index)
                if matrix is not None:
                    self._last_shared_body_matrices[body_index] = matrix.copy()
                    local_matrices[rigid.index] = self._shared_to_model_matrix(model_world.model, matrix)
            model_world._apply_body_matrices(local_matrices)
            total_bone_writes += int(model_world.performance.get("last_bone_writes", 0))
            total_object_writes += int(model_world.performance.get("last_object_writes", 0))
        self.performance["last_bone_writes"] = total_bone_writes
        self.performance["last_object_writes"] = total_object_writes

    def _preserve_dynamic_world_space_on_root_motion(self):
        # Shared-world root dragging needs a different preservation strategy than
        # single-model mode. Bodies are stored in Blender world space so one
        # model root cannot move the entire shared world, but preserving dynamic
        # body world-space across each individual root drag is still separate.
        return

    def _follow_root_motion_for_model_bodies(self):
        if self.native is None or not self.models:
            return
        if len(self._last_root_worlds) != len(self.models):
            self._last_root_worlds = [model.root.matrix_world.copy() for model in self.models]
            return

        adjusted = {}
        for model_index, model in enumerate(self.models):
            previous_root_world = self._last_root_worlds[model_index]
            current_root_world = model.root.matrix_world.copy()
            move, angle = PhysicsWorld._matrix_delta(previous_root_world, current_root_world)
            if move <= 1.0e-6 and angle <= 1.0e-5:
                continue

            previous_shared_root = self._shared_root_matrix_from_world(model, previous_root_world)
            current_shared_root = self._shared_root_matrix_from_world(model, current_root_world)
            delta = current_shared_root @ previous_shared_root.inverted_safe()
            offset = self._offsets[model_index]
            for rigid in model.rigid_bodies:
                body_index = offset + rigid.index
                matrix = self._last_shared_body_matrices.get(body_index)
                if matrix is None:
                    continue
                adjusted[body_index] = delta @ matrix

        if adjusted:
            self.native.freeze_body_transforms(adjusted)
            self._last_shared_body_matrices.update(self._copy_matrices(adjusted))
            self._last_kinematic_matrices = self._copy_matrices(self._current_kinematic_matrices())

    def _step_with_kinematic_smoothing(self, kinematic_matrices, timestep, max_substeps):
        changed_kinematic = self._changed_kinematic_matrices(kinematic_matrices)
        if not changed_kinematic:
            self.performance["last_smoothing_segments"] = 1
            self.native.step(timestep, max_substeps)
            return

        if (
            not self._kinematic_smoothing_enabled
            or self._kinematic_smoothing_steps <= 1
            or not self._last_kinematic_matrices
        ):
            self.performance["last_smoothing_segments"] = 1
            self.native.set_kinematic_transforms(changed_kinematic)
            self.native.step(timestep, max_substeps)
            return

        segments = self._kinematic_segment_count(changed_kinematic)
        self.performance["last_smoothing_segments"] = int(segments)
        self.performance["max_smoothing_segments"] = max(
            int(self.performance.get("max_smoothing_segments", 1)),
            int(segments),
        )
        if segments <= 1:
            self.native.set_kinematic_transforms(changed_kinematic)
            self.native.step(timestep, max_substeps)
            return

        sub_timestep = timestep / segments
        for segment in range(1, segments + 1):
            factor = segment / segments
            blended = self._interpolate_kinematic_matrices(changed_kinematic, factor)
            self.native.set_kinematic_transforms(blended)
            self.native.step(sub_timestep, 1)

    def _changed_kinematic_matrices(self, kinematic_matrices):
        if not self._last_kinematic_matrices:
            return dict(kinematic_matrices)

        changed = {}
        for index, matrix in kinematic_matrices.items():
            previous = self._last_kinematic_matrices.get(index)
            if previous is None:
                changed[index] = matrix
                continue
            move, angle = PhysicsWorld._matrix_delta(previous, matrix)
            if move > 1.0e-6 or angle > math.radians(0.001):
                changed[index] = matrix
        return changed

    def _active_model_indices(self, kinematic_matrices):
        if not self.models:
            return set()
        if len(self.models) == 1:
            return {0}
        if not self._contact_gate_enabled:
            return set(range(len(self.models)))
        self._last_contact_pairs = set()
        self._last_enabled_cross_body_pairs = set()
        if len(self._model_active_cooldowns) != len(self.models):
            self._model_active_cooldowns = [0 for _model in self.models]
        if not self._last_shared_body_matrices:
            return set(range(len(self.models)))

        changed = self._changed_model_indices(kinematic_matrices)
        self._last_changed_models = set(changed)
        if not changed:
            self._model_active_cooldowns = [0 for _model in self.models]
            self._last_contact_models = set()
            self._last_contact_pairs = set()
            self._last_enabled_cross_body_pairs = set()
            return set(range(len(self.models)))

        contact_probe = self._contact_probe_matrices(kinematic_matrices)
        active = self._contact_model_indices(changed, contact_probe)
        self._last_contact_models = set(active) - set(changed)
        self._model_active_cooldowns = [
            self._shared_activity_cooldown if model_index in active else 0
            for model_index in range(len(self.models))
        ]
        return set(range(len(self.models)))

    def _decay_model_activity(self, active_models):
        if self._contact_gate_enabled:
            return
        active_models = set(active_models)
        for index, cooldown in enumerate(self._model_active_cooldowns):
            if cooldown <= 0:
                continue
            self._model_active_cooldowns[index] = max(0, cooldown - 1)

    def _contact_probe_matrices(self, kinematic_matrices):
        matrices = self._current_body_matrices_all(include_dynamic=True)
        matrices.update(kinematic_matrices)
        return matrices

    def _refresh_kinematic_baseline(self, model_indices):
        self.flush_depsgraph()
        selected = set(model_indices)
        for model_index, (model_world, offset) in enumerate(zip(self._model_worlds, self._offsets)):
            if model_index not in selected:
                continue
            for rigid in model_world.model.rigid_bodies:
                if rigid.mode != MODE_STATIC:
                    continue
                self._last_kinematic_matrices[offset + rigid.index] = self._model_to_shared_matrix(
                    model_world.model,
                    model_world._current_body_matrix(rigid),
                )

    def _refresh_inactive_body_cache(self, model_indices, body_matrices):
        for model_index in model_indices:
            if model_index < 0 or model_index >= len(self.models):
                continue
            offset = self._offsets[model_index]
            for rigid in self.models[model_index].rigid_bodies:
                body_index = offset + rigid.index
                previous = self._last_shared_body_matrices.get(body_index)
                if previous is not None:
                    body_matrices[body_index] = previous.copy()

    def _nearby_model_indices(self, body_matrices):
        result = set()
        spheres_by_model = {
            model_index: self._model_body_spheres(model_index, body_matrices)
            for model_index in range(len(self.models))
        }
        bounds_by_model = {
            model_index: self._bounds_from_spheres(spheres)
            for model_index, spheres in spheres_by_model.items()
        }
        for model_index in range(len(self.models)):
            a_bounds = bounds_by_model.get(model_index)
            if a_bounds is None:
                continue
            for other_index in range(model_index + 1, len(self.models)):
                b_bounds = bounds_by_model.get(other_index)
                if (
                    self._bounds_overlap(a_bounds, b_bounds, self._shared_activation_margin)
                    and self._spheres_are_close(spheres_by_model[model_index], spheres_by_model[other_index])
                ):
                    result.add(model_index)
                    result.add(other_index)
        return result

    def _kinematic_segment_count(self, kinematic_matrices):
        segments = 1
        for index, matrix in kinematic_matrices.items():
            previous = self._last_kinematic_matrices.get(index)
            if previous is None:
                continue
            move, angle = PhysicsWorld._matrix_delta(previous, matrix)
            if move > self._kinematic_smoothing_move:
                segments = max(segments, int(move / self._kinematic_smoothing_move) + 1)
            if angle > self._kinematic_smoothing_angle:
                segments = max(segments, int(angle / self._kinematic_smoothing_angle) + 1)
        return max(1, min(self._kinematic_smoothing_steps, segments))

    def _interpolate_kinematic_matrices(self, kinematic_matrices, factor):
        blended = {}
        for index, matrix in kinematic_matrices.items():
            previous = self._last_kinematic_matrices.get(index)
            if previous is None:
                blended[index] = matrix
            else:
                blended[index] = PhysicsWorld._interpolate_matrix(previous, matrix, factor)
        return blended

    @staticmethod
    def _copy_matrices(matrices):
        return {index: matrix.copy() for index, matrix in matrices.items()}

    def _changed_model_indices(self, kinematic_matrices):
        if not self._last_kinematic_matrices:
            return set(range(len(self.models)))
        changed = set()
        for model_index, model in enumerate(self.models):
            if model_index < len(self._last_root_worlds):
                move, angle = PhysicsWorld._matrix_delta(self._last_root_worlds[model_index], model.root.matrix_world)
                if move > 1.0e-5 or angle > math.radians(0.01):
                    changed.add(model_index)
                    continue
            offset = self._offsets[model_index]
            for rigid in model.rigid_bodies:
                if rigid.mode != MODE_STATIC:
                    continue
                body_index = offset + rigid.index
                matrix = kinematic_matrices.get(body_index)
                previous = self._last_kinematic_matrices.get(body_index)
                if matrix is None or previous is None:
                    changed.add(model_index)
                    break
                move, angle = PhysicsWorld._matrix_delta(previous, matrix)
                if model_index in self._last_writeback_models:
                    move_limit = 1.0e-3
                    angle_limit = math.radians(0.25)
                else:
                    move_limit = 2.0e-4
                    angle_limit = math.radians(0.08)
                if move > move_limit or angle > angle_limit:
                    changed.add(model_index)
                    break
        return changed

    def _contact_model_indices(self, active_model_indices, body_matrices):
        result = set(active_model_indices)
        self._last_contact_pairs = set()
        self._last_enabled_cross_body_pairs = set()
        active_spheres = {
            model_index: self._model_body_spheres(model_index, body_matrices)
            for model_index in active_model_indices
        }
        active_bounds = {
            model_index: self._bounds_from_spheres(spheres)
            for model_index, spheres in active_spheres.items()
        }
        active_model_indices = [
            model_index for model_index in active_model_indices
            if active_bounds.get(model_index) is not None
        ]
        for model_index in range(len(self.models)):
            if model_index in result:
                continue
            other_spheres = self._model_body_spheres(model_index, body_matrices)
            other_bounds = self._bounds_from_spheres(other_spheres)
            if other_bounds is None:
                continue
            touched = False
            for active_index in active_model_indices:
                if (
                    self._bounds_overlap(active_bounds.get(active_index), other_bounds, self._shared_activation_margin)
                ):
                    body_pairs = self._close_body_pairs(active_spheres[active_index], other_spheres)
                    if body_pairs:
                        self._last_contact_pairs.add(tuple(sorted((active_index, model_index))))
                        self._last_enabled_cross_body_pairs.update(body_pairs)
                        touched = True
                        break
            if touched:
                result.add(model_index)
        return result

    def _apply_contact_gate_pairs(self, force=False):
        if not self._contact_gate_enabled or self.native is None:
            return
        disabled = set()
        contact = {
            tuple(sorted(pair))
            for pair in self._last_contact_pairs
        }
        for model_index in range(len(self.models)):
            for other_index in range(model_index + 1, len(self.models)):
                pair = (model_index, other_index)
                if pair not in contact:
                    disabled.add(pair)
        if not force and disabled == self._last_disabled_model_pairs:
            pass
        else:
            self.native.set_disabled_model_pairs(sorted(disabled))
            self._last_disabled_model_pairs = disabled
        if self._last_enabled_cross_body_pairs != self._last_applied_enabled_cross_body_pairs:
            self.native.set_enabled_cross_model_body_pairs(sorted(self._last_enabled_cross_body_pairs))
            self._last_applied_enabled_cross_body_pairs = set(self._last_enabled_cross_body_pairs)

    def _model_body_spheres(self, model_index, body_matrices):
        spheres = []
        offset = self._offsets[model_index]
        model = self.models[model_index]
        scale = self._model_shared_scale(model)
        for rigid in model.rigid_bodies:
            body_index = offset + rigid.index
            matrix = body_matrices.get(body_index)
            if matrix is None:
                matrix = self._last_shared_body_matrices.get(body_index)
            if matrix is None:
                continue
            group = 1 << int(rigid.collision_group_number)
            mask = 0xFFFF & ~int(rigid.no_collision_mask)
            body_index = offset + rigid.index
            spheres.extend(self._rigid_contact_spheres(rigid, matrix, scale, group, mask, body_index))
        return spheres

    def _spheres_are_close(self, a_spheres, b_spheres):
        return bool(self._close_body_pairs(a_spheres, b_spheres))

    def _close_body_pairs(self, a_spheres, b_spheres):
        if not a_spheres or not b_spheres:
            return set()
        if len(a_spheres) > len(b_spheres):
            return self._close_body_pairs(b_spheres, a_spheres)
        margin = self._shared_activation_margin
        max_radius = max(
            max(float(sphere[1]) for sphere in a_spheres),
            max(float(sphere[1]) for sphere in b_spheres),
        )
        cell_size = max(0.04, max_radius * 2.0 + margin)
        grid = {}
        for b_index, sphere in enumerate(b_spheres):
            b_center, b_radius, _b_group, _b_mask, _b_body_index = sphere
            for cell in self._sphere_cells(b_center, b_radius + margin, cell_size):
                grid.setdefault(cell, []).append((b_index, sphere))

        pairs = set()
        for a_center, a_radius, a_group, a_mask, a_body_index in a_spheres:
            seen = set()
            for cell in self._sphere_cells(a_center, a_radius + margin, cell_size):
                for b_index, sphere in grid.get(cell, ()):
                    if b_index in seen:
                        continue
                    seen.add(b_index)
                    b_center, b_radius, b_group, b_mask, b_body_index = sphere
                    if (a_group & b_mask) == 0 or (b_group & a_mask) == 0:
                        continue
                    limit = a_radius + b_radius + margin
                    if (a_center - b_center).length_squared <= limit * limit:
                        pairs.add(tuple(sorted((int(a_body_index), int(b_body_index)))))
        return pairs

    @staticmethod
    def _sphere_cells(center, radius, cell_size):
        inv = 1.0 / max(1.0e-8, float(cell_size))
        min_x = math.floor((center.x - radius) * inv)
        max_x = math.floor((center.x + radius) * inv)
        min_y = math.floor((center.y - radius) * inv)
        max_y = math.floor((center.y + radius) * inv)
        min_z = math.floor((center.z - radius) * inv)
        max_z = math.floor((center.z + radius) * inv)
        for x in range(min_x, max_x + 1):
            for y in range(min_y, max_y + 1):
                for z in range(min_z, max_z + 1):
                    yield x, y, z

    @staticmethod
    def _bounds_from_spheres(spheres):
        if not spheres:
            return None
        min_corner = spheres[0][0].copy()
        max_corner = spheres[0][0].copy()
        first_radius = spheres[0][1]
        min_corner.x -= first_radius
        min_corner.y -= first_radius
        min_corner.z -= first_radius
        max_corner.x += first_radius
        max_corner.y += first_radius
        max_corner.z += first_radius
        for center, radius, *_rest in spheres[1:]:
            min_corner.x = min(min_corner.x, center.x - radius)
            min_corner.y = min(min_corner.y, center.y - radius)
            min_corner.z = min(min_corner.z, center.z - radius)
            max_corner.x = max(max_corner.x, center.x + radius)
            max_corner.y = max(max_corner.y, center.y + radius)
            max_corner.z = max(max_corner.z, center.z + radius)
        return min_corner, max_corner

    @staticmethod
    def _bounds_overlap(a_bounds, b_bounds, margin):
        if a_bounds is None or b_bounds is None:
            return False
        a_min, a_max = a_bounds
        b_min, b_max = b_bounds
        return (
            a_min.x <= b_max.x + margin and a_max.x + margin >= b_min.x
            and a_min.y <= b_max.y + margin and a_max.y + margin >= b_min.y
            and a_min.z <= b_max.z + margin and a_max.z + margin >= b_min.z
        )

    @staticmethod
    def _rigid_radius(rigid):
        size = tuple(abs(float(value)) for value in rigid.size)
        if rigid.shape == SHAPE_BOX:
            return math.sqrt(size[0] * size[0] + size[1] * size[1] + size[2] * size[2])
        if rigid.shape == SHAPE_CAPSULE:
            return size[0] + size[1] * 0.5
        return size[0]

    @staticmethod
    def _rigid_contact_spheres(rigid, matrix, scale, group, mask, body_index):
        size = tuple(abs(float(value)) * float(scale) for value in rigid.size)
        center = matrix.to_translation()

        if rigid.shape == SHAPE_CAPSULE:
            radius = size[0]
            half_height = size[1] * 0.5
            if radius <= 0.0:
                return []
            result = [
                (matrix @ Vector((0.0, 0.0, -half_height)), radius, group, mask),
                (center, radius, group, mask),
                (matrix @ Vector((0.0, 0.0, half_height)), radius, group, mask),
            ]
            return [(center, radius, group, mask, body_index) for center, radius, group, mask in result]

        if rigid.shape == SHAPE_BOX:
            positive = [value for value in size if value > 1.0e-8]
            if not positive:
                return []
            radius = max(0.004 * float(scale), min(positive) * 0.65)
            offsets = (
                (0.0, 0.0, 0.0),
                (size[0], 0.0, 0.0),
                (-size[0], 0.0, 0.0),
                (0.0, size[1], 0.0),
                (0.0, -size[1], 0.0),
                (0.0, 0.0, size[2]),
                (0.0, 0.0, -size[2]),
            )
            return [
                (matrix @ Vector(offset), radius, group, mask, body_index)
                for offset in offsets
            ]

        radius = size[0]
        if radius <= 0.0:
            return []
        return [(center, radius, group, mask, body_index)]

    def _freeze_model_indices(self, model_indices):
        if not model_indices:
            return
        transforms = {}
        for model_index in model_indices:
            offset = self._offsets[model_index]
            for rigid in self.models[model_index].rigid_bodies:
                body_index = offset + rigid.index
                matrix = self._last_shared_body_matrices.get(body_index)
                if matrix is not None:
                    transforms[body_index] = matrix
                    self._last_shared_body_matrices[body_index] = matrix.copy()
        self.native.freeze_body_transforms(transforms)

    @staticmethod
    def _root_uniform_scale(root):
        try:
            _loc, _rot, scale = root.matrix_world.decompose()
            values = [abs(float(scale.x)), abs(float(scale.y)), abs(float(scale.z))]
            values = [value for value in values if value > 1.0e-8]
            if values:
                return sum(values) / len(values)
        except Exception:
            pass
        return 1.0

    def _shared_reference_scale(self, roots):
        for root in roots:
            scale = self._root_uniform_scale(root)
            if scale > 1.0e-8:
                return scale
        return 1.0

    def _model_shared_scale(self, model):
        return self._root_uniform_scale(model.root) / max(1.0e-8, self._shared_unit_scale)

    def _model_shared_root_matrix(self, model):
        return self._shared_root_matrix_from_world(model, model.root.matrix_world)

    def _shared_root_matrix_from_world(self, model, root_world):
        loc, rot, _scale = root_world.decompose()
        scale = self._model_shared_scale(model)
        return (
            Matrix.Translation(loc / max(1.0e-8, self._shared_unit_scale))
            @ rot.to_matrix().to_4x4()
            @ Matrix.Diagonal((scale, scale, scale, 1.0))
        )

    def _model_to_shared_matrix(self, model, matrix):
        return self._model_shared_root_matrix(model) @ matrix

    def _shared_to_model_matrix(self, model, matrix):
        return self._model_shared_root_matrix(model).inverted_safe() @ matrix

    def _shift_model_to_shared(self, model, offset):
        shifted = copy.copy(model)
        shifted.rigid_bodies = [copy.copy(rigid) for rigid in model.rigid_bodies]
        shifted.joints = [copy.copy(joint) for joint in model.joints]
        shifted.non_collision_pairs = [
            (int(rigid_a) + offset, int(rigid_b) + offset)
            for rigid_a, rigid_b in model.non_collision_pairs
        ]
        shifted.object_to_rigid_index = {
            obj: int(index) + offset
            for obj, index in model.object_to_rigid_index.items()
        }
        for rigid in shifted.rigid_bodies:
            scale = self._model_shared_scale(model)
            rigid.index += offset
            rigid.size = tuple(float(value) * scale for value in rigid.size)
            rigid.local_matrix = self._model_to_shared_matrix(model, rigid.local_matrix)
        for joint in shifted.joints:
            scale = self._model_shared_scale(model)
            joint.index += offset
            joint.rigid_a_index += offset
            joint.rigid_b_index += offset
            joint.linear_lower = tuple(float(value) * scale for value in joint.linear_lower)
            joint.linear_upper = tuple(float(value) * scale for value in joint.linear_upper)
            joint.local_matrix = self._model_to_shared_matrix(model, joint.local_matrix)
        return shifted
