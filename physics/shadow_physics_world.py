import copy
import time

from mathutils import Matrix

from .physics_world import PhysicsWorld, _new_performance, capture_initial_snapshot, restore_snapshot
from .types import MODE_DYNAMIC, MODE_DYNAMIC_BONE, MODE_STATIC


class ShadowPhysicsWorld:
    """Babylon-MMD-style multi-model collision using kinematic shadows.

    Each PMX model keeps its own dynamic Bullet world.  Other models are added
    to that world as kinematic collider copies, so cross-model collision can be
    seen without putting every dynamic body in one shared solver island.
    """

    def __init__(self):
        self.models = []
        self.roots = []
        self._context = None
        self._init_args = {}
        self._worlds = []
        self._shadow_maps = []
        self._initial_snapshots = []
        self._view_layer = None
        self.apply_options = {}
        self.performance = _new_performance()

    def set_interaction_pose_scope(self, pose_bones=None):
        for world in self._active_worlds():
            world.set_interaction_pose_scope(pose_bones)

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
    ):
        self.roots = [root for root in roots if root is not None]
        if not self.roots:
            raise RuntimeError("No mmd_tools model root selected")

        self._context = context
        self._view_layer = getattr(context, "view_layer", None)
        self.apply_options = apply_options or {}
        self._init_args = {
            "dll_path": dll_path,
            "gravity": gravity,
            "solver_iterations": solver_iterations,
            "use_frame_offset": use_frame_offset,
            "joint_stop_erp": joint_stop_erp,
            "joint_stop_cfm": joint_stop_cfm,
            "locked_joint_stop_erp": locked_joint_stop_erp,
            "locked_joint_stop_cfm": locked_joint_stop_cfm,
            "prewarm_steps": prewarm_steps,
            "kinematic_smoothing": kinematic_smoothing,
            "kinematic_smoothing_steps": kinematic_smoothing_steps,
            "kinematic_smoothing_move": kinematic_smoothing_move,
            "kinematic_smoothing_angle": kinematic_smoothing_angle,
            "startup_sync_steps": startup_sync_steps,
            "locked_joint_pullback": locked_joint_pullback,
            "resting_body_stabilization": resting_body_stabilization,
            "apply_options": self.apply_options,
        }

        self.models = []
        self._worlds = []
        self._shadow_maps = []
        self._initial_snapshots = []
        for model_index, root in enumerate(self.roots):
            world = PhysicsWorld()
            try:
                world.initialize(context, root, **self._init_args)
                world.native.set_body_model_ids(0, len(world.model.rigid_bodies), model_index)
            except Exception:
                world.destroy(restore_initial=False)
                raise
            self._worlds.append(world)
            self.models.append(world.model)
            self._initial_snapshots.append(getattr(world, "_initial_snapshot", None) or capture_initial_snapshot(world.model))

        self._install_shadow_bodies()
        self._record_counts()
        self.flush_depsgraph()

    def destroy(self, restore_initial=False):
        for index, world in enumerate(self._worlds):
            if world is not None:
                world.destroy(restore_initial=False)
                self._worlds[index] = None
        if restore_initial:
            for snapshot in self._initial_snapshots:
                restore_snapshot(snapshot, self._view_layer)
        self.models = []
        self.roots = []
        self._context = None
        self._init_args = {}
        self._worlds = []
        self._shadow_maps = []
        self._initial_snapshots = []
        self._view_layer = None
        self.performance = _new_performance()

    def configure_apply_options(self, options):
        self.apply_options = options or {}
        for world in self._active_worlds():
            world.configure_apply_options(options)

    def step(self, timestep, max_substeps, apply_results=True):
        start_time = time.perf_counter()
        for world in self._active_worlds():
            world.flush_depsgraph()

        shadow_start = time.perf_counter()
        self._update_shadow_bodies()
        shadow_ms = (time.perf_counter() - shadow_start) * 1000.0

        for world in self._active_worlds():
            world.step(timestep, max_substeps, apply_results=apply_results)

        self._aggregate_performance((time.perf_counter() - start_time) * 1000.0, shadow_ms)

    def reset_to_current_pose(self, prewarm_steps=0):
        for world in self._active_worlds():
            world.reset_to_current_pose(prewarm_steps=prewarm_steps)
        self._update_shadow_bodies()
        self.flush_depsgraph()

    def sync_kinematic_only(self):
        count = 0
        for world in self._active_worlds():
            count += world.sync_kinematic_only()
        self._update_shadow_bodies()
        return count

    def interaction_snap_dynamic_bones(self, clear_velocity=False, pose_bones=None):
        count = 0
        for world in self._active_worlds():
            count += world.interaction_snap_dynamic_bones(clear_velocity=clear_velocity, pose_bones=pose_bones)
        self._update_shadow_bodies()
        return count

    def flush_depsgraph(self):
        for world in self._active_worlds():
            world.flush_depsgraph()

    def record_tick_time(self, elapsed_ms, steps):
        PhysicsWorld.record_tick_time(self, elapsed_ms, steps)

    def record_flush_time(self, elapsed_ms):
        PhysicsWorld.record_flush_time(self, elapsed_ms)

    def _active_worlds(self):
        return [world for world in self._worlds if world is not None]

    def _install_shadow_bodies(self):
        self._shadow_maps = []
        for target_index, target_world in enumerate(self._worlds):
            target_map = {}
            self._shadow_maps.append(target_map)
            if target_world is None or target_world.native is None:
                continue

            next_index = len(target_world.model.rigid_bodies)
            for source_index, source_world in enumerate(self._worlds):
                if source_index == target_index or source_world is None:
                    continue
                start_index = next_index
                shadow_model, initial_matrices, body_map = self._make_shadow_model(
                    source_world,
                    target_world,
                    start_index,
                )
                if not shadow_model.rigid_bodies:
                    continue
                target_world.native.add_rigid_bodies(shadow_model, initial_matrices=initial_matrices)
                target_world.native.set_body_model_ids(start_index, len(shadow_model.rigid_bodies), source_index)
                target_map.update({(source_index, source_body): shadow_body for source_body, shadow_body in body_map.items()})
                next_index += len(shadow_model.rigid_bodies)

    def _make_shadow_model(self, source_world, target_world, start_index):
        source_model = source_world.model
        target_model = target_world.model
        source_matrices = self._body_matrices_for_world(source_world)
        shadow_bodies = []
        initial_matrices = {}
        body_map = {}
        scale = self._root_uniform_scale(source_model.root) / max(1.0e-8, self._root_uniform_scale(target_model.root))
        for source_rigid in source_model.rigid_bodies:
            shadow = copy.copy(source_rigid)
            shadow.index = start_index + len(shadow_bodies)
            shadow.name = f"{source_rigid.name} shadow"
            shadow.mode = MODE_STATIC
            shadow.mass = 0.0
            shadow.size = tuple(float(value) * scale for value in source_rigid.size)
            matrix = source_matrices.get(source_rigid.index)
            if matrix is None:
                matrix = source_world._current_body_matrix(source_rigid)
            target_matrix = self._source_to_target_matrix(source_model, target_model, matrix)
            shadow.local_matrix = target_matrix
            initial_matrices[shadow.index] = target_matrix
            body_map[source_rigid.index] = shadow.index
            shadow_bodies.append(shadow)

        shadow_model = copy.copy(source_model)
        shadow_model.root = target_model.root
        shadow_model.armature = None
        shadow_model.rigid_bodies = shadow_bodies
        shadow_model.joints = []
        shadow_model.non_collision_pairs = []
        shadow_model.object_to_rigid_index = {}
        return shadow_model, initial_matrices, body_map

    def _update_shadow_bodies(self):
        if not self._shadow_maps:
            return
        source_matrices = [
            self._body_matrices_for_world(world) if world is not None else {}
            for world in self._worlds
        ]
        for target_index, target_world in enumerate(self._worlds):
            if target_world is None or target_world.native is None:
                continue
            transforms = {}
            target_model = target_world.model
            for (source_index, source_body), shadow_body in self._shadow_maps[target_index].items():
                source_world = self._worlds[source_index]
                if source_world is None:
                    continue
                matrix = source_matrices[source_index].get(source_body)
                if matrix is None:
                    continue
                transforms[shadow_body] = self._source_to_target_matrix(source_world.model, target_model, matrix)
            if transforms:
                target_world.native.freeze_body_transforms(transforms)

    def _body_matrices_for_world(self, world):
        if world is None or world.model is None:
            return {}
        matrices = {}
        if world.native is not None:
            try:
                matrices.update(world.native.get_body_transforms(len(world.model.rigid_bodies)))
            except Exception:
                matrices = {}
        for rigid in world.model.rigid_bodies:
            if rigid.mode == MODE_STATIC or rigid.index not in matrices:
                try:
                    matrices[rigid.index] = world._current_body_matrix(rigid)
                except Exception:
                    pass
        return {index: matrix.copy() for index, matrix in matrices.items()}

    @staticmethod
    def _source_to_target_matrix(source_model, target_model, matrix):
        source_root = source_model.root.matrix_world
        target_root_inverse = target_model.root.matrix_world.inverted_safe()
        return target_root_inverse @ source_root @ matrix

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

    def _record_counts(self):
        own_body_count = sum(len(model.rigid_bodies) for model in self.models)
        shadow_body_count = sum(len(mapping) for mapping in self._shadow_maps)
        self.performance = _new_performance()
        self.performance.update(
            {
                "body_count": own_body_count,
                "joint_count": sum(len(model.joints) for model in self.models),
                "pair_count": sum(len(model.non_collision_pairs) for model in self.models),
                "shadow_body_count": shadow_body_count,
            }
        )

    def _aggregate_performance(self, elapsed_ms, shadow_ms):
        perfs = [world.performance for world in self._active_worlds()]
        previous = {
            "tick_count": self.performance.get("tick_count", 0),
            "avg_tick_ms": self.performance.get("avg_tick_ms", 0.0),
            "max_tick_ms": self.performance.get("max_tick_ms", 0.0),
            "flush_count": self.performance.get("flush_count", 0),
            "avg_flush_ms": self.performance.get("avg_flush_ms", 0.0),
            "max_flush_ms": self.performance.get("max_flush_ms", 0.0),
            "last_flush_ms": self.performance.get("last_flush_ms", 0.0),
        }
        self._record_counts()
        self.performance["last_step_ms"] = float(elapsed_ms)
        self.performance["step_count"] = max(1, sum(int(perf.get("step_count", 0)) for perf in perfs))
        self.performance["avg_step_ms"] = sum(float(perf.get("avg_step_ms", 0.0)) for perf in perfs)
        self.performance["max_step_ms"] = max((float(perf.get("max_step_ms", 0.0)) for perf in perfs), default=0.0)
        for key in ("collect_ms", "native_ms", "readback_ms", "apply_ms"):
            self.performance[f"last_{key}"] = sum(float(perf.get(f"last_{key}", 0.0)) for perf in perfs)
            self.performance[f"avg_{key}"] = sum(float(perf.get(f"avg_{key}", 0.0)) for perf in perfs)
            self.performance[f"max_{key}"] = max((float(perf.get(f"max_{key}", 0.0)) for perf in perfs), default=0.0)
        self.performance["last_collect_ms"] += float(shadow_ms)
        self.performance["avg_collect_ms"] += float(shadow_ms)
        self.performance["max_collect_ms"] = max(float(self.performance.get("max_collect_ms", 0.0)), float(shadow_ms))
        self.performance["last_smoothing_segments"] = max((int(perf.get("last_smoothing_segments", 1)) for perf in perfs), default=1)
        self.performance["max_smoothing_segments"] = max((int(perf.get("max_smoothing_segments", 1)) for perf in perfs), default=1)
        self.performance["last_bone_writes"] = sum(int(perf.get("last_bone_writes", 0)) for perf in perfs)
        self.performance["last_object_writes"] = sum(int(perf.get("last_object_writes", 0)) for perf in perfs)
        self.performance["interaction_static_scope_count"] = sum(int(perf.get("interaction_static_scope_count", 0)) for perf in perfs)
        self.performance["interaction_dynamic_scope_count"] = sum(int(perf.get("interaction_dynamic_scope_count", 0)) for perf in perfs)
        self.performance["interaction_frozen_dynamic_count"] = sum(int(perf.get("interaction_frozen_dynamic_count", 0)) for perf in perfs)
        written = [str(perf.get("interaction_written_bones", "")) for perf in perfs if perf.get("interaction_written_bones", "")]
        self.performance["interaction_written_bones"] = " | ".join(written)[:240]
        self.performance["last_contact_pairs"] = len(self.models) * max(0, len(self.models) - 1) // 2
        self.performance["last_shared_active_models"] = len(self.models)
        self.performance["last_changed_models"] = 0
        self.performance["last_contact_models"] = 0
        self.performance["last_disabled_model_pairs"] = 0
        self.performance["last_writeback_models"] = len(self.models)
        self.performance["last_contact_detect_ms"] = float(shadow_ms)
        for key, value in previous.items():
            self.performance[key] = value
