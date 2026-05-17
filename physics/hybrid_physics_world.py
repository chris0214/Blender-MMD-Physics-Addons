import math
import time

from mathutils import Vector

from .physics_world import PhysicsWorld, _new_performance, capture_initial_snapshot, restore_snapshot
from .shared_physics_world import SharedPhysicsWorld
from .types import MODE_STATIC, SHAPE_BOX, SHAPE_CAPSULE


class HybridPhysicsWorld:
    """mmd_tools-style per-root multi-model simulation.

    mmd_tools builds and updates rigid bodies through the current MMD root, so
    model A never writes kinematic or solved state into model B.  Keep that
    boundary for the default contact-hybrid mode; the shared-island code below
    is retained as an experimental path, but is disabled until cross-model
    contact transfer can be made deterministic.
    """

    def __init__(self, allow_shared_island=False):
        self.models = []
        self.roots = []
        self._context = None
        self._init_args = {}
        self._worlds = []
        self._shared_world = None
        self._shared_indices = set()
        self._initial_snapshots = []
        self._view_layer = None
        self._contact_margin = 0.004
        self._shared_enter_margin = 0.08
        self._shared_exit_margin = 0.10
        self._contact_broadphase_margin = 0.12
        self._contact_exit_cooldown = 0
        self._contact_check_interval = 1
        self._contact_check_counter = 0
        self._contact_enter_frames = 1
        self._contact_enter_counter = 0
        self._shared_exit_counter = 0
        self._allow_shared_island = bool(allow_shared_island)
        self._shared_collision_delay_frames = 8
        self._shared_collision_delay_counter = 0
        self._last_contact_pairs = set()
        self._last_contact_detect_ms = 0.0
        self.apply_options = {}
        self.performance = _new_performance()

    def set_interaction_pose_scope(self, pose_bones=None):
        for world in self._active_worlds():
            world.set_interaction_pose_scope(pose_bones)
        if self._shared_world is not None:
            self._shared_world.set_interaction_pose_scope(pose_bones)

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
        self._initial_snapshots = []
        for root in self.roots:
            world = self._create_independent_world(root, startup_sync_steps=startup_sync_steps)
            self._worlds.append(world)
            self.models.append(world.model)
            self._initial_snapshots.append(getattr(world, "_initial_snapshot", None) or capture_initial_snapshot(world.model))
        self._record_counts()
        self.flush_depsgraph()

    def destroy(self, restore_initial=False):
        self._destroy_shared_world()
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
        self._shared_indices = set()
        self._initial_snapshots = []
        self._view_layer = None
        self._contact_check_counter = 0
        self._contact_enter_counter = 0
        self._last_contact_pairs = set()
        self._last_contact_detect_ms = 0.0
        self.performance = _new_performance()

    def configure_apply_options(self, options):
        self.apply_options = options or {}
        for world in self._active_worlds():
            world.configure_apply_options(options)
        if self._shared_world is not None:
            self._shared_world.configure_apply_options(options)

    def step(self, timestep, max_substeps, apply_results=True):
        start_time = time.perf_counter()

        if not self._allow_shared_island:
            if self._shared_world is not None:
                self._exit_shared_island()
            self._contact_enter_counter = 0
            self._shared_exit_counter = 0
            self._last_contact_pairs = set()
            self._last_contact_detect_ms = 0.0
            for world in self._worlds:
                if world is not None:
                    world.step(timestep, max_substeps, apply_results=apply_results)
            self._aggregate_performance((time.perf_counter() - start_time) * 1000.0)
            return

        prechecked_contacts = False
        if self._shared_world is None:
            contact_pairs, prechecked_contacts = self._maybe_detect_contact_pairs()
            if prechecked_contacts:
                if contact_pairs:
                    contact_indices = {index for pair in contact_pairs for index in pair}
                    self._contact_enter_counter += 1
                    if self._contact_enter_counter >= self._contact_enter_frames:
                        self._enter_shared_island(contact_indices)
                else:
                    self._contact_enter_counter = 0

        for index, world in enumerate(self._worlds):
            if world is not None:
                world.step(timestep, max_substeps, apply_results=apply_results)

        if self._shared_world is not None:
            self._update_shared_collision_delay()
            self._shared_world.step(timestep, max_substeps, apply_results=apply_results)

        contact_pairs, checked_contacts = self._maybe_detect_contact_pairs()
        if prechecked_contacts and self._shared_world is None:
            checked_contacts = False
        if checked_contacts:
            if contact_pairs:
                contact_indices = {index for pair in contact_pairs for index in pair}
                if self._shared_world is None:
                    self._contact_enter_counter += 1
                    if self._contact_enter_counter >= self._contact_enter_frames:
                        self._enter_shared_island(contact_indices)
                elif not contact_indices.issubset(self._shared_indices):
                    self._rebuild_shared_island(self._shared_indices | contact_indices)
                self._shared_exit_counter = self._contact_exit_cooldown
            else:
                self._contact_enter_counter = 0
                if self._shared_world is not None:
                    self._shared_exit_counter -= 1
                    if self._shared_exit_counter <= 0:
                        self._exit_shared_island()

        self._aggregate_performance((time.perf_counter() - start_time) * 1000.0)

    def reset_to_current_pose(self, prewarm_steps=0):
        for world in self._active_worlds():
            world.reset_to_current_pose(prewarm_steps=prewarm_steps)
        if self._shared_world is not None:
            self._shared_world.reset_to_current_pose(prewarm_steps=prewarm_steps)
        self.flush_depsgraph()

    def sync_kinematic_only(self):
        count = 0
        for world in self._active_worlds():
            count += world.sync_kinematic_only()
        if self._shared_world is not None:
            count += self._shared_world.sync_kinematic_only()
        return count

    def interaction_snap_dynamic_bones(self, clear_velocity=False, pose_bones=None):
        count = 0
        for world in self._active_worlds():
            count += world.interaction_snap_dynamic_bones(clear_velocity=clear_velocity, pose_bones=pose_bones)
        if self._shared_world is not None:
            count += self._shared_world.interaction_snap_dynamic_bones(clear_velocity=clear_velocity, pose_bones=pose_bones)
        return count

    def flush_depsgraph(self):
        for world in self._active_worlds():
            world.flush_depsgraph()
        if self._shared_world is not None:
            self._shared_world.flush_depsgraph()

    def record_tick_time(self, elapsed_ms, steps):
        PhysicsWorld.record_tick_time(self, elapsed_ms, steps)

    def record_flush_time(self, elapsed_ms):
        PhysicsWorld.record_flush_time(self, elapsed_ms)

    def _create_independent_world(self, root, startup_sync_steps=None, prewarm_steps=None):
        args = dict(self._init_args)
        if startup_sync_steps is not None:
            args["startup_sync_steps"] = startup_sync_steps
        if prewarm_steps is not None:
            args["prewarm_steps"] = prewarm_steps
        world = PhysicsWorld()
        try:
            world.initialize(self._context, root, **args)
        except Exception:
            world.destroy(restore_initial=False)
            raise
        return world

    def _create_shared_world(self, indices, initial_body_matrices_by_root=None):
        roots = [self.roots[index] for index in sorted(indices)]
        args = dict(self._init_args)
        args["startup_sync_steps"] = 0
        args["prewarm_steps"] = 0
        args["initial_body_matrices_by_root"] = initial_body_matrices_by_root or {}
        world = SharedPhysicsWorld()
        try:
            world.initialize(self._context, roots, **args)
        except Exception:
            world.destroy(restore_initial=False)
            raise
        return world

    def _disable_shared_cross_model_pairs(self):
        if self._shared_world is None or self._shared_world.native is None:
            return
        model_count = len(self._shared_indices)
        disabled = []
        for model_index in range(model_count):
            for other_index in range(model_index + 1, model_count):
                disabled.append((model_index, other_index))
        self._shared_world.native.set_disabled_model_pairs(disabled)

    def _enable_shared_cross_model_pairs(self):
        if self._shared_world is None or self._shared_world.native is None:
            return
        self._shared_world.native.set_disabled_model_pairs([])

    def _update_shared_collision_delay(self):
        if self._shared_world is None:
            return
        if self._shared_collision_delay_counter <= 0:
            return
        self._disable_shared_cross_model_pairs()
        self._shared_collision_delay_counter -= 1
        if self._shared_collision_delay_counter <= 0:
            self._enable_shared_cross_model_pairs()

    def _enter_shared_island(self, indices):
        indices = set(indices)
        initial_body_matrices_by_root = self._capture_independent_body_matrices(indices)
        for index in sorted(indices):
            world = self._worlds[index]
            if world is not None:
                self._detach_independent_world(world)
                self._worlds[index] = None
        self._shared_world = self._create_shared_world(indices, initial_body_matrices_by_root)
        self._shared_indices = indices
        self._shared_collision_delay_counter = max(0, int(self._shared_collision_delay_frames))
        if self._shared_collision_delay_counter > 0:
            self._disable_shared_cross_model_pairs()
        self._shared_exit_counter = self._contact_exit_cooldown
        self._contact_enter_counter = 0

    def _rebuild_shared_island(self, indices):
        self._destroy_shared_world()
        self._enter_shared_island(indices)

    def _exit_shared_island(self):
        indices = sorted(self._shared_indices)
        self._destroy_shared_world()
        for index in indices:
            self._worlds[index] = self._create_independent_world(
                self.roots[index],
                startup_sync_steps=0,
                prewarm_steps=0,
            )
            self.models[index] = self._worlds[index].model
        self.flush_depsgraph()

    def _destroy_shared_world(self):
        if self._shared_world is not None:
            self._shared_world.destroy(restore_initial=False)
            self._shared_world = None
        self._shared_indices = set()
        self._shared_exit_counter = 0
        self._shared_collision_delay_counter = 0

    @staticmethod
    def _detach_independent_world(world):
        if world.native is not None:
            world.native.destroy()
            world.native = None
        world.model = None
        world._view_layer = None
        world.bone_driver_rigid_indices = {}
        world._last_bone_targets = {}
        world._initial_pose_basis = {}
        world._initial_rigid_matrices = {}
        world._initial_snapshot = None
        world._prewarm_steps = 0
        world._last_kinematic_matrices = {}
        world.performance = _new_performance()

    def _capture_independent_body_matrices(self, indices):
        captured = {}
        for index in sorted(indices):
            world = self._worlds[index]
            if world is None or world.model is None or world.native is None:
                continue
            try:
                world._preserve_dynamic_world_space_on_root_motion()
                native_matrices = world.native.get_body_transforms(len(world.model.rigid_bodies))
            except Exception:
                native_matrices = {}

            matrices = {}
            for rigid in world.model.rigid_bodies:
                if rigid.mode == MODE_STATIC:
                    try:
                        matrices[rigid.index] = world._current_body_matrix(rigid)
                    except Exception:
                        pass
                    continue
                matrix = native_matrices.get(rigid.index)
                if matrix is None:
                    try:
                        matrix = world._current_body_matrix(rigid)
                    except Exception:
                        matrix = None
                if matrix is not None:
                    matrices[rigid.index] = matrix.copy()
            captured[world.model.root.name] = matrices
        return captured

    def _active_worlds(self):
        return [world for world in self._worlds if world is not None]

    def _model_world(self, index):
        world = self._worlds[index]
        if world is not None:
            return world
        if self._shared_world is None:
            return None
        ordered = sorted(self._shared_indices)
        try:
            shared_index = ordered.index(index)
        except ValueError:
            return None
        return self._shared_world._model_worlds[shared_index]

    def _maybe_detect_contact_pairs(self):
        self._contact_check_counter += 1
        interval = max(1, int(self._contact_check_interval))
        if self._shared_world is not None:
            interval = 1
        if self._contact_check_counter < interval:
            return set(self._last_contact_pairs), False
        self._contact_check_counter = 0

        detect_start = time.perf_counter()
        margin = self._shared_exit_margin if self._shared_world is not None else self._shared_enter_margin
        pairs = self._detect_contact_pairs(margin)
        self._last_contact_pairs = set(pairs)
        self._last_contact_detect_ms = (time.perf_counter() - detect_start) * 1000.0
        return pairs, True

    def _detect_contact_pairs(self, margin=None):
        margin = self._contact_margin if margin is None else float(margin)
        coarse_bounds = {
            index: self._model_coarse_bounds(index)
            for index in range(len(self.roots))
        }
        spheres_by_model = {}
        bounds_by_model = {}

        def spheres_for(index):
            if index not in spheres_by_model:
                spheres = self._model_contact_spheres(index)
                spheres_by_model[index] = spheres
                bounds_by_model[index] = self._bounds_from_spheres(spheres)
            return spheres_by_model[index]

        pairs = set()
        for index in range(len(self.roots)):
            a_bounds = coarse_bounds.get(index)
            if a_bounds is None:
                continue
            for other in range(index + 1, len(self.roots)):
                b_bounds = coarse_bounds.get(other)
                if not self._bounds_overlap(a_bounds, b_bounds, max(self._contact_broadphase_margin, margin)):
                    continue
                a_spheres = spheres_for(index)
                b_spheres = spheres_for(other)
                if not self._bounds_overlap(bounds_by_model.get(index), bounds_by_model.get(other), margin):
                    continue
                if self._spheres_contact(a_spheres, b_spheres, margin):
                    pairs.add((index, other))
        return pairs

    def _model_coarse_bounds(self, index):
        model_world = self._model_world(index)
        if model_world is None or model_world.model is None:
            return None
        model = model_world.model
        root_world = model.root.matrix_world
        root_scale = self._root_uniform_scale(model.root)
        min_corner = None
        max_corner = None
        for rigid in model.rigid_bodies:
            try:
                world_matrix = root_world @ rigid.local_matrix
                center = world_matrix.to_translation()
                radius = max(0.01, self._rigid_radius(rigid) * root_scale)
            except Exception:
                continue
            if min_corner is None:
                min_corner = Vector((center.x - radius, center.y - radius, center.z - radius))
                max_corner = Vector((center.x + radius, center.y + radius, center.z + radius))
                continue
            min_corner.x = min(min_corner.x, center.x - radius)
            min_corner.y = min(min_corner.y, center.y - radius)
            min_corner.z = min(min_corner.z, center.z - radius)
            max_corner.x = max(max_corner.x, center.x + radius)
            max_corner.y = max(max_corner.y, center.y + radius)
            max_corner.z = max(max_corner.z, center.z + radius)
        if min_corner is None:
            return None
        return min_corner, max_corner

    def _model_contact_spheres(self, index):
        model_world = self._model_world(index)
        if model_world is None or model_world.model is None:
            return []
        model = model_world.model
        root_world = model.root.matrix_world
        spheres = []
        for rigid in model.rigid_bodies:
            local_matrix = model_world._current_body_matrix(rigid)
            world_matrix = root_world @ local_matrix
            group = 1 << int(rigid.collision_group_number)
            mask = 0xFFFF & ~int(rigid.no_collision_mask)
            spheres.extend(self._rigid_contact_spheres(rigid, world_matrix, group, mask, int(rigid.mode), self._root_uniform_scale(model.root)))
        return spheres

    def _spheres_contact(self, a_spheres, b_spheres, margin=None):
        if not a_spheres or not b_spheres:
            return False
        if len(a_spheres) > len(b_spheres):
            return self._spheres_contact(b_spheres, a_spheres, margin)

        margin = self._contact_margin if margin is None else float(margin)
        max_radius = max(
            max(float(sphere[1]) for sphere in a_spheres),
            max(float(sphere[1]) for sphere in b_spheres),
        )
        cell_size = max(0.05, max_radius * 2.0 + margin)
        grid = {}
        for b_index, sphere in enumerate(b_spheres):
            b_center, b_radius, _b_group, _b_mask, _b_mode = sphere
            for cell in self._sphere_cells(b_center, b_radius + margin, cell_size):
                grid.setdefault(cell, []).append((b_index, sphere))

        for a_center, a_radius, a_group, a_mask, a_mode in a_spheres:
            seen = set()
            for cell in self._sphere_cells(a_center, a_radius + margin, cell_size):
                for b_index, sphere in grid.get(cell, ()):
                    if b_index in seen:
                        continue
                    seen.add(b_index)
                    b_center, b_radius, b_group, b_mask, b_mode = sphere
                    if (a_group & b_mask) == 0 or (b_group & a_mask) == 0:
                        continue
                    limit = a_radius + b_radius + margin
                    if (a_center - b_center).length_squared <= limit * limit:
                        return True
        return False

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
        first_center, first_radius, _group, _mask, _mode = spheres[0]
        min_corner = first_center.copy()
        max_corner = first_center.copy()
        min_corner.x -= first_radius
        min_corner.y -= first_radius
        min_corner.z -= first_radius
        max_corner.x += first_radius
        max_corner.y += first_radius
        max_corner.z += first_radius
        for center, radius, _group, _mask, _mode in spheres[1:]:
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
    def _rigid_contact_spheres(rigid, world_matrix, group, mask, mode, root_scale):
        size = tuple(abs(float(value)) for value in rigid.size)
        scale = max(1.0e-8, float(root_scale))
        center = world_matrix.to_translation()

        if rigid.shape == SHAPE_CAPSULE:
            radius = size[0] * scale
            half_height = size[1] * 0.5
            if radius <= 0.0:
                return []
            return [
                (world_matrix @ Vector((0.0, 0.0, -half_height)), radius, group, mask, mode),
                (center, radius, group, mask, mode),
                (world_matrix @ Vector((0.0, 0.0, half_height)), radius, group, mask, mode),
            ]

        if rigid.shape == SHAPE_BOX:
            positive = [value for value in size if value > 1.0e-8]
            if not positive:
                return []
            radius = max(0.005 * scale, min(positive) * 0.65 * scale)
            offsets = [
                Vector((0.0, 0.0, 0.0)),
                Vector((size[0], 0.0, 0.0)),
                Vector((-size[0], 0.0, 0.0)),
                Vector((0.0, size[1], 0.0)),
                Vector((0.0, -size[1], 0.0)),
                Vector((0.0, 0.0, size[2])),
                Vector((0.0, 0.0, -size[2])),
            ]
            return [(world_matrix @ offset, radius, group, mask, mode) for offset in offsets]

        radius = size[0] * scale
        if radius <= 0.0:
            return []
        return [(center, radius, group, mask, mode)]

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
        self.performance = _new_performance()
        self.performance.update(
            {
                "body_count": sum(len(model.rigid_bodies) for model in self.models),
                "joint_count": sum(len(model.joints) for model in self.models),
                "pair_count": sum(len(model.non_collision_pairs) for model in self.models),
            }
        )

    def _aggregate_performance(self, elapsed_ms):
        perfs = []
        for world in self._active_worlds():
            perfs.append(world.performance)
        if self._shared_world is not None:
            perfs.append(self._shared_world.performance)
        if not perfs:
            self._record_counts()
            return

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
        count = max(1, sum(int(perf.get("step_count", 0)) for perf in perfs))
        self.performance["step_count"] = count
        self.performance["avg_step_ms"] = sum(float(perf.get("avg_step_ms", 0.0)) for perf in perfs)
        self.performance["max_step_ms"] = max(float(perf.get("max_step_ms", 0.0)) for perf in perfs)
        for key in ("collect_ms", "native_ms", "readback_ms", "apply_ms"):
            self.performance[f"last_{key}"] = sum(float(perf.get(f"last_{key}", 0.0)) for perf in perfs)
            self.performance[f"avg_{key}"] = sum(float(perf.get(f"avg_{key}", 0.0)) for perf in perfs)
            self.performance[f"max_{key}"] = max(float(perf.get(f"max_{key}", 0.0)) for perf in perfs)
        self.performance["last_smoothing_segments"] = max(int(perf.get("last_smoothing_segments", 1)) for perf in perfs)
        self.performance["max_smoothing_segments"] = max(int(perf.get("max_smoothing_segments", 1)) for perf in perfs)
        self.performance["last_bone_writes"] = sum(int(perf.get("last_bone_writes", 0)) for perf in perfs)
        self.performance["last_object_writes"] = sum(int(perf.get("last_object_writes", 0)) for perf in perfs)
        self.performance["interaction_static_scope_count"] = sum(int(perf.get("interaction_static_scope_count", 0)) for perf in perfs)
        self.performance["interaction_dynamic_scope_count"] = sum(int(perf.get("interaction_dynamic_scope_count", 0)) for perf in perfs)
        self.performance["interaction_frozen_dynamic_count"] = sum(int(perf.get("interaction_frozen_dynamic_count", 0)) for perf in perfs)
        written = [str(perf.get("interaction_written_bones", "")) for perf in perfs if perf.get("interaction_written_bones", "")]
        self.performance["interaction_written_bones"] = " | ".join(written)[:240]
        self.performance["last_contact_pairs"] = len(self._last_contact_pairs)
        self.performance["last_contact_detect_ms"] = float(self._last_contact_detect_ms)
        self.performance["last_shared_active_models"] = len(self._shared_indices)
        self.performance["last_disabled_model_pairs"] = 1 if self._shared_collision_delay_counter > 0 else 0
        self.performance["last_contact_models"] = len({index for pair in self._last_contact_pairs for index in pair})
        self.performance["last_writeback_models"] = len(self._shared_indices)
        for key, value in previous.items():
            self.performance[key] = value
