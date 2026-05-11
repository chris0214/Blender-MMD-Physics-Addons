import bpy

from ..physics import bake, debug_visual, physics_sync, pmx_data_reader


def iface_(text):
    return bpy.app.translations.pgettext_iface(text)


def _settings(context):
    return context.scene.pmx_physics


def _set_scan_status(settings, model, diagnostics=None):
    matched_rules = getattr(model, "zone_rules", [])
    parent_chain_corrections = int(getattr(model, "parent_chain_corrections", 0))
    summary = (
        f"{len(model.rigid_bodies)} {iface_('bodies')}, "
        f"{len(model.joints)} {iface_('joints')}, "
        f"{len(model.non_collision_pairs)} {iface_('no-collision pairs')}"
    )
    if matched_rules:
        summary += f", {iface_('rules')}: {', '.join(matched_rules)}"
    if parent_chain_corrections:
        summary += f", {parent_chain_corrections} {iface_('parent-chain fixes')}"
    warnings = []
    if diagnostics:
        warnings.extend(diagnostics.get("warnings", []))
        invalid = int(diagnostics.get("invalid_joint_count", 0))
        if invalid:
            warnings.append(f"{invalid} {iface_('invalid joints')}")
    settings.scan_summary = summary
    settings.scan_warnings = "; ".join(warnings)
    settings.perf_body_count = len(model.rigid_bodies)
    settings.perf_joint_count = len(model.joints)
    settings.perf_pair_count = len(model.non_collision_pairs)
    settings.status = summary


class PMXPHYSICS_OT_use_active_model(bpy.types.Operator):
    bl_idname = "pmx_physics.use_active_model"
    bl_label = "Use Active Model"
    bl_description = "Use the active mmd_tools model root"
    bl_options = {"REGISTER"}

    def execute(self, context):
        settings = _settings(context)
        root = pmx_data_reader.find_root_object(context.active_object)
        if root is None:
            self.report({"ERROR"}, iface_("Active object is not under an mmd_tools model root"))
            return {"CANCELLED"}
        settings.model_root = root
        settings.status = f"{iface_('Selected')} {root.name}"
        return {"FINISHED"}


class PMXPHYSICS_OT_scan_model(bpy.types.Operator):
    bl_idname = "pmx_physics.scan_model"
    bl_label = "Scan Model"
    bl_description = "Read mmd_tools rigid bodies and joints from the selected model"
    bl_options = {"REGISTER"}

    def execute(self, context):
        settings = _settings(context)
        root = settings.model_root or pmx_data_reader.find_root_object(context.active_object)
        diagnostics = pmx_data_reader.diagnose_model(context, root)
        if diagnostics["errors"]:
            settings.scan_summary = ""
            settings.scan_warnings = "; ".join(diagnostics["warnings"])
            settings.status = "; ".join(diagnostics["errors"])
            self.report({"ERROR"}, settings.status)
            return {"CANCELLED"}
        try:
            model = pmx_data_reader.read_model(context, root)
        except Exception as exc:
            settings.status = str(exc)
            self.report({"ERROR"}, str(exc))
            return {"CANCELLED"}

        settings.model_root = model.root
        _set_scan_status(settings, model, diagnostics)
        self.report({"INFO"}, settings.status)
        return {"FINISHED"}


class PMXPHYSICS_OT_import_vmd(bpy.types.Operator):
    bl_idname = "pmx_physics.import_vmd"
    bl_label = "Import VMD Motion"
    bl_description = "Select the PMX model root and open mmd_tools VMD import"
    bl_options = {"REGISTER"}

    def execute(self, context):
        settings = _settings(context)
        root = settings.model_root or pmx_data_reader.find_root_object(context.active_object)
        if root is None:
            settings.status = iface_("No mmd_tools model root selected")
            self.report({"ERROR"}, settings.status)
            return {"CANCELLED"}

        try:
            import_vmd = bpy.ops.mmd_tools.import_vmd
        except AttributeError:
            settings.status = iface_("mmd_tools VMD import operator was not found")
            self.report({"ERROR"}, settings.status)
            return {"CANCELLED"}

        physics_sync.force_stop()
        settings.is_running = False
        settings.model_root = root

        view_layer = context.view_layer
        try:
            for obj in context.selected_objects:
                obj.select_set(False)
            root.select_set(True)
            view_layer.objects.active = root
        except RuntimeError as exc:
            settings.status = str(exc)
            self.report({"ERROR"}, str(exc))
            return {"CANCELLED"}

        try:
            return import_vmd("INVOKE_DEFAULT")
        except Exception as exc:
            settings.status = str(exc)
            self.report({"ERROR"}, str(exc))
            return {"CANCELLED"}


class PMXPHYSICS_OT_apply_quality_preset(bpy.types.Operator):
    bl_idname = "pmx_physics.apply_quality_preset"
    bl_label = "Apply Preset"
    bl_description = "Apply the selected PMX Physics quality preset"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        settings = _settings(context)
        if not settings.apply_quality_preset():
            settings.status = iface_("Unknown quality preset")
            self.report({"ERROR"}, settings.status)
            return {"CANCELLED"}
        settings.status = f"{iface_('Applied quality preset')}: {settings.quality_preset}"
        self.report({"INFO"}, settings.status)
        return {"FINISHED"}


class PMXPHYSICS_OT_apply_debug_visuals(bpy.types.Operator):
    bl_idname = "pmx_physics.apply_debug_visuals"
    bl_label = "Apply Debug Visuals"
    bl_description = "Show PMX rigid bodies and joints with physics debug colors"
    bl_options = {"REGISTER"}

    def execute(self, context):
        settings = _settings(context)
        root = settings.model_root or pmx_data_reader.find_root_object(context.active_object)
        if root is None:
            settings.status = iface_("No mmd_tools model root selected")
            self.report({"ERROR"}, settings.status)
            return {"CANCELLED"}
        try:
            count = debug_visual.apply_debug_visuals(
                context,
                root,
                settings.debug_visual_mode,
                settings.debug_force_visible,
            )
        except Exception as exc:
            settings.status = str(exc)
            self.report({"ERROR"}, str(exc))
            return {"CANCELLED"}
        settings.model_root = root
        settings.status = f"{iface_('Debug visualized')} {count} {iface_('objects')}"
        self.report({"INFO"}, settings.status)
        return {"FINISHED"}


class PMXPHYSICS_OT_clear_debug_visuals(bpy.types.Operator):
    bl_idname = "pmx_physics.clear_debug_visuals"
    bl_label = "Clear Debug Visuals"
    bl_description = "Restore display settings changed by PMX Physics debug visuals"
    bl_options = {"REGISTER"}

    def execute(self, context):
        settings = _settings(context)
        count = debug_visual.clear_debug_visuals()
        settings.status = f"{iface_('Cleared debug visuals')} {count} {iface_('objects')}"
        return {"FINISHED"}


class PMXPHYSICS_OT_start(bpy.types.Operator):
    bl_idname = "pmx_physics.start"
    bl_label = "Start"
    bl_description = "Start realtime PMX physics using the external Bullet DLL"
    bl_options = {"REGISTER"}

    def execute(self, context):
        settings = _settings(context)
        try:
            physics_sync.start(context, settings)
        except Exception as exc:
            settings.is_running = False
            settings.status = str(exc)
            self.report({"ERROR"}, str(exc))
            return {"CANCELLED"}
        settings.is_running = True
        settings.status = iface_("Running")
        return {"FINISHED"}


class PMXPHYSICS_OT_stop(bpy.types.Operator):
    bl_idname = "pmx_physics.stop"
    bl_label = "Stop"
    bl_description = "Stop realtime PMX physics"
    bl_options = {"REGISTER"}

    def execute(self, context):
        physics_sync.stop()
        settings = _settings(context)
        settings.is_running = False
        settings.status = iface_("Stopped")
        return {"FINISHED"}


class PMXPHYSICS_OT_force_stop(bpy.types.Operator):
    bl_idname = "pmx_physics.force_stop"
    bl_label = "Force Stop"
    bl_description = "Force stop and clean up all PMX Physics timers and handlers"
    bl_options = {"REGISTER"}

    def execute(self, context):
        count = physics_sync.force_stop()
        settings = _settings(context)
        settings.is_running = False
        settings.status = f"{iface_('Force stopped')} {count} {iface_('controller(s)')}"
        return {"FINISHED"}


class PMXPHYSICS_OT_reset(bpy.types.Operator):
    bl_idname = "pmx_physics.reset"
    bl_label = "Reset"
    bl_description = "Stop simulation and restore the pose captured when PMX physics started"
    bl_options = {"REGISTER"}

    def execute(self, context):
        settings = _settings(context)
        try:
            restored = physics_sync.reset(context, settings)
        except Exception as exc:
            settings.is_running = False
            settings.status = str(exc)
            self.report({"ERROR"}, str(exc))
            return {"CANCELLED"}
        settings.is_running = False
        settings.status = iface_("Reset") if restored else iface_("Reset: simulation is not running")
        return {"FINISHED"}


class PMXPHYSICS_OT_step(bpy.types.Operator):
    bl_idname = "pmx_physics.step"
    bl_label = "Step"
    bl_description = "Step the PMX physics world once"
    bl_options = {"REGISTER"}

    def execute(self, context):
        settings = _settings(context)
        try:
            physics_sync.step_once(context, settings)
        except Exception as exc:
            settings.status = str(exc)
            self.report({"ERROR"}, str(exc))
            return {"CANCELLED"}
        settings.status = iface_("Stepped")
        return {"FINISHED"}


class PMXPHYSICS_OT_bake(bpy.types.Operator):
    bl_idname = "pmx_physics.bake"
    bl_label = "Bake"
    bl_description = "Bake Bullet-driven physics bones to Blender keyframes"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        settings = _settings(context)
        try:
            count = bake.bake_to_keyframes(context, settings)
        except Exception as exc:
            settings.status = str(exc)
            self.report({"ERROR"}, str(exc))
            return {"CANCELLED"}
        settings.status = f"{iface_('Baked')} {count} {iface_('frames')}"
        self.report({"INFO"}, settings.status)
        return {"FINISHED"}


class PMXPHYSICS_OT_compare_bake(bpy.types.Operator):
    bl_idname = "pmx_physics.compare_bake"
    bl_label = "Compare Bake"
    bl_description = "Compare current baked keyframes with a fresh Bullet simulation over the bake range"
    bl_options = {"REGISTER"}

    def execute(self, context):
        settings = _settings(context)
        try:
            result = bake.compare_baked_motion(context, settings)
        except Exception as exc:
            settings.bake_compare_summary = str(exc)
            settings.status = str(exc)
            self.report({"ERROR"}, str(exc))
            return {"CANCELLED"}

        settings.bake_compare_summary = (
            f"{iface_('Samples')}: {result['samples']}, "
            f"{iface_('Avg Loc')}: {result['avg_loc']:.5f}, "
            f"{iface_('Max Loc')}: {result['max_loc']:.5f}, "
            f"{iface_('Avg Rot')}: {result['avg_angle_deg']:.3f} deg, "
            f"{iface_('Max Rot')}: {result['max_angle_deg']:.3f} deg"
        )
        settings.status = settings.bake_compare_summary
        self.report({"INFO"}, settings.status)
        return {"FINISHED"}
