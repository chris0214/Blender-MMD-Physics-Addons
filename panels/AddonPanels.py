import bpy

from ..physics import physics_sync


class PMXPHYSICS_PT_main(bpy.types.Panel):
    bl_idname = "PMXPHYSICS_PT_main"
    bl_label = "PMX Physics"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "MMP"

    def draw(self, context):
        settings = context.scene.pmx_physics
        layout = self.layout

        col = layout.column(align=True)
        col.prop(settings, "model_root")
        col.operator("pmx_physics.use_active_model", icon="ARMATURE_DATA")
        col.operator("pmx_physics.scan_model", icon="VIEWZOOM")
        col.operator("pmx_physics.import_vmd", icon="ANIM")

        box = layout.box()
        box.label(text="Multi Model")
        list_col = box.column(align=True)
        if len(settings.model_roots):
            for index, item in enumerate(settings.model_roots):
                row = list_col.row(align=True)
                row.prop(item, "enabled", text="")
                row.prop(item, "root", text="")
                op = row.operator("pmx_physics.remove_model_slot", text="", icon="X")
                op.index = index
        else:
            list_col.label(text="No models in list")
        box.prop(settings, "multi_model_mode")
        row = box.row(align=True)
        row.operator("pmx_physics.add_active_model", icon="ADD")
        row.operator("pmx_physics.clear_model_slots", icon="TRASH")
        row = box.row(align=True)
        row.operator("pmx_physics.scan_all_models", icon="VIEWZOOM")

        layout.separator()

        col = layout.column(align=True)
        col.prop(settings, "dll_path")
        col.prop(settings, "fixed_timestep_preset")
        if settings.fixed_timestep_preset == "CUSTOM":
            col.prop(settings, "fixed_timestep")
        col.prop(settings, "max_substeps")
        col.prop(settings, "kinematic_smoothing")
        if settings.kinematic_smoothing:
            col.prop(settings, "kinematic_smoothing_steps")
            col.prop(settings, "kinematic_smoothing_move")
            col.prop(settings, "kinematic_smoothing_angle")
        col.prop(settings, "prewarm_steps")
        col.prop(settings, "startup_sync_steps")
        col.prop(settings, "time_scale")
        col.prop(settings, "solver_iterations")
        col.prop(settings, "gravity")
        col.prop(settings, "gravity_scale")
        col.prop(settings, "timeline_mode")

        box = layout.box()
        box.label(text="Joint Quality")
        col = box.column(align=True)
        row = col.row(align=True)
        row.prop(settings, "quality_preset")
        row.operator("pmx_physics.apply_quality_preset", text="", icon="CHECKMARK")
        col.prop(settings, "use_frame_offset")
        col.prop(settings, "joint_stop_erp")
        col.prop(settings, "joint_stop_cfm")
        col.prop(settings, "locked_joint_stop_erp")
        col.prop(settings, "locked_joint_stop_cfm")
        col.prop(settings, "locked_joint_pullback")
        col.prop(settings, "resting_body_stabilization")
        col.prop(settings, "dynamic_parent_chain_correction")

        box = layout.box()
        box.label(text="mmd_tools Compatibility")
        col = box.column(align=True)
        col.prop(settings, "zone_rule_preset")
        if settings.zone_rule_preset != "OFF":
            col.prop(settings, "zone_rule_strength")
            col.prop(settings, "show_rule_advanced")
            if settings.show_rule_advanced:
                sub = col.box()
                sub.label(text="Soft Constraint")
                sub.prop(settings, "zone_joint_stop_erp")
                sub.prop(settings, "zone_joint_stop_cfm")
                sub.prop(settings, "zone_locked_stop_erp")
                sub.prop(settings, "zone_locked_stop_cfm")
                sub.prop(settings, "zone_spring_damping")
                sub.prop(settings, "zone_linear_damping_scale")
                sub.prop(settings, "zone_angular_damping_scale")
                sub.separator()
                sub.label(text="Keywords")
                sub.prop(settings, "zone_hair_keywords")
                sub.prop(settings, "zone_skirt_keywords")
                sub.prop(settings, "zone_tail_keywords")
                sub.prop(settings, "zone_soft_keywords")
                sub.prop(settings, "zone_accessory_keywords")
                sub.prop(settings, "zone_custom_keywords")
        if settings.scan_summary:
            box.label(text=settings.scan_summary)
        if settings.scan_warnings:
            box.label(text=settings.scan_warnings, icon="ERROR")

        box = layout.box()
        box.label(text="Physics Debug")
        col = box.column(align=True)
        col.prop(settings, "debug_visual_mode")
        col.prop(settings, "debug_force_visible")
        row = col.row(align=True)
        row.operator("pmx_physics.apply_debug_visuals", icon="HIDE_OFF")
        row.operator("pmx_physics.apply_all_debug_visuals", icon="OUTLINER_COLLECTION")
        row.operator("pmx_physics.clear_debug_visuals", icon="LOOP_BACK")

        is_running = settings.is_running or physics_sync.is_active()

        row = layout.row(align=True)
        row.enabled = not is_running
        row.operator("pmx_physics.start", icon="PLAY")
        row.operator("pmx_physics.start_all", icon="PLAY")
        row = layout.row(align=True)
        row.enabled = is_running
        row.operator("pmx_physics.stop", icon="PAUSE")
        row.operator("pmx_physics.force_stop", icon="CANCEL")

        row = layout.row(align=True)
        row.operator("pmx_physics.reset", icon="FILE_REFRESH")
        row.operator("pmx_physics.step", icon="FRAME_NEXT")

        layout.separator()

        col = layout.column(align=True)
        col.prop(settings, "bake_start")
        col.prop(settings, "bake_end")
        col.prop(settings, "bake_preroll")
        col.prop(settings, "bake_restore_after")
        col.operator("pmx_physics.bake", icon="REC")
        col.operator("pmx_physics.compare_bake", icon="GRAPH")
        if settings.bake_compare_summary:
            layout.label(text=settings.bake_compare_summary)

        box = layout.box()
        row = box.row(align=True)
        row.prop(settings, "show_performance_stats", text="")
        row.label(text="Performance")
        if settings.show_performance_stats:
            col = box.column(align=True)
            col.label(text=f"Bodies {settings.perf_body_count} / Joints {settings.perf_joint_count} / Pairs {settings.perf_pair_count}")
            col.label(text=f"Steps {settings.perf_step_count} / Tick steps {settings.perf_last_tick_steps} / Smooth {settings.perf_last_smoothing_segments} (max {settings.perf_max_smoothing_segments})")
            col.label(text=f"Writes Bones {settings.perf_last_bone_writes} / Objects {settings.perf_last_object_writes}")
            col.label(text=f"Contact Pairs {settings.perf_last_contact_pairs} / Detect {settings.perf_last_contact_detect_ms:.2f} ms")
            col.label(text=f"Models Active {settings.perf_last_shared_active_models} / Changed {settings.perf_last_changed_models} / Contact {settings.perf_last_contact_models}")
            col.label(text=f"Disabled Model Pairs {settings.perf_last_disabled_model_pairs} / Writeback {settings.perf_last_writeback_models}")
            grid = box.grid_flow(columns=4, align=True)
            grid.label(text="")
            grid.label(text="Last")
            grid.label(text="Avg")
            grid.label(text="Max")
            _perf_row(grid, "Timer Tick", settings.perf_last_tick_ms, settings.perf_avg_tick_ms, settings.perf_max_tick_ms)
            _perf_row(grid, "World Step", settings.perf_last_step_ms, settings.perf_avg_step_ms, settings.perf_max_step_ms)
            _perf_row(grid, "Collect Pose", settings.perf_last_collect_ms, settings.perf_avg_collect_ms, settings.perf_max_collect_ms)
            _perf_row(grid, "Native Bullet", settings.perf_last_native_ms, settings.perf_avg_native_ms, settings.perf_max_native_ms)
            _perf_row(grid, "Readback", settings.perf_last_readback_ms, settings.perf_avg_readback_ms, settings.perf_max_readback_ms)
            _perf_row(grid, "Apply Bones", settings.perf_last_apply_ms, settings.perf_avg_apply_ms, settings.perf_max_apply_ms)
            _perf_row(grid, "Depsgraph", settings.perf_last_flush_ms, settings.perf_avg_flush_ms, settings.perf_max_flush_ms)

        box = layout.box()
        row = box.row(align=True)
        row.prop(settings, "show_interaction_debug", text="")
        row.label(text="交互调试")
        if settings.show_interaction_debug:
            col = box.column(align=True)
            col.label(text=f"类型: {settings.interaction_debug_kind or '-'}")
            col.label(text=f"作用域: {settings.interaction_debug_scope or '-'}")
            col.label(text=f"静态 {settings.interaction_debug_static_count} / 动态 {settings.interaction_debug_dynamic_count} / 冻结 {settings.interaction_debug_frozen_count}")
            col.label(text=f"静态源: {settings.interaction_debug_static_bodies or '-'}")
            col.label(text=f"动态源: {settings.interaction_debug_dynamic_bodies or '-'}")
            col.label(text=f"写回: {settings.interaction_debug_written_bones or '-'}")
            col.operator("pmx_physics.dump_interaction_debug", icon="TEXT")

        box = layout.box()
        box.label(text="Realtime Performance")
        col = box.column(align=True)
        col.prop(settings, "interaction_response_mode")
        if settings.interaction_response_mode != "OFF":
            col.prop(settings, "interaction_response_min_interval")
            col.prop(settings, "interaction_response_step_scale")
        col.prop(settings, "realtime_update_rigid_objects")
        col.prop(settings, "realtime_follow_root_motion")
        col.prop(settings, "realtime_drag_compensation")
        if settings.realtime_drag_compensation:
            sub = col.column(align=True)
            sub.prop(settings, "realtime_drag_compensate_static")
            sub.prop(settings, "realtime_drag_compensate_dynamic_bone")
            sub.prop(settings, "realtime_drag_max_segments")
            sub.prop(settings, "realtime_drag_resync")
            if settings.realtime_drag_resync:
                sub.prop(settings, "realtime_drag_resync_threshold")
                sub.prop(settings, "realtime_drag_resync_clear_velocity")
        col.prop(settings, "realtime_skip_unchanged_bones")
        if settings.realtime_skip_unchanged_bones or settings.realtime_update_rigid_objects:
            col.prop(settings, "realtime_write_location_threshold")
            col.prop(settings, "realtime_write_rotation_threshold")

        layout.separator()
        layout.label(text=settings.status)


def _perf_row(grid, label, last, avg, max_value):
    grid.label(text=label)
    grid.label(text=f"{last:.2f}")
    grid.label(text=f"{avg:.2f}")
    grid.label(text=f"{max_value:.2f}")
