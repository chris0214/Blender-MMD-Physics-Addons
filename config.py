import os

import bpy


DEFAULT_DLL_NAME = "pmx_bullet.dll"


def default_dll_path():
    return os.path.join(os.path.dirname(__file__), "native", DEFAULT_DLL_NAME)


_FIXED_TIMESTEP_VALUES = {
    "HZ_240": 1.0 / 240.0,
    "HZ_120": 1.0 / 120.0,
    "HZ_90": 1.0 / 90.0,
    "HZ_60": 1.0 / 60.0,
    "HZ_30": 1.0 / 30.0,
}


QUALITY_PRESETS = {
    "MID": {
        "fixed_timestep_preset": "HZ_120",
        "max_substeps": 4,
        "prewarm_steps": 0,
        "solver_iterations": 20,
        "use_frame_offset": True,
        "joint_stop_erp": -1.0,
        "joint_stop_cfm": -1.0,
        "locked_joint_stop_erp": 0.2,
        "locked_joint_stop_cfm": 0.0002,
        "locked_joint_pullback": True,
        "resting_body_stabilization": False,
    },
    "HARD": {
        "fixed_timestep_preset": "HZ_90",
        "max_substeps": 10,
        "prewarm_steps": 5,
        "solver_iterations": 20,
        "use_frame_offset": True,
        "joint_stop_erp": -1.0,
        "joint_stop_cfm": -1.0,
        "locked_joint_stop_erp": 0.2,
        "locked_joint_stop_cfm": 0.0002,
        "locked_joint_pullback": True,
        "resting_body_stabilization": False,
    },
    "DEFAULT": {
        "fixed_timestep_preset": "HZ_60",
        "max_substeps": 2,
        "prewarm_steps": 5,
        "solver_iterations": 20,
        "use_frame_offset": True,
        "joint_stop_erp": 0.5,
        "joint_stop_cfm": 0.1,
        "locked_joint_stop_erp": 0.5,
        "locked_joint_stop_cfm": 0.1,
        "locked_joint_pullback": True,
        "resting_body_stabilization": False,
    },
    "PREVIEW": {
        "fixed_timestep_preset": "HZ_30",
        "max_substeps": 1,
        "prewarm_steps": 0,
        "solver_iterations": 10,
        "use_frame_offset": True,
        "joint_stop_erp": 0.5,
        "joint_stop_cfm": 0.1,
        "locked_joint_stop_erp": 0.5,
        "locked_joint_stop_cfm": 0.1,
        "locked_joint_pullback": True,
        "resting_body_stabilization": True,
    },
}


ZONE_RULE_PRESETS = {
    "OFF": [],
    "MMD_COMPAT": [
        {
            "name": "hair",
            "patterns": (
                "\u9aea",
                "\u53d1",
                "\u982d\u9aea",
                "\u524d\u9aea",
                "\u6a2a\u9aea",
                "\u5f8c\u9aea",
                "hair",
            ),
            "joint_stop_erp": 0.5,
            "joint_stop_cfm": 0.08,
            "locked_joint_stop_erp": 0.5,
            "locked_joint_stop_cfm": 0.08,
            "spring_damping": 0.9,
            "linear_damping_scale": 1.15,
            "angular_damping_scale": 1.15,
        },
        {
            "name": "skirt",
            "patterns": ("\u30b9\u30ab\u30fc\u30c8", "\u88d9", "skirt"),
            "joint_stop_erp": 0.5,
            "joint_stop_cfm": 0.1,
            "locked_joint_stop_erp": 0.5,
            "locked_joint_stop_cfm": 0.1,
            "spring_damping": 0.85,
            "linear_damping_scale": 1.1,
            "angular_damping_scale": 1.1,
        },
        {
            "name": "tail",
            "patterns": ("\u5c3b\u5c3e", "\u3057\u3063\u307d", "\u5c3e", "tail"),
            "joint_stop_erp": 0.5,
            "joint_stop_cfm": 0.1,
            "locked_joint_stop_erp": 0.5,
            "locked_joint_stop_cfm": 0.1,
            "spring_damping": 0.82,
            "linear_damping_scale": 1.2,
            "angular_damping_scale": 1.2,
        },
        {
            "name": "soft-body-part",
            "patterns": ("\u80f8", "\u4e73", "breast", "bust"),
            "joint_stop_erp": 0.45,
            "joint_stop_cfm": 0.2,
            "locked_joint_stop_erp": 0.45,
            "locked_joint_stop_cfm": 0.2,
            "spring_damping": 0.75,
            "linear_damping_scale": 1.05,
            "angular_damping_scale": 1.05,
        },
        {
            "name": "accessory",
            "patterns": (
                "\u98fe",
                "\u30ea\u30dc\u30f3",
                "\u94fe",
                "\u9396",
                "belt",
                "chain",
                "ribbon",
            ),
            "joint_stop_erp": 0.5,
            "joint_stop_cfm": 0.1,
            "locked_joint_stop_erp": 0.5,
            "locked_joint_stop_cfm": 0.1,
            "spring_damping": 0.85,
            "linear_damping_scale": 1.1,
            "angular_damping_scale": 1.1,
        },
    ],
}


class PMXPhysicsModelSlot(bpy.types.PropertyGroup):
    root: bpy.props.PointerProperty(
        name="Model Root",
        description="mmd_tools root object for one PMX model in independent multi-model simulation",
        type=bpy.types.Object,
    )

    enabled: bpy.props.BoolProperty(
        name="Enabled",
        description="Include this model when starting or scanning all configured models",
        default=True,
    )


class PMXPhysicsSettings(bpy.types.PropertyGroup):
    model_root: bpy.props.PointerProperty(
        name="Model Root",
        description="mmd_tools root object for the PMX model",
        type=bpy.types.Object,
    )

    model_roots: bpy.props.CollectionProperty(
        name="Model List",
        description="PMX model roots used by independent multi-model simulation",
        type=PMXPhysicsModelSlot,
    )

    model_root_index: bpy.props.IntProperty(
        name="Model Index",
        description="Selected PMX model list item",
        default=0,
        min=0,
        options={"SKIP_SAVE"},
    )

    multi_model_mode: bpy.props.EnumProperty(
        name="Multi Model Mode",
        description="How Start All simulates multiple PMX models",
        items=(
            ("INDEPENDENT", "Independent Worlds", "Each PMX model uses its own Bullet world and never collides with other models"),
            ("ROOT_ISOLATED", "Root Isolated", "Stable per-root isolation; kept for compatibility with older saved settings"),
            ("SHARED_COLLISION", "Shadow Collision", "Babylon-MMD-style per-model Bullet worlds with kinematic shadow colliders for inter-model collision"),
            ("GLOBAL_SHARED", "Full Shared World", "Experimental: all enabled models share one dynamic Bullet world from simulation start"),
        ),
        default="SHARED_COLLISION",
    )

    dll_path: bpy.props.StringProperty(
        name="Bullet DLL",
        description="Path to the external Bullet 2.82 PMX physics DLL",
        subtype="FILE_PATH",
        default="",
    )

    fixed_timestep_preset: bpy.props.EnumProperty(
        name="Fixed Step",
        description="Physics step interval preset",
        items=(
            ("HZ_120", "120 Hz (0.00833)", "MMD-like high quality physics step"),
            ("HZ_90", "90 Hz (0.01111)", "Hard preset style physics step"),
            ("HZ_60", "60 Hz (0.01667)", "Legacy/default realtime step"),
            ("HZ_240", "240 Hz (0.00417)", "High precision test step"),
            ("HZ_30", "30 Hz (0.03333)", "Low precision preview step"),
            ("CUSTOM", "Custom", "Use the custom fixed step value"),
        ),
        default="HZ_120",
    )

    fixed_timestep: bpy.props.FloatProperty(
        name="Custom Step",
        description="Custom physics step interval in seconds",
        default=1.0 / 120.0,
        min=1.0 / 240.0,
        max=1.0 / 15.0,
        precision=5,
    )

    max_substeps: bpy.props.IntProperty(
        name="Max Substeps",
        description="Maximum physics substeps per Blender timer tick",
        default=8,
        min=1,
        max=16,
    )

    kinematic_smoothing: bpy.props.BoolProperty(
        name="Kinematic Smoothing",
        description="Split large bone-driven rigid-body jumps into smaller Bullet steps",
        default=True,
    )

    kinematic_smoothing_steps: bpy.props.IntProperty(
        name="Smoothing Steps",
        description="Maximum internal substeps used when bone-driven rigid bodies move too far in one update",
        default=12,
        min=1,
        max=64,
    )

    kinematic_smoothing_move: bpy.props.FloatProperty(
        name="Move Threshold",
        description="Model-local distance per internal smoothing step before extra substeps are inserted",
        default=0.03,
        min=0.001,
        max=1.0,
        precision=4,
    )

    kinematic_smoothing_angle: bpy.props.FloatProperty(
        name="Angle Threshold",
        description="Degrees of bone-driven rotation per internal smoothing step before extra substeps are inserted",
        default=8.0,
        min=0.1,
        max=90.0,
        precision=2,
    )

    prewarm_steps: bpy.props.IntProperty(
        name="Prewarm Steps",
        description="Number of 1/30-second settling frames to run after physics initialization",
        default=5,
        min=0,
        max=300,
    )

    startup_sync_steps: bpy.props.IntProperty(
        name="Startup Sync Steps",
        description="MMD-style transition frames from rest pose to the current animated pose when starting physics",
        default=30,
        min=0,
        max=300,
    )

    time_scale: bpy.props.FloatProperty(
        name="Time Scale",
        description="Physics simulation speed multiplier",
        default=1.0,
        min=0.0,
        max=4.0,
        precision=3,
    )

    solver_iterations: bpy.props.IntProperty(
        name="Solver Iterations",
        description="Bullet constraint solver iterations",
        default=20,
        min=1,
        max=128,
    )

    quality_preset: bpy.props.EnumProperty(
        name="Quality Preset",
        description="Apply known MMD-compatible Bullet quality parameter sets",
        items=(
            (
                "MID",
                "Mid",
                "Balanced mid-quality defaults used by this addon",
            ),
            (
                "HARD",
                "Hard",
                "Higher-quality 90 Hz preset with more substeps and short prewarm",
            ),
            (
                "DEFAULT",
                "Default",
                "Default ERP/CFM and low max-substep solver settings",
            ),
            (
                "PREVIEW",
                "Preview",
                "Low-end viewport preview preset with fewer steps and solver iterations",
            ),
        ),
        default="DEFAULT",
    )

    use_frame_offset: bpy.props.BoolProperty(
        name="Use Frame Offset",
        description="Use Bullet's Generic6Dof frame offset behavior for PMX joints",
        default=True,
    )

    joint_stop_erp: bpy.props.FloatProperty(
        name="Joint Stop ERP",
        description="Override BT_CONSTRAINT_STOP_ERP for all joint axes; negative keeps Bullet defaults",
        default=-1.0,
        min=-1.0,
        max=1.0,
        precision=4,
    )

    joint_stop_cfm: bpy.props.FloatProperty(
        name="Joint Stop CFM",
        description="Override BT_CONSTRAINT_STOP_CFM for all joint axes; negative keeps Bullet defaults",
        default=-1.0,
        min=-1.0,
        max=1.0,
        precision=6,
    )

    locked_joint_stop_erp: bpy.props.FloatProperty(
        name="Locked Joint ERP",
        description="Stop ERP used for locked translation axes; negative disables the locked-joint override",
        default=0.2,
        min=-1.0,
        max=1.0,
        precision=4,
    )

    locked_joint_stop_cfm: bpy.props.FloatProperty(
        name="Locked Joint CFM",
        description="Stop CFM used for locked translation axes; negative disables the locked-joint override",
        default=0.0002,
        min=-1.0,
        max=1.0,
        precision=6,
    )

    locked_joint_pullback: bpy.props.BoolProperty(
        name="Locked Joint Pullback",
        description="Apply a small post-solve correction to locked translation joints",
        default=True,
    )

    resting_body_stabilization: bpy.props.BoolProperty(
        name="Resting Body Stabilization",
        description="Sleep near-motionless dynamic bodies to reduce residual jitter",
        default=False,
    )

    dynamic_parent_chain_correction: bpy.props.BoolProperty(
        name="Parent Chain Mode Fix",
        description="Convert dynamic-bone children in dynamic parent chains to pure dynamic mode",
        default=True,
    )

    zone_rule_preset: bpy.props.EnumProperty(
        name="Name Rules",
        description="Apply per-zone and per-name MMD compatibility rules to matched rigid bodies and joints",
        items=(
            ("OFF", "Off", "Use only the global joint quality parameters"),
            ("MMD_COMPAT", "MMD Compatible", "Use built-in hair/skirt/tail/chest/accessory name rules"),
        ),
        default="OFF",
    )

    zone_rule_strength: bpy.props.FloatProperty(
        name="Rule Strength",
        description="Blend strength for per-zone/per-name damping and ERP/CFM overrides",
        default=0.7,
        min=0.0,
        max=1.0,
        precision=2,
    )

    show_rule_advanced: bpy.props.BoolProperty(
        name="Show Rule Parameters",
        description="Show editable per-name soft-constraint parameters",
        default=False,
    )

    zone_joint_stop_erp: bpy.props.FloatProperty(
        name="Soft Joint ERP",
        description="ERP used by matched name-rule joints; higher values pull harder toward the joint limit",
        default=0.5,
        min=0.0,
        max=1.0,
        precision=4,
    )

    zone_joint_stop_cfm: bpy.props.FloatProperty(
        name="Soft Joint CFM",
        description="CFM used by matched name-rule joints; higher values make the constraint softer",
        default=0.1,
        min=0.0,
        max=1.0,
        precision=6,
    )

    zone_locked_stop_erp: bpy.props.FloatProperty(
        name="Locked ERP",
        description="ERP used by locked translation axes on matched name-rule joints",
        default=0.5,
        min=0.0,
        max=1.0,
        precision=4,
    )

    zone_locked_stop_cfm: bpy.props.FloatProperty(
        name="Locked CFM",
        description="CFM used by locked translation axes on matched name-rule joints",
        default=0.1,
        min=0.0,
        max=1.0,
        precision=6,
    )

    zone_spring_damping: bpy.props.FloatProperty(
        name="Spring Damping",
        description="Spring damping applied to matched name-rule joints",
        default=0.85,
        min=0.0,
        max=1.0,
        precision=3,
    )

    zone_linear_damping_scale: bpy.props.FloatProperty(
        name="Linear Damping Scale",
        description="Multiplier applied to matched rigid bodies' linear damping",
        default=1.1,
        min=0.1,
        max=4.0,
        precision=3,
    )

    zone_angular_damping_scale: bpy.props.FloatProperty(
        name="Angular Damping Scale",
        description="Multiplier applied to matched rigid bodies' angular damping",
        default=1.1,
        min=0.1,
        max=4.0,
        precision=3,
    )

    zone_custom_keywords: bpy.props.StringProperty(
        name="Custom Keywords",
        description="Comma-separated extra names that should use the accessory soft-constraint rule",
        default="",
    )

    zone_hair_keywords: bpy.props.StringProperty(
        name="Hair Keywords",
        description="Comma-separated hair rigid-body or joint name keywords",
        default="髪,发,頭髪,前髪,横髪,後髪,hair",
    )

    zone_skirt_keywords: bpy.props.StringProperty(
        name="Skirt Keywords",
        description="Comma-separated skirt rigid-body or joint name keywords",
        default="スカート,裙,skirt",
    )

    zone_tail_keywords: bpy.props.StringProperty(
        name="Tail Keywords",
        description="Comma-separated tail rigid-body or joint name keywords",
        default="尻尾,しっぽ,尾,tail",
    )

    zone_soft_keywords: bpy.props.StringProperty(
        name="Soft Part Keywords",
        description="Comma-separated soft body part name keywords",
        default="胸,乳,breast,bust",
    )

    zone_accessory_keywords: bpy.props.StringProperty(
        name="Accessory Keywords",
        description="Comma-separated accessory rigid-body or joint name keywords",
        default="飾,リボン,链,鎖,belt,chain,ribbon",
    )

    show_performance_stats: bpy.props.BoolProperty(
        name="Show Performance Stats",
        description="Display realtime step timing in the panel",
        default=False,
    )

    show_interaction_debug: bpy.props.BoolProperty(
        name="显示交互调试",
        description="显示交互作用域、刚体筛选和骨骼写回诊断",
        default=False,
    )

    realtime_update_rigid_objects: bpy.props.BoolProperty(
        name="Update Rigid Objects",
        description="Write physics transforms back to mmd_tools rigid body objects during realtime preview",
        default=False,
    )

    realtime_follow_root_motion: bpy.props.BoolProperty(
        name="Follow Root Motion",
        description="Move dynamic rigid bodies with the PMX model root during realtime dragging",
        default=True,
    )

    interaction_response_mode: bpy.props.EnumProperty(
        name="Interaction Response",
        description=(
            "Reduce viewport drag latency by reacting to object or armature changes between regular timer steps. "
            "Immediate isolates physics to the bone you are dragging so unrelated cloth/hair stays stable, "
            "matching MMD-native pose-editing behavior"
        ),
        items=(
            ("OFF", "Off", "Use regular timer stepping only"),
            ("BALANCED", "Balanced", "Run one lightweight response step after interactive transforms"),
            ("IMMEDIATE", "Immediate", "Run multiple response steps for faster viewport drag following"),
        ),
        default="IMMEDIATE",
    )

    interaction_response_min_interval: bpy.props.FloatProperty(
        name="Response Min Interval",
        description="Minimum seconds between interactive response steps",
        default=0.004,
        min=0.001,
        max=0.1,
        precision=4,
    )

    interaction_response_step_scale: bpy.props.FloatProperty(
        name="Response Step Scale",
        description="Scale of the fixed timestep used by interactive response steps",
        default=0.75,
        min=0.05,
        max=1.0,
        precision=3,
    )

    realtime_drag_compensation: bpy.props.BoolProperty(
        name="Fast Drag Protection",
        description="Use adaptive extra physics segments when the model or bone-driven bodies move quickly",
        default=True,
    )

    realtime_drag_compensate_static: bpy.props.BoolProperty(
        name="Static Body Compensation",
        description="Include bone-driven static collision bodies in fast-drag adaptive smoothing",
        default=True,
    )

    realtime_drag_compensate_dynamic_bone: bpy.props.BoolProperty(
        name="Dynamic Bone Compensation",
        description="Include dynamic-bone rigid bodies in fast-drag adaptive smoothing",
        default=True,
    )

    realtime_drag_max_segments: bpy.props.IntProperty(
        name="Max Drag Segments",
        description="Maximum internal physics segments used for one fast drag update",
        default=32,
        min=1,
        max=96,
    )

    realtime_drag_resync: bpy.props.BoolProperty(
        name="Extreme Drag Resync",
        description="Temporarily align dynamic rigid bodies when a drag jump is too large for continuous simulation",
        default=True,
    )

    realtime_drag_resync_threshold: bpy.props.FloatProperty(
        name="Resync Threshold",
        description="Model-space movement in one update that triggers extreme drag resync",
        default=0.5,
        min=0.01,
        max=5.0,
        precision=3,
    )

    realtime_drag_resync_clear_velocity: bpy.props.BoolProperty(
        name="Clear Velocity On Resync",
        description="Clear dynamic rigid-body velocity after extreme drag resync for a more stable but less inertial preview",
        default=False,
    )

    realtime_skip_unchanged_bones: bpy.props.BoolProperty(
        name="Skip Unchanged Bones",
        description="Skip Blender bone writes when the physics target changed less than the realtime write threshold",
        default=True,
    )

    realtime_write_location_threshold: bpy.props.FloatProperty(
        name="Write Position Threshold",
        description="Minimum model-space movement before realtime preview writes the same bone or rigid object again",
        default=0.00015,
        min=0.0,
        max=0.01,
        precision=6,
    )

    realtime_write_rotation_threshold: bpy.props.FloatProperty(
        name="Write Rotation Threshold",
        description="Minimum rotation in degrees before realtime preview writes the same bone or rigid object again",
        default=0.02,
        min=0.0,
        max=2.0,
        precision=4,
    )

    debug_visual_mode: bpy.props.EnumProperty(
        name="Debug Color Mode",
        description="Color PMX physics objects by body mode or collision group",
        items=(
            ("BODY_MODE", "By Body Mode", "Static, dynamic, and dynamic-bone rigid bodies use different colors"),
            ("COLLISION_GROUP", "By Collision Group", "Rigid bodies use their PMX collision-group colors"),
        ),
        default="BODY_MODE",
    )

    debug_force_visible: bpy.props.BoolProperty(
        name="Force Visible",
        description="Unhide rigid bodies and joints while physics debug visuals are active",
        default=True,
    )

    gravity: bpy.props.FloatVectorProperty(
        name="Gravity",
        description="Gravity in Blender model-local coordinates",
        subtype="XYZ",
        size=3,
        default=(0.0, 0.0, -9.8),
    )

    gravity_scale: bpy.props.FloatProperty(
        name="Gravity Scale",
        description="Scale applied to the displayed gravity before sending it to Bullet",
        default=1.0,
        min=0.0,
        max=100.0,
        precision=3,
    )

    timeline_mode: bpy.props.BoolProperty(
        name="Timeline Mode",
        description="Step once per frame for deterministic baking workflows",
        default=False,
    )

    bake_start: bpy.props.IntProperty(
        name="Start",
        description="Bake start frame",
        default=1,
        min=-1048574,
        max=1048574,
    )

    bake_end: bpy.props.IntProperty(
        name="End",
        description="Bake end frame",
        default=250,
        min=-1048574,
        max=1048574,
    )

    bake_preroll: bpy.props.IntProperty(
        name="Preroll",
        description="Physics settle frames to simulate before writing baked keys",
        default=30,
        min=0,
        max=10000,
    )

    bake_restore_after: bpy.props.BoolProperty(
        name="Restore After Bake",
        description="Restore the pose captured before baking after keyframes are inserted",
        default=True,
    )

    status: bpy.props.StringProperty(
        name="Status",
        description="Last PMX Physics status message",
        default="Idle",
        options={"SKIP_SAVE"},
    )

    is_running: bpy.props.BoolProperty(
        name="Running",
        description="Whether the timer simulation is active",
        default=False,
        options={"SKIP_SAVE"},
    )

    scan_summary: bpy.props.StringProperty(
        name="Scan Summary",
        description="Last mmd_tools compatibility scan summary",
        default="",
        options={"SKIP_SAVE"},
    )

    scan_warnings: bpy.props.StringProperty(
        name="Scan Warnings",
        description="Last mmd_tools compatibility scan warnings",
        default="",
        options={"SKIP_SAVE"},
    )

    bake_compare_summary: bpy.props.StringProperty(
        name="Bake Compare",
        description="Last bake comparison result",
        default="",
        options={"SKIP_SAVE"},
    )

    perf_body_count: bpy.props.IntProperty(name="Rigid Bodies", default=0, options={"SKIP_SAVE"})
    perf_joint_count: bpy.props.IntProperty(name="Joints", default=0, options={"SKIP_SAVE"})
    perf_pair_count: bpy.props.IntProperty(name="No-Collision Pairs", default=0, options={"SKIP_SAVE"})
    perf_last_step_ms: bpy.props.FloatProperty(name="Last Step ms", default=0.0, precision=3, options={"SKIP_SAVE"})
    perf_avg_step_ms: bpy.props.FloatProperty(name="Avg Step ms", default=0.0, precision=3, options={"SKIP_SAVE"})
    perf_max_step_ms: bpy.props.FloatProperty(name="Max Step ms", default=0.0, precision=3, options={"SKIP_SAVE"})
    perf_step_count: bpy.props.IntProperty(name="Step Count", default=0, options={"SKIP_SAVE"})
    perf_last_tick_ms: bpy.props.FloatProperty(name="Last Tick ms", default=0.0, precision=3, options={"SKIP_SAVE"})
    perf_avg_tick_ms: bpy.props.FloatProperty(name="Avg Tick ms", default=0.0, precision=3, options={"SKIP_SAVE"})
    perf_max_tick_ms: bpy.props.FloatProperty(name="Max Tick ms", default=0.0, precision=3, options={"SKIP_SAVE"})
    perf_last_flush_ms: bpy.props.FloatProperty(name="Last Flush ms", default=0.0, precision=3, options={"SKIP_SAVE"})
    perf_avg_flush_ms: bpy.props.FloatProperty(name="Avg Flush ms", default=0.0, precision=3, options={"SKIP_SAVE"})
    perf_max_flush_ms: bpy.props.FloatProperty(name="Max Flush ms", default=0.0, precision=3, options={"SKIP_SAVE"})
    perf_last_collect_ms: bpy.props.FloatProperty(name="Last Collect ms", default=0.0, precision=3, options={"SKIP_SAVE"})
    perf_avg_collect_ms: bpy.props.FloatProperty(name="Avg Collect ms", default=0.0, precision=3, options={"SKIP_SAVE"})
    perf_max_collect_ms: bpy.props.FloatProperty(name="Max Collect ms", default=0.0, precision=3, options={"SKIP_SAVE"})
    perf_last_native_ms: bpy.props.FloatProperty(name="Last Native ms", default=0.0, precision=3, options={"SKIP_SAVE"})
    perf_avg_native_ms: bpy.props.FloatProperty(name="Avg Native ms", default=0.0, precision=3, options={"SKIP_SAVE"})
    perf_max_native_ms: bpy.props.FloatProperty(name="Max Native ms", default=0.0, precision=3, options={"SKIP_SAVE"})
    perf_last_readback_ms: bpy.props.FloatProperty(name="Last Readback ms", default=0.0, precision=3, options={"SKIP_SAVE"})
    perf_avg_readback_ms: bpy.props.FloatProperty(name="Avg Readback ms", default=0.0, precision=3, options={"SKIP_SAVE"})
    perf_max_readback_ms: bpy.props.FloatProperty(name="Max Readback ms", default=0.0, precision=3, options={"SKIP_SAVE"})
    perf_last_apply_ms: bpy.props.FloatProperty(name="Last Apply ms", default=0.0, precision=3, options={"SKIP_SAVE"})
    perf_avg_apply_ms: bpy.props.FloatProperty(name="Avg Apply ms", default=0.0, precision=3, options={"SKIP_SAVE"})
    perf_max_apply_ms: bpy.props.FloatProperty(name="Max Apply ms", default=0.0, precision=3, options={"SKIP_SAVE"})
    perf_last_tick_steps: bpy.props.IntProperty(name="Tick Steps", default=0, options={"SKIP_SAVE"})
    perf_last_smoothing_segments: bpy.props.IntProperty(name="Smoothing Segments", default=1, options={"SKIP_SAVE"})
    perf_max_smoothing_segments: bpy.props.IntProperty(name="Max Smoothing Segments", default=1, options={"SKIP_SAVE"})
    perf_last_bone_writes: bpy.props.IntProperty(name="Bone Writes", default=0, options={"SKIP_SAVE"})
    perf_last_object_writes: bpy.props.IntProperty(name="Object Writes", default=0, options={"SKIP_SAVE"})
    perf_last_contact_pairs: bpy.props.IntProperty(name="Contact Pairs", default=0, options={"SKIP_SAVE"})
    perf_last_shared_active_models: bpy.props.IntProperty(name="Active Models", default=0, options={"SKIP_SAVE"})
    perf_last_changed_models: bpy.props.IntProperty(name="Changed Models", default=0, options={"SKIP_SAVE"})
    perf_last_contact_models: bpy.props.IntProperty(name="Contact Models", default=0, options={"SKIP_SAVE"})
    perf_last_disabled_model_pairs: bpy.props.IntProperty(name="Disabled Model Pairs", default=0, options={"SKIP_SAVE"})
    perf_last_writeback_models: bpy.props.IntProperty(name="Writeback Models", default=0, options={"SKIP_SAVE"})
    perf_last_contact_detect_ms: bpy.props.FloatProperty(name="Contact Detect ms", default=0.0, precision=3, options={"SKIP_SAVE"})
    interaction_debug_kind: bpy.props.StringProperty(name="Interaction Kind", default="", options={"SKIP_SAVE"})
    interaction_debug_scope: bpy.props.StringProperty(name="Interaction Scope", default="", options={"SKIP_SAVE"})
    interaction_debug_static_count: bpy.props.IntProperty(name="Static Scope", default=0, options={"SKIP_SAVE"})
    interaction_debug_dynamic_count: bpy.props.IntProperty(name="Dynamic Scope", default=0, options={"SKIP_SAVE"})
    interaction_debug_frozen_count: bpy.props.IntProperty(name="Frozen Dynamic", default=0, options={"SKIP_SAVE"})
    interaction_debug_static_bodies: bpy.props.StringProperty(name="Static Bodies", default="", options={"SKIP_SAVE"})
    interaction_debug_dynamic_bodies: bpy.props.StringProperty(name="Dynamic Bodies", default="", options={"SKIP_SAVE"})
    interaction_debug_written_bones: bpy.props.StringProperty(name="Written Bones", default="", options={"SKIP_SAVE"})

    def resolved_dll_path(self):
        path = self.dll_path.strip()
        return bpy.path.abspath(path) if path else default_dll_path()

    def effective_fixed_timestep(self):
        preset = getattr(self, "fixed_timestep_preset", "HZ_120")
        if preset == "CUSTOM":
            return float(self.fixed_timestep)
        return _FIXED_TIMESTEP_VALUES.get(preset, 1.0 / 120.0)

    def effective_gravity(self):
        scale = float(getattr(self, "gravity_scale", 1.0))
        return tuple(float(value) * scale for value in self.gravity)

    def apply_quality_preset(self):
        preset = QUALITY_PRESETS.get(getattr(self, "quality_preset", "MID"))
        if preset is None:
            return False
        for key, value in preset.items():
            setattr(self, key, value)
        return True
