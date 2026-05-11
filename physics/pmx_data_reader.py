from mathutils import Vector

from .. import config as addon_config
from .types import (
    MODE_DYNAMIC,
    MODE_DYNAMIC_BONE,
    MODE_STATIC,
    SHAPE_BOX,
    SHAPE_CAPSULE,
    SHAPE_SPHERE,
    JointData,
    PmxModelData,
    RigidBodyData,
)


SHAPE_MAP = {
    "SPHERE": SHAPE_SPHERE,
    "BOX": SHAPE_BOX,
    "CAPSULE": SHAPE_CAPSULE,
}


def _mmd_type(obj):
    if obj is None:
        return ""
    try:
        return obj.mmd_type
    except Exception:
        return ""


def find_root_object(obj):
    while obj is not None and _mmd_type(obj) != "ROOT":
        obj = obj.parent
    return obj


def _iter_children(obj):
    if obj is None:
        return
    for child in obj.children:
        yield child
        yield from _iter_children(child)


def _find_armature(root):
    if root is None:
        return None
    for obj in root.children:
        if obj.type == "ARMATURE":
            return obj
    return None


def _iter_by_mmd_type(root, mmd_type):
    for obj in _iter_children(root):
        if _mmd_type(obj) == mmd_type:
            yield obj


def _root_local_matrix(root, obj):
    if root is None:
        return obj.matrix_world.copy()
    return root.matrix_world.inverted_safe() @ obj.matrix_world


def _vec3(value, fallback=(0.0, 0.0, 0.0)):
    try:
        return (float(value[0]), float(value[1]), float(value[2]))
    except Exception:
        return fallback


def _bool_mask_to_bits(mask):
    bits = 0
    for index, blocked in enumerate(mask):
        if blocked:
            bits |= 1 << index
    return bits


def _context_settings(context_or_object):
    scene = getattr(context_or_object, "scene", None)
    if scene is None:
        return None
    return getattr(scene, "pmx_physics", None)


def _normalize_name(value):
    return str(value or "").strip().casefold()


def _collect_name_hints(rigid, obj):
    hints = [rigid.name, obj.name, rigid.bone_name]
    try:
        mmd_rigid = getattr(obj, "mmd_rigid", None)
        if mmd_rigid is not None:
            hints.append(getattr(mmd_rigid, "name_j", ""))
            hints.append(getattr(mmd_rigid, "name_e", ""))
    except Exception:
        pass
    return [hint for hint in hints if hint]


def _matches_patterns(hints, patterns):
    normalized_hints = [_normalize_name(hint) for hint in hints]
    for pattern in patterns:
        needle = _normalize_name(pattern)
        if not needle:
            continue
        if any(needle in hint for hint in normalized_hints):
            return True
    return False


def _blend_value(base, override, strength):
    if override is None:
        return base
    if base is None or base < 0.0:
        return override
    if strength <= 0.0:
        return base
    if strength >= 1.0:
        return override
    return base + (override - base) * strength


def _apply_zone_rules_to_rigid(rigid, rule, strength):
    rigid.linear_damping = max(0.0, min(1.0, rigid.linear_damping * (1.0 + (rule.get("linear_damping_scale", 1.0) - 1.0) * strength)))
    rigid.angular_damping = max(0.0, min(1.0, rigid.angular_damping * (1.0 + (rule.get("angular_damping_scale", 1.0) - 1.0) * strength)))


def _apply_zone_rules_to_joint(joint, rule, strength):
    joint.joint_stop_erp = _blend_value(joint.joint_stop_erp, rule.get("joint_stop_erp"), strength)
    joint.joint_stop_cfm = _blend_value(joint.joint_stop_cfm, rule.get("joint_stop_cfm"), strength)
    joint.locked_joint_stop_erp = _blend_value(joint.locked_joint_stop_erp, rule.get("locked_joint_stop_erp"), strength)
    joint.locked_joint_stop_cfm = _blend_value(joint.locked_joint_stop_cfm, rule.get("locked_joint_stop_cfm"), strength)
    damping = float(rule.get("spring_damping", 1.0))
    if strength >= 1.0:
        joint.spring_linear_damping = (damping, damping, damping)
        joint.spring_angular_damping = (damping, damping, damping)
    elif strength > 0.0:
        joint.spring_linear_damping = tuple(
            max(0.0, min(1.0, value + (damping - value) * strength)) for value in joint.spring_linear_damping
        )
        joint.spring_angular_damping = tuple(
            max(0.0, min(1.0, value + (damping - value) * strength)) for value in joint.spring_angular_damping
        )


def _apply_zone_rules(model, settings):
    if settings is None or getattr(settings, "zone_rule_preset", "OFF") == "OFF":
        return []

    preset_name = getattr(settings, "zone_rule_preset", "OFF")
    rules = addon_config.ZONE_RULE_PRESETS.get(preset_name, [])
    if not rules:
        return []

    strength = float(getattr(settings, "zone_rule_strength", 1.0))
    keyword_properties = {
        "hair": "zone_hair_keywords",
        "skirt": "zone_skirt_keywords",
        "tail": "zone_tail_keywords",
        "soft-body-part": "zone_soft_keywords",
        "accessory": "zone_accessory_keywords",
    }
    extra_keywords = [item.strip() for item in str(getattr(settings, "zone_custom_keywords", "")).split(",") if item.strip()]
    matched_rules = []

    for rigid in model.rigid_bodies:
        hints = _collect_name_hints(rigid, rigid.obj)
        for rule in rules:
            patterns = tuple(
                item.strip()
                for item in str(getattr(settings, keyword_properties.get(rule.get("name"), ""), "")).split(",")
                if item.strip()
            ) or tuple(rule.get("patterns", ()))
            if extra_keywords:
                patterns = patterns + tuple(extra_keywords)
            if _matches_patterns(hints, patterns):
                editable_rule = dict(rule)
                editable_rule.update(_editable_rule_values(settings))
                _apply_zone_rules_to_rigid(rigid, editable_rule, strength)
                matched_rules.append(rule["name"])
                break

    rigid_by_index = {rigid.index: rigid for rigid in model.rigid_bodies}
    for joint in model.joints:
        rigid_a = rigid_by_index.get(joint.rigid_a_index)
        rigid_b = rigid_by_index.get(joint.rigid_b_index)
        hints = [joint.name, joint.obj.name]
        if rigid_a is not None:
            hints.extend(_collect_name_hints(rigid_a, rigid_a.obj))
        if rigid_b is not None:
            hints.extend(_collect_name_hints(rigid_b, rigid_b.obj))
        for rule in rules:
            patterns = tuple(
                item.strip()
                for item in str(getattr(settings, keyword_properties.get(rule.get("name"), ""), "")).split(",")
                if item.strip()
            ) or tuple(rule.get("patterns", ()))
            if extra_keywords:
                patterns = patterns + tuple(extra_keywords)
            if _matches_patterns(hints, patterns):
                editable_rule = dict(rule)
                editable_rule.update(_editable_rule_values(settings))
                _apply_zone_rules_to_joint(joint, editable_rule, strength)
                joint.zone_rule = rule["name"]
                matched_rules.append(rule["name"])
                break

    return sorted(set(matched_rules))


def _editable_rule_values(settings):
    return {
        "joint_stop_erp": float(getattr(settings, "zone_joint_stop_erp", 0.5)),
        "joint_stop_cfm": float(getattr(settings, "zone_joint_stop_cfm", 0.1)),
        "locked_joint_stop_erp": float(getattr(settings, "zone_locked_stop_erp", 0.5)),
        "locked_joint_stop_cfm": float(getattr(settings, "zone_locked_stop_cfm", 0.1)),
        "spring_damping": float(getattr(settings, "zone_spring_damping", 0.85)),
        "linear_damping_scale": float(getattr(settings, "zone_linear_damping_scale", 1.1)),
        "angular_damping_scale": float(getattr(settings, "zone_angular_damping_scale", 1.1)),
    }


def _rigid_range(obj):
    try:
        a = Vector(obj.bound_box[0])
        b = Vector(obj.bound_box[6])
        return (a - b).length
    except Exception:
        return 0.0


def _bone_model_matrix(root, armature, bone_name):
    if root is None or armature is None or not bone_name:
        return None
    pose_bone = armature.pose.bones.get(bone_name)
    if pose_bone is None:
        return None
    return root.matrix_world.inverted_safe() @ armature.matrix_world @ pose_bone.matrix


def _bone_rest_model_matrix(root, armature, bone_name):
    if root is None or armature is None or not bone_name:
        return None
    pose_bone = armature.pose.bones.get(bone_name)
    if pose_bone is None:
        return None
    return root.matrix_world.inverted_safe() @ armature.matrix_world @ pose_bone.bone.matrix_local


def _bone_pose_delta_model_matrix(root, armature, bone_name):
    current = _bone_model_matrix(root, armature, bone_name)
    rest = _bone_rest_model_matrix(root, armature, bone_name)
    if current is None or rest is None:
        return None
    return current @ rest.inverted_safe()


def _read_rigid(root, armature, obj, index):
    mmd_rigid = obj.mmd_rigid
    rb = obj.rigid_body
    mode = int(getattr(mmd_rigid, "type", str(MODE_DYNAMIC)))
    local_matrix = _root_local_matrix(root, obj)

    bone_name = str(getattr(mmd_rigid, "bone", "") or "")
    bone_matrix = _bone_rest_model_matrix(root, armature, bone_name)
    bone_offset = None
    if bone_matrix is not None:
        bone_offset = bone_matrix.inverted_safe() @ local_matrix

    return RigidBodyData(
        index=index,
        name=str(getattr(mmd_rigid, "name_j", "") or obj.name),
        obj=obj,
        bone_name=bone_name,
        shape=SHAPE_MAP.get(str(getattr(mmd_rigid, "shape", "SPHERE")), SHAPE_SPHERE),
        mode=mode,
        size=_vec3(getattr(mmd_rigid, "size", (0.0, 0.0, 0.0))),
        mass=float(getattr(rb, "mass", 0.0 if mode == MODE_STATIC else 1.0)) if rb else 0.0,
        friction=float(getattr(rb, "friction", 0.5)) if rb else 0.5,
        restitution=float(getattr(rb, "restitution", 0.0)) if rb else 0.0,
        linear_damping=float(getattr(rb, "linear_damping", 0.04)) if rb else 0.04,
        angular_damping=float(getattr(rb, "angular_damping", 0.1)) if rb else 0.1,
        collision_group_number=int(getattr(mmd_rigid, "collision_group_number", 0)),
        no_collision_mask=_bool_mask_to_bits(getattr(mmd_rigid, "collision_group_mask", [False] * 16)),
        local_matrix=local_matrix,
        bone_offset_matrix=bone_offset,
    )


def _joint_model_matrix(root, armature, obj, rbc, rigid_by_object):
    return _root_local_matrix(root, obj)


def _read_joint(root, armature, obj, index, object_to_rigid_index, rigid_by_object):
    rbc = obj.rigid_body_constraint
    if rbc is None or rbc.object1 not in object_to_rigid_index or rbc.object2 not in object_to_rigid_index:
        return None

    mmd_joint = obj.mmd_joint
    return JointData(
        index=index,
        name=str(getattr(mmd_joint, "name_j", "") or obj.name),
        obj=obj,
        rigid_a_index=object_to_rigid_index[rbc.object1],
        rigid_b_index=object_to_rigid_index[rbc.object2],
        local_matrix=_joint_model_matrix(root, armature, obj, rbc, rigid_by_object),
        linear_lower=(
            float(rbc.limit_lin_x_lower),
            float(rbc.limit_lin_y_lower),
            float(rbc.limit_lin_z_lower),
        ),
        linear_upper=(
            float(rbc.limit_lin_x_upper),
            float(rbc.limit_lin_y_upper),
            float(rbc.limit_lin_z_upper),
        ),
        angular_lower=(
            float(rbc.limit_ang_x_lower),
            float(rbc.limit_ang_y_lower),
            float(rbc.limit_ang_z_lower),
        ),
        angular_upper=(
            float(rbc.limit_ang_x_upper),
            float(rbc.limit_ang_y_upper),
            float(rbc.limit_ang_z_upper),
        ),
        spring_linear=_vec3(getattr(mmd_joint, "spring_linear", (0.0, 0.0, 0.0))),
        spring_linear_damping=(1.0, 1.0, 1.0),
        spring_angular=_vec3(getattr(mmd_joint, "spring_angular", (0.0, 0.0, 0.0))),
        spring_angular_damping=(1.0, 1.0, 1.0),
        disable_collisions=bool(getattr(rbc, "disable_collisions", False)),
        joint_stop_erp=-1.0,
        joint_stop_cfm=-1.0,
        locked_joint_stop_erp=-1.0,
        locked_joint_stop_cfm=-1.0,
    )


def _build_non_collision_pairs(rigid_bodies, joints, object_to_rigid_index, distance_scale=1.5):
    groups = [[] for _ in range(16)]
    for rigid in rigid_bodies:
        group = max(0, min(15, int(rigid.collision_group_number)))
        groups[group].append(rigid)

    pairs = set()
    joint_by_pair = {}
    for joint in joints:
        pair = tuple(sorted((joint.rigid_a_index, joint.rigid_b_index)))
        joint_by_pair[pair] = joint

    for rigid_a in rigid_bodies:
        for group_index in range(16):
            if not (rigid_a.no_collision_mask & (1 << group_index)):
                continue
            for rigid_b in groups[group_index]:
                if rigid_a.index == rigid_b.index:
                    continue
                pair = tuple(sorted((rigid_a.index, rigid_b.index)))
                if pair in pairs:
                    continue
                joint = joint_by_pair.get(pair)
                if joint is not None:
                    joint.disable_collisions = True
                    pairs.add(pair)
                    continue
                distance = (rigid_a.local_matrix.to_translation() - rigid_b.local_matrix.to_translation()).length
                max_distance = distance_scale * (_rigid_range(rigid_a.obj) + _rigid_range(rigid_b.obj)) * 0.5
                if distance < max_distance:
                    pairs.add(pair)

    return sorted(pairs)


def _is_direct_parent_bone(armature, parent_name, child_name):
    if armature is None or not parent_name or not child_name or parent_name == child_name:
        return False
    pose_bones = getattr(armature, "pose", None)
    if pose_bones is None:
        return False
    child = armature.pose.bones.get(child_name)
    return child is not None and child.parent is not None and child.parent.name == parent_name


def _apply_dynamic_parent_chain_correction(rigid_bodies, joints, armature, settings):
    if not bool(getattr(settings, "dynamic_parent_chain_correction", True)):
        return 0

    corrected = set()
    by_index = {rigid.index: rigid for rigid in rigid_bodies}
    for joint in joints:
        rigid_a = by_index.get(joint.rigid_a_index)
        rigid_b = by_index.get(joint.rigid_b_index)
        if rigid_a is None or rigid_b is None:
            continue

        if (
            rigid_a.mode != MODE_STATIC
            and rigid_b.mode == MODE_DYNAMIC_BONE
            and _is_direct_parent_bone(armature, rigid_a.bone_name, rigid_b.bone_name)
        ):
            rigid_b.mode = MODE_DYNAMIC
            corrected.add(rigid_b.index)
        elif (
            rigid_b.mode != MODE_STATIC
            and rigid_a.mode == MODE_DYNAMIC_BONE
            and _is_direct_parent_bone(armature, rigid_b.bone_name, rigid_a.bone_name)
        ):
            rigid_a.mode = MODE_DYNAMIC
            corrected.add(rigid_a.index)

    return len(corrected)


def read_model(context_or_object, root=None):
    if root is None:
        if _mmd_type(context_or_object) == "ROOT" or hasattr(context_or_object, "matrix_world"):
            root = find_root_object(context_or_object)
        else:
            root = find_root_object(getattr(context_or_object, "active_object", None))
    if root is None:
        raise RuntimeError("No mmd_tools model root selected")

    armature = _find_armature(root)
    rigid_objects = list(_iter_by_mmd_type(root, "RIGID_BODY"))
    joint_objects = list(_iter_by_mmd_type(root, "JOINT"))
    if not rigid_objects:
        raise RuntimeError("Selected model has no mmd_tools rigid bodies")

    rigid_bodies = []
    object_to_rigid_index = {}
    rigid_by_object = {}
    for index, obj in enumerate(rigid_objects):
        rigid = _read_rigid(root, armature, obj, index)
        rigid_bodies.append(rigid)
        object_to_rigid_index[obj] = index
        rigid_by_object[obj] = rigid

    joints = []
    for obj in joint_objects:
        joint = _read_joint(root, armature, obj, len(joints), object_to_rigid_index, rigid_by_object)
        if joint is not None:
            joints.append(joint)

    settings = _context_settings(context_or_object)
    parent_chain_corrections = _apply_dynamic_parent_chain_correction(rigid_bodies, joints, armature, settings)
    non_collision_pairs = _build_non_collision_pairs(rigid_bodies, joints, object_to_rigid_index)
    matched_rules = _apply_zone_rules(
        PmxModelData(
            root=root,
            armature=armature,
            rigid_bodies=rigid_bodies,
            joints=joints,
            object_to_rigid_index=object_to_rigid_index,
            non_collision_pairs=non_collision_pairs,
        ),
        settings,
    )

    model = PmxModelData(
        root=root,
        armature=armature,
        rigid_bodies=rigid_bodies,
        joints=joints,
        object_to_rigid_index=object_to_rigid_index,
        non_collision_pairs=non_collision_pairs,
    )
    setattr(model, "zone_rules", matched_rules)
    setattr(model, "parent_chain_corrections", parent_chain_corrections)
    return model


def bone_model_matrix(model, bone_name):
    return _bone_model_matrix(model.root, model.armature, bone_name)


def bone_rest_model_matrix(model, bone_name):
    return _bone_rest_model_matrix(model.root, model.armature, bone_name)


def diagnose_model(context_or_object, root=None):
    if root is None:
        if _mmd_type(context_or_object) == "ROOT" or hasattr(context_or_object, "matrix_world"):
            root = find_root_object(context_or_object)
        else:
            root = find_root_object(getattr(context_or_object, "active_object", None))

    report = {
        "root": root,
        "armature": _find_armature(root) if root is not None else None,
        "rigid_count": 0,
        "joint_count": 0,
        "static_count": 0,
        "dynamic_count": 0,
        "dynamic_bone_count": 0,
        "invalid_joint_count": 0,
        "warnings": [],
        "errors": [],
    }

    if root is None:
        report["errors"].append("No mmd_tools model root selected")
        return report

    rigid_objects = list(_iter_by_mmd_type(root, "RIGID_BODY"))
    joint_objects = list(_iter_by_mmd_type(root, "JOINT"))
    report["rigid_count"] = len(rigid_objects)
    report["joint_count"] = len(joint_objects)

    if not rigid_objects:
        report["errors"].append("Selected model has no mmd_tools rigid bodies")
        report["warnings"].append("Check whether the model was imported with mmd_tools rigid bodies enabled")
        return report

    rigid_by_object = {}
    object_to_rigid_index = {}
    for index, obj in enumerate(rigid_objects):
        rigid = _read_rigid(root, report["armature"], obj, index)
        rigid_by_object[obj] = rigid
        object_to_rigid_index[obj] = index
        if rigid.mode == MODE_STATIC:
            report["static_count"] += 1
        elif rigid.mode == MODE_DYNAMIC:
            report["dynamic_count"] += 1
        elif rigid.mode == MODE_DYNAMIC_BONE:
            report["dynamic_bone_count"] += 1

    for obj in joint_objects:
        rbc = getattr(obj, "rigid_body_constraint", None)
        if rbc is None or rbc.object1 not in object_to_rigid_index or rbc.object2 not in object_to_rigid_index:
            report["invalid_joint_count"] += 1

    if report["invalid_joint_count"]:
        report["warnings"].append(f"{report['invalid_joint_count']} joint(s) do not reference two valid rigid bodies")

    if report["dynamic_count"] == 0 and report["dynamic_bone_count"] == 0:
        report["warnings"].append("No dynamic rigid bodies were detected; the model may only contain static collision parts")

    if report["armature"] is None:
        report["warnings"].append("No armature was found under the mmd_tools root")

    return report
