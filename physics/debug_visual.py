import bpy

from . import pmx_data_reader
from .types import MODE_DYNAMIC, MODE_DYNAMIC_BONE, MODE_STATIC


_STATE_KEY = "pmx_physics_debug_visual_state"
_MATERIAL_PREFIX = "PMX Physics Debug "

_MODE_COLORS = {
    MODE_STATIC: (0.35, 0.7, 1.0, 0.36),
    MODE_DYNAMIC: (0.8, 1.0, 0.32, 0.36),
    MODE_DYNAMIC_BONE: (1.0, 0.58, 0.5, 0.36),
}
_JOINT_COLOR = (0.6, 0.75, 1.0, 0.42)
_GROUP_COLORS = (
    (0.95, 0.35, 0.35, 0.36),
    (0.95, 0.62, 0.25, 0.36),
    (0.95, 0.9, 0.28, 0.36),
    (0.55, 0.9, 0.3, 0.36),
    (0.35, 0.82, 0.7, 0.36),
    (0.35, 0.65, 1.0, 0.36),
    (0.65, 0.48, 1.0, 0.36),
    (0.95, 0.45, 0.9, 0.36),
)


def _state():
    state = bpy.app.driver_namespace.get(_STATE_KEY)
    if state is None:
        state = {}
        bpy.app.driver_namespace[_STATE_KEY] = state
    return state


def _save_object_state(obj, state):
    if obj.name in state:
        return
    state[obj.name] = {
        "color": tuple(obj.color),
        "display_type": obj.display_type,
        "show_name": obj.show_name,
        "show_in_front": obj.show_in_front,
        "show_wire": obj.show_wire,
        "hide_viewport": obj.hide_viewport,
        "hidden": obj.hide_get(),
        "material_slots": [slot.material for slot in obj.material_slots],
    }


def _restore_object_state(obj, saved):
    obj.color = saved["color"]
    obj.display_type = saved["display_type"]
    obj.show_name = saved["show_name"]
    obj.show_in_front = saved["show_in_front"]
    obj.show_wire = saved.get("show_wire", False)
    obj.hide_viewport = saved["hide_viewport"]
    obj.hide_set(saved["hidden"])
    if obj.type == "MESH":
        obj.data.materials.clear()
        for material in saved.get("material_slots", []):
            obj.data.materials.append(material)


def _debug_material(name, color):
    material_name = _MATERIAL_PREFIX + name
    material = bpy.data.materials.get(material_name)
    if material is None:
        material = bpy.data.materials.new(material_name)
    material.diffuse_color = color
    material.use_nodes = True
    material.blend_method = "BLEND"
    material.show_transparent_back = True
    material.use_screen_refraction = False
    material.alpha_threshold = 0.01
    material.show_transparent_back = True

    bsdf = material.node_tree.nodes.get("Principled BSDF")
    if bsdf is not None:
        if "Alpha" in bsdf.inputs:
            bsdf.inputs["Alpha"].default_value = color[3]
        if "Base Color" in bsdf.inputs:
            bsdf.inputs["Base Color"].default_value = color
        if "Roughness" in bsdf.inputs:
            bsdf.inputs["Roughness"].default_value = 0.85
    return material


def _apply_material(obj, color, material_name):
    if obj.type != "MESH":
        return
    material = _debug_material(material_name, color)
    obj.data.materials.clear()
    obj.data.materials.append(material)


def _apply_object_visual(obj, color, force_visible, material_name):
    obj.color = color
    _apply_material(obj, color, material_name)
    obj.display_type = "TEXTURED"
    obj.show_name = False
    obj.show_in_front = True
    obj.show_wire = True
    if force_visible:
        obj.hide_viewport = False
        obj.hide_set(False)


def clear_debug_visuals():
    state = _state()
    restored = 0
    for obj_name, saved in list(state.items()):
        obj = bpy.data.objects.get(obj_name)
        if obj is not None:
            _restore_object_state(obj, saved)
            restored += 1
    state.clear()
    return restored


def _apply_model_debug_visuals(context, root, mode, force_visible):
    model = pmx_data_reader.read_model(context, root)
    state = _state()
    count = 0

    for rigid in model.rigid_bodies:
        obj = rigid.obj
        if obj is None:
            continue
        _save_object_state(obj, state)
        if mode == "COLLISION_GROUP":
            group = int(rigid.collision_group_number)
            color = _GROUP_COLORS[group % len(_GROUP_COLORS)]
            material_name = f"Group {group:02d}"
        else:
            color = _MODE_COLORS.get(rigid.mode, (0.7, 0.7, 0.7, 1.0))
            if rigid.mode == MODE_STATIC:
                material_name = "Static"
            elif rigid.mode == MODE_DYNAMIC_BONE:
                material_name = "Dynamic Bone"
            else:
                material_name = "Dynamic"
        _apply_object_visual(obj, color, force_visible, material_name)
        count += 1

    for joint in model.joints:
        obj = joint.obj
        if obj is None:
            continue
        _save_object_state(obj, state)
        _apply_object_visual(obj, _JOINT_COLOR, force_visible, "Joint")
        count += 1

    return count


def apply_debug_visuals(context, root, mode="BODY_MODE", force_visible=True):
    clear_debug_visuals()
    return _apply_model_debug_visuals(context, root, mode, force_visible)


def apply_debug_visuals_for_roots(context, roots, mode="BODY_MODE", force_visible=True):
    clear_debug_visuals()
    total = 0
    models = 0
    for root in roots:
        if root is None:
            continue
        total += _apply_model_debug_visuals(context, root, mode, force_visible)
        models += 1
    return models, total
