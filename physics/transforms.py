from mathutils import Matrix, Quaternion, Vector


def blender_vector_to_pmx(value):
    return Vector(value)


def pmx_vector_to_blender(value):
    return Vector(value)


def blender_quaternion_to_pmx(quat):
    return quat.copy() if isinstance(quat, Quaternion) else Quaternion(quat)


def pmx_quaternion_to_blender(quat):
    return quat.copy() if isinstance(quat, Quaternion) else Quaternion(quat)


def blender_matrix_to_pmx_transform(matrix):
    loc, rot, _scale = matrix.decompose()
    return blender_vector_to_pmx(loc), blender_quaternion_to_pmx(rot)


def pmx_transform_to_blender_matrix(position, rotation):
    loc = pmx_vector_to_blender(position)
    quat = rotation if isinstance(rotation, Quaternion) else Quaternion(rotation)
    quat = pmx_quaternion_to_blender(quat)
    return Matrix.Translation(loc) @ quat.to_matrix().to_4x4()
