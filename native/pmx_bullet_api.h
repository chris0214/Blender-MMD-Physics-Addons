#pragma once

#include <stdint.h>

#ifdef _WIN32
#  define PMX_BULLET_API extern "C" __declspec(dllexport)
#else
#  define PMX_BULLET_API extern "C"
#endif

enum PmxBtShape
{
    PMX_BT_SHAPE_SPHERE = 0,
    PMX_BT_SHAPE_BOX = 1,
    PMX_BT_SHAPE_CAPSULE = 2,
};

enum PmxBtRigidMode
{
    PMX_BT_MODE_STATIC = 0,
    PMX_BT_MODE_DYNAMIC = 1,
    PMX_BT_MODE_DYNAMIC_BONE = 2,
};

struct PmxBtRigidDesc
{
    int index;
    int shape;
    int mode;
    float size[3];
    float mass;
    float friction;
    float restitution;
    float linear_damping;
    float angular_damping;
    uint16_t collision_group;
    uint16_t collision_mask;
    float position[3];
    float rotation[4]; // x, y, z, w
};

struct PmxBtJointDesc
{
    int index;
    int rigid_a;
    int rigid_b;
    int disable_collisions;
    float position[3];
    float rotation[4]; // x, y, z, w
    float linear_lower[3];
    float linear_upper[3];
    float angular_lower[3];
    float angular_upper[3];
    float spring_linear[3];
    float spring_linear_damping[3];
    float spring_angular[3];
    float spring_angular_damping[3];
    float joint_stop_erp;
    float joint_stop_cfm;
    float locked_joint_stop_erp;
    float locked_joint_stop_cfm;
};

struct PmxBtBodyTransform
{
    int index;
    float position[3];
    float rotation[4]; // x, y, z, w
};

struct PmxBtNonCollisionPair
{
    int rigid_a;
    int rigid_b;
};

struct PmxBtModelPair
{
    int model_a;
    int model_b;
};

struct PmxBtBodyPair
{
    int body_a;
    int body_b;
};

PMX_BULLET_API int pmx_bullet_api_version();
PMX_BULLET_API void* pmx_bullet_create_world();
PMX_BULLET_API void pmx_bullet_destroy_world(void* world);
PMX_BULLET_API void pmx_bullet_set_gravity(void* world, float x, float y, float z);
PMX_BULLET_API int pmx_bullet_set_solver_iterations(void* world, int iterations);
PMX_BULLET_API int pmx_bullet_set_joint_quality(
    void* world,
    int use_frame_offset,
    float joint_stop_erp,
    float joint_stop_cfm,
    float locked_joint_stop_erp,
    float locked_joint_stop_cfm);
PMX_BULLET_API int pmx_bullet_set_stabilization(
    void* world,
    int locked_joint_pullback,
    int resting_body_stabilization);
PMX_BULLET_API int pmx_bullet_add_rigid_bodies(void* world, const PmxBtRigidDesc* bodies, int count);
PMX_BULLET_API int pmx_bullet_set_body_model_ids(void* world, int start_body, int count, int model_index);
PMX_BULLET_API int pmx_bullet_set_disabled_model_pairs(void* world, const PmxBtModelPair* pairs, int count);
PMX_BULLET_API int pmx_bullet_set_cross_model_body_pair_filter_enabled(void* world, int enabled);
PMX_BULLET_API int pmx_bullet_set_enabled_cross_model_body_pairs(void* world, const PmxBtBodyPair* pairs, int count);
PMX_BULLET_API int pmx_bullet_add_non_collision_pairs(void* world, const PmxBtNonCollisionPair* pairs, int count);
PMX_BULLET_API int pmx_bullet_add_joints(void* world, const PmxBtJointDesc* joints, int count);
PMX_BULLET_API int pmx_bullet_temporal_kinematic_init(void* world, const PmxBtBodyTransform* transforms, int count);
PMX_BULLET_API int pmx_bullet_set_kinematic_transforms(void* world, const PmxBtBodyTransform* transforms, int count);
PMX_BULLET_API int pmx_bullet_freeze_body_transforms(void* world, const PmxBtBodyTransform* transforms, int count);
PMX_BULLET_API int pmx_bullet_step(void* world, float fixed_timestep, int max_substeps);
PMX_BULLET_API int pmx_bullet_get_body_transforms(void* world, PmxBtBodyTransform* transforms, int count);
PMX_BULLET_API int pmx_bullet_reset(void* world);
PMX_BULLET_API int pmx_bullet_prewarm(void* world, int steps, float timestep);
