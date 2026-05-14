#include "pmx_bullet_api.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <set>
#include <vector>

#include <btBulletDynamicsCommon.h>
#include <BulletDynamics/ConstraintSolver/btGeneric6DofSpringConstraint.h>

namespace {

constexpr int kApiVersion = 9;
constexpr int kSolverIterations = 20;
constexpr int kConstraintStopErp = 2;
constexpr int kConstraintStopCfm = 4;
constexpr btScalar kLinearRestVelocity2 = btScalar(2.5e-7);
constexpr btScalar kAngularRestVelocity2 = btScalar(9.0e-6);
constexpr btScalar kLockedLinearStopCfm = btScalar(2.0e-4);
constexpr btScalar kLockedLinearStopErp = btScalar(0.2);
constexpr btScalar kLockedJointCorrectionMinError2 = btScalar(4.0e-6);
constexpr btScalar kLockedJointCorrectionFactor = btScalar(0.15);
constexpr btScalar kUnsetConstraintParam = btScalar(-1.0);
constexpr btScalar kKinematicWakeMove2 = btScalar(1.0e-10);
constexpr btScalar kKinematicWakeAngle2 = btScalar(1.0e-10);
constexpr btScalar kRestMove2 = btScalar(4.0e-8);
constexpr btScalar kRestAngle2 = btScalar(2.5e-10);
constexpr int kRestFramesToSleep = 30;
constexpr int kDynamicModes[2] = {PMX_BT_MODE_DYNAMIC, PMX_BT_MODE_DYNAMIC_BONE};

btVector3 to_vec3(const float value[3])
{
    return btVector3(value[0], value[1], value[2]);
}

btQuaternion to_quat(const float value[4])
{
    return btQuaternion(value[0], value[1], value[2], value[3]);
}

btTransform to_transform(const float position[3], const float rotation[4])
{
    btTransform transform;
    transform.setIdentity();
    transform.setOrigin(to_vec3(position));
    transform.setRotation(to_quat(rotation));
    return transform;
}

bool is_dynamic_mode(int mode)
{
    return mode == kDynamicModes[0] || mode == kDynamicModes[1];
}

struct PmxOverlapFilter : public btOverlapFilterCallback {
    std::set<std::pair<const btBroadphaseProxy*, const btBroadphaseProxy*>> ignored_pairs;
    const std::vector<int>* body_model_ids = nullptr;
    std::set<std::pair<int, int>> disabled_model_pairs;
    bool use_cross_model_body_pair_filter = false;
    std::set<std::pair<int, int>> enabled_cross_model_body_pairs;

    bool needBroadphaseCollision(btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1) const override
    {
        const auto ordered = ordered_pair(proxy0, proxy1);
        if (ignored_pairs.find(ordered) != ignored_pairs.end()) {
            return false;
        }

        if (body_model_ids != nullptr) {
            const int body0 = body_index(proxy0);
            const int body1 = body_index(proxy1);
            const int model0 = model_index(proxy0);
            const int model1 = model_index(proxy1);
            if (model0 >= 0 && model1 >= 0 && model0 != model1) {
                if (use_cross_model_body_pair_filter &&
                    enabled_cross_model_body_pairs.find(ordered_index_pair(body0, body1)) == enabled_cross_model_body_pairs.end()) {
                    return false;
                }
                if (disabled_model_pairs.find(ordered_model_pair(model0, model1)) != disabled_model_pairs.end()) {
                    return false;
                }
            }
        }

        bool collides = (proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask) != 0;
        collides = collides && (proxy1->m_collisionFilterGroup & proxy0->m_collisionFilterMask);
        return collides;
    }

    void add(btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1)
    {
        ignored_pairs.insert(ordered_pair(proxy0, proxy1));
    }

    static std::pair<const btBroadphaseProxy*, const btBroadphaseProxy*> ordered_pair(
        const btBroadphaseProxy* proxy0,
        const btBroadphaseProxy* proxy1)
    {
        if (std::less<const btBroadphaseProxy*>()(proxy1, proxy0)) {
            std::swap(proxy0, proxy1);
        }
        return std::make_pair(proxy0, proxy1);
    }

    static std::pair<int, int> ordered_model_pair(int model0, int model1)
    {
        if (model1 < model0) {
            std::swap(model0, model1);
        }
        return std::make_pair(model0, model1);
    }

    static std::pair<int, int> ordered_index_pair(int index0, int index1)
    {
        if (index1 < index0) {
            std::swap(index0, index1);
        }
        return std::make_pair(index0, index1);
    }

    int body_index(const btBroadphaseProxy* proxy) const
    {
        if (proxy == nullptr || proxy->m_clientObject == nullptr) {
            return -1;
        }
        const btCollisionObject* object = static_cast<const btCollisionObject*>(proxy->m_clientObject);
        return object->getUserIndex();
    }

    int model_index(const btBroadphaseProxy* proxy) const
    {
        if (body_model_ids == nullptr) {
            return -1;
        }
        const int body_index = this->body_index(proxy);
        if (body_index < 0 || body_index >= static_cast<int>(body_model_ids->size())) {
            return -1;
        }
        return (*body_model_ids)[body_index];
    }
};

struct PmxJointRuntime {
    btGeneric6DofSpringConstraint* constraint = nullptr;
    int rigid_a = -1;
    int rigid_b = -1;
    bool locked_translation = false;
    btScalar joint_stop_erp = kUnsetConstraintParam;
    btScalar joint_stop_cfm = kUnsetConstraintParam;
    btScalar locked_joint_stop_erp = kUnsetConstraintParam;
    btScalar locked_joint_stop_cfm = kUnsetConstraintParam;
};

struct PmxWorld {
    std::unique_ptr<btDefaultCollisionConfiguration> collision_configuration;
    std::unique_ptr<btCollisionDispatcher> dispatcher;
    std::unique_ptr<btDbvtBroadphase> broadphase;
    std::unique_ptr<btSequentialImpulseConstraintSolver> solver;
    std::unique_ptr<btDiscreteDynamicsWorld> dynamics_world;

    std::vector<std::unique_ptr<btCollisionShape>> shapes;
    std::vector<std::unique_ptr<btDefaultMotionState>> motion_states;
    std::vector<std::unique_ptr<btRigidBody>> bodies;
    std::vector<std::unique_ptr<btTypedConstraint>> constraints;
    std::vector<PmxJointRuntime> joint_runtimes;
    std::vector<btTransform> initial_transforms;
    std::vector<btTransform> last_transforms;
    std::vector<int> resting_frames;
    std::vector<int> body_modes;
    std::vector<int> body_model_ids;
    PmxOverlapFilter overlap_filter;
    int solver_iterations = kSolverIterations;
    bool use_frame_offset = true;
    bool use_locked_joint_pullback = true;
    bool use_resting_body_stabilization = false;
    btScalar joint_stop_erp = kUnsetConstraintParam;
    btScalar joint_stop_cfm = kUnsetConstraintParam;
    btScalar locked_joint_stop_erp = kLockedLinearStopErp;
    btScalar locked_joint_stop_cfm = kLockedLinearStopCfm;

    PmxWorld()
    {
        collision_configuration = std::make_unique<btDefaultCollisionConfiguration>();
        dispatcher = std::make_unique<btCollisionDispatcher>(collision_configuration.get());
        broadphase = std::make_unique<btDbvtBroadphase>();
        solver = std::make_unique<btSequentialImpulseConstraintSolver>();
        dynamics_world = std::make_unique<btDiscreteDynamicsWorld>(
            dispatcher.get(),
            broadphase.get(),
            solver.get(),
            collision_configuration.get());
        dynamics_world->setGravity(btVector3(0.0f, -9.8f, 0.0f));
        dynamics_world->getSolverInfo().m_numIterations = solver_iterations;
        dynamics_world->getSolverInfo().m_solverMode |= SOLVER_USE_WARMSTARTING;
        overlap_filter.body_model_ids = &body_model_ids;
        dynamics_world->getPairCache()->setOverlapFilterCallback(&overlap_filter);
    }

    ~PmxWorld()
    {
        if (dynamics_world) {
            for (auto& constraint : constraints) {
                dynamics_world->removeConstraint(constraint.get());
            }
            for (auto& body : bodies) {
                dynamics_world->removeRigidBody(body.get());
            }
        }
    }
};

btScalar effective_param(btScalar local_value, btScalar global_value)
{
    return local_value >= btScalar(0.0) ? local_value : global_value;
}

void apply_joint_quality(
    PmxWorld& world,
    const PmxJointRuntime& runtime,
    btGeneric6DofSpringConstraint& constraint)
{
    const btScalar joint_stop_erp = effective_param(runtime.joint_stop_erp, world.joint_stop_erp);
    const btScalar joint_stop_cfm = effective_param(runtime.joint_stop_cfm, world.joint_stop_cfm);
    const btScalar locked_joint_stop_erp = effective_param(runtime.locked_joint_stop_erp, world.locked_joint_stop_erp);
    const btScalar locked_joint_stop_cfm = effective_param(runtime.locked_joint_stop_cfm, world.locked_joint_stop_cfm);

    for (int axis = 0; axis < 6; ++axis) {
        if (joint_stop_erp >= btScalar(0.0)) {
            constraint.setParam(kConstraintStopErp, joint_stop_erp, axis);
        }
        if (joint_stop_cfm >= btScalar(0.0)) {
            constraint.setParam(kConstraintStopCfm, joint_stop_cfm, axis);
        }
    }

    if (runtime.locked_translation) {
        for (int axis = 0; axis < 3; ++axis) {
            if (locked_joint_stop_cfm >= btScalar(0.0)) {
                constraint.setParam(kConstraintStopCfm, locked_joint_stop_cfm, axis);
            }
            if (locked_joint_stop_erp >= btScalar(0.0)) {
                constraint.setParam(kConstraintStopErp, locked_joint_stop_erp, axis);
            }
        }
    }
}

btCollisionShape* make_shape(PmxWorld& world, const PmxBtRigidDesc& desc)
{
    std::unique_ptr<btCollisionShape> shape;
    switch (desc.shape) {
        case PMX_BT_SHAPE_BOX:
            shape = std::make_unique<btBoxShape>(btVector3(desc.size[0], desc.size[1], desc.size[2]));
            break;
        case PMX_BT_SHAPE_CAPSULE:
            shape = std::make_unique<btCapsuleShapeZ>(desc.size[0], desc.size[1]);
            break;
        case PMX_BT_SHAPE_SPHERE:
        default:
            shape = std::make_unique<btSphereShape>(desc.size[0]);
            break;
    }
    btCollisionShape* raw = shape.get();
    world.shapes.push_back(std::move(shape));
    return raw;
}

void set_body_transform(btRigidBody& body, const btTransform& transform)
{
    body.setWorldTransform(transform);
    body.setInterpolationWorldTransform(transform);
    if (body.getMotionState()) {
        body.getMotionState()->setWorldTransform(transform);
    }
}

int set_kinematic_transform(PmxWorld& world, const PmxBtBodyTransform& body_transform, bool activate_body = true)
{
    if (body_transform.index < 0 || body_transform.index >= static_cast<int>(world.bodies.size())) {
        return 0;
    }
    btRigidBody& body = *world.bodies[body_transform.index];
    set_body_transform(body, to_transform(body_transform.position, body_transform.rotation));
    if (activate_body) {
        body.activate(true);
    }
    return 1;
}

void clean_body_pairs(PmxWorld& world, btRigidBody& body)
{
    btBroadphaseProxy* proxy = body.getBroadphaseHandle();
    if (proxy == nullptr) {
        return;
    }
    world.dynamics_world->getPairCache()->cleanProxyFromPairs(proxy, world.dispatcher.get());
}

void refresh_body_proxy(PmxWorld& world, btRigidBody& body)
{
    if (body.getBroadphaseHandle() == nullptr) {
        return;
    }
    clean_body_pairs(world, body);
}

void clean_all_body_pairs(PmxWorld& world)
{
    for (const auto& body : world.bodies) {
        if (body && body->getBroadphaseHandle() != nullptr) {
            clean_body_pairs(world, *body);
        }
    }
}

void refresh_world_pairs(PmxWorld& world)
{
    world.dynamics_world->updateAabbs();
    world.dynamics_world->computeOverlappingPairs();
}

int body_model_id(const PmxWorld& world, const btCollisionObject* object)
{
    if (object == nullptr) {
        return -1;
    }
    const int body_index = object->getUserIndex();
    if (body_index < 0 || body_index >= static_cast<int>(world.body_model_ids.size())) {
        return -1;
    }
    return world.body_model_ids[body_index];
}

bool disabled_model_pair(const PmxWorld& world, const btCollisionObject* object0, const btCollisionObject* object1)
{
    const int model0 = body_model_id(world, object0);
    const int model1 = body_model_id(world, object1);
    if (model0 < 0 || model1 < 0 || model0 == model1) {
        return false;
    }
    return world.overlap_filter.disabled_model_pairs.find(
        PmxOverlapFilter::ordered_model_pair(model0, model1)) != world.overlap_filter.disabled_model_pairs.end();
}

bool disabled_cross_model_body_pair(const PmxWorld& world, int index0, int index1)
{
    if (!world.overlap_filter.use_cross_model_body_pair_filter) {
        return false;
    }
    if (index0 < 0 || index1 < 0 ||
        index0 >= static_cast<int>(world.body_model_ids.size()) ||
        index1 >= static_cast<int>(world.body_model_ids.size())) {
        return false;
    }
    const int model0 = world.body_model_ids[index0];
    const int model1 = world.body_model_ids[index1];
    if (model0 < 0 || model1 < 0 || model0 == model1) {
        return false;
    }
    return world.overlap_filter.enabled_cross_model_body_pairs.find(
        PmxOverlapFilter::ordered_index_pair(index0, index1)) == world.overlap_filter.enabled_cross_model_body_pairs.end();
}

bool disabled_cross_model_body_pair(const PmxWorld& world, const btCollisionObject* object0, const btCollisionObject* object1)
{
    if (object0 == nullptr || object1 == nullptr) {
        return false;
    }
    return disabled_cross_model_body_pair(world, object0->getUserIndex(), object1->getUserIndex());
}

int body_index_from_proxy(const btBroadphaseProxy* proxy)
{
    if (proxy == nullptr || proxy->m_clientObject == nullptr) {
        return -1;
    }
    const auto* object = static_cast<const btCollisionObject*>(proxy->m_clientObject);
    return object->getUserIndex();
}

bool same_model_body_indices(const PmxWorld& world, int index0, int index1)
{
    if (index0 < 0 || index1 < 0 ||
        index0 >= static_cast<int>(world.body_model_ids.size()) ||
        index1 >= static_cast<int>(world.body_model_ids.size())) {
        return false;
    }
    const int model0 = world.body_model_ids[index0];
    const int model1 = world.body_model_ids[index1];
    return model0 >= 0 && model1 >= 0 && model0 == model1;
}

bool disabled_model_pair_indices(const PmxWorld& world, int index0, int index1)
{
    if (disabled_cross_model_body_pair(world, index0, index1)) {
        return true;
    }
    if (index0 < 0 || index1 < 0 ||
        index0 >= static_cast<int>(world.body_model_ids.size()) ||
        index1 >= static_cast<int>(world.body_model_ids.size())) {
        return false;
    }
    const int model0 = world.body_model_ids[index0];
    const int model1 = world.body_model_ids[index1];
    if (model0 < 0 || model1 < 0 || model0 == model1) {
        return false;
    }
    return world.overlap_filter.disabled_model_pairs.find(
        PmxOverlapFilter::ordered_model_pair(model0, model1)) != world.overlap_filter.disabled_model_pairs.end();
}

void remove_disabled_model_pairs(PmxWorld& world)
{
    btOverlappingPairCache* cache = world.dynamics_world->getPairCache();
    if (cache == nullptr) {
        return;
    }

    btBroadphasePairArray& pairs = cache->getOverlappingPairArray();
    for (int i = pairs.size() - 1; i >= 0; --i) {
        btBroadphasePair& pair = pairs[i];
        const int index_a = body_index_from_proxy(pair.m_pProxy0);
        const int index_b = body_index_from_proxy(pair.m_pProxy1);
        if (!disabled_model_pair_indices(world, index_a, index_b)) {
            continue;
        }
        cache->removeOverlappingPair(pair.m_pProxy0, pair.m_pProxy1, world.dispatcher.get());
    }
}

void clear_disabled_model_manifolds(PmxWorld& world)
{
    const int count = world.dispatcher->getNumManifolds();
    for (int i = 0; i < count; ++i) {
        btPersistentManifold* manifold = world.dispatcher->getManifoldByIndexInternal(i);
        if (manifold == nullptr) {
            continue;
        }
        const btCollisionObject* object0 = static_cast<const btCollisionObject*>(manifold->getBody0());
        const btCollisionObject* object1 = static_cast<const btCollisionObject*>(manifold->getBody1());
        if (disabled_model_pair(world, object0, object1) || disabled_cross_model_body_pair(world, object0, object1)) {
            manifold->clearManifold();
        }
    }
}

bool is_locked_translation_joint(const PmxBtJointDesc& desc)
{
    constexpr btScalar kEpsilon = btScalar(1.0e-5);
    for (int axis = 0; axis < 3; ++axis) {
        if (std::fabs(desc.linear_lower[axis]) > kEpsilon ||
            std::fabs(desc.linear_upper[axis]) > kEpsilon ||
            std::fabs(desc.linear_lower[axis] - desc.linear_upper[axis]) > kEpsilon) {
            return false;
        }
    }
    return true;
}

btScalar rotation_delta2(const btQuaternion& a, const btQuaternion& b)
{
    const btScalar dot = btFabs(a.dot(b));
    const btScalar clamped = btMin(dot, btScalar(1.0));
    const btScalar delta = btScalar(1.0) - clamped;
    return delta * delta;
}

bool transform_changed(const btTransform& a, const btTransform& b, btScalar move2, btScalar angle2)
{
    if ((a.getOrigin() - b.getOrigin()).length2() > move2) {
        return true;
    }
    return rotation_delta2(a.getRotation(), b.getRotation()) > angle2;
}

void reset_rest_state(PmxWorld& world, int index, const btTransform& transform)
{
    if (index < 0 || index >= static_cast<int>(world.last_transforms.size())) {
        return;
    }
    world.last_transforms[index] = transform;
    world.resting_frames[index] = 0;
}

bool valid_body_index(const PmxWorld& world, int index)
{
    return index >= 0 && index < static_cast<int>(world.bodies.size());
}

bool is_dynamic_body(const PmxWorld& world, int index)
{
    return valid_body_index(world, index) && is_dynamic_mode(world.body_modes[index]);
}

void wake_dynamic_body(PmxWorld& world, int index)
{
    if (!is_dynamic_body(world, index)) {
        return;
    }
    btRigidBody& body = *world.bodies[index];
    body.forceActivationState(ACTIVE_TAG);
    body.activate(true);
    reset_rest_state(world, index, body.getWorldTransform());
}

void wake_dynamic_bodies(PmxWorld& world)
{
    for (size_t i = 0; i < world.bodies.size(); ++i) {
        wake_dynamic_body(world, static_cast<int>(i));
    }
}

void collect_joint_connected_dynamics(PmxWorld& world, const std::vector<int>& seeds, std::set<int>& wake_indices)
{
    std::set<int> visited;
    std::vector<int> queue;
    for (int seed : seeds) {
        if (!valid_body_index(world, seed)) {
            continue;
        }
        if (visited.insert(seed).second) {
            queue.push_back(seed);
        }
    }

    for (size_t cursor = 0; cursor < queue.size(); ++cursor) {
        const int current = queue[cursor];
        for (const PmxJointRuntime& runtime : world.joint_runtimes) {
            int other = -1;
            if (runtime.rigid_a == current) {
                other = runtime.rigid_b;
            }
            else if (runtime.rigid_b == current) {
                other = runtime.rigid_a;
            }
            if (!valid_body_index(world, other)) {
                continue;
            }
            if (is_dynamic_body(world, other)) {
                wake_indices.insert(other);
            }
            if (visited.insert(other).second) {
                queue.push_back(other);
            }
        }
    }
}

void collect_overlapping_dynamics(
    PmxWorld& world,
    const std::set<int>& changed_kinematic_indices,
    std::set<int>& wake_indices)
{
    btBroadphasePairArray& pairs = world.dynamics_world->getPairCache()->getOverlappingPairArray();
    for (int i = 0; i < pairs.size(); ++i) {
        const btBroadphasePair& pair = pairs[i];
        const int index_a = body_index_from_proxy(pair.m_pProxy0);
        const int index_b = body_index_from_proxy(pair.m_pProxy1);
        if (disabled_model_pair_indices(world, index_a, index_b)) {
            continue;
        }
        if (!same_model_body_indices(world, index_a, index_b)) {
            continue;
        }
        if (changed_kinematic_indices.find(index_a) != changed_kinematic_indices.end() &&
            is_dynamic_body(world, index_b)) {
            wake_indices.insert(index_b);
        }
        if (changed_kinematic_indices.find(index_b) != changed_kinematic_indices.end() &&
            is_dynamic_body(world, index_a)) {
            wake_indices.insert(index_a);
        }
    }
}

void wake_related_dynamic_bodies(PmxWorld& world, const std::vector<int>& changed_kinematic_indices)
{
    if (changed_kinematic_indices.empty()) {
        return;
    }

    std::set<int> wake_indices;
    collect_joint_connected_dynamics(world, changed_kinematic_indices, wake_indices);

    const std::set<int> changed_set(changed_kinematic_indices.begin(), changed_kinematic_indices.end());
    collect_overlapping_dynamics(world, changed_set, wake_indices);

    if (!wake_indices.empty()) {
        const std::vector<int> dynamic_seeds(wake_indices.begin(), wake_indices.end());
        collect_joint_connected_dynamics(world, dynamic_seeds, wake_indices);
    }

    for (int index : wake_indices) {
        wake_dynamic_body(world, index);
    }
}

void damp_correction_velocity(btRigidBody& body, const btVector3& correction)
{
    const btScalar length2 = correction.length2();
    if (length2 <= btScalar(1.0e-12)) {
        return;
    }

    const btVector3 normal = correction / btSqrt(length2);
    const btScalar along = body.getLinearVelocity().dot(normal);
    body.setLinearVelocity(body.getLinearVelocity() - normal * along);
}

void pullback_locked_joints(PmxWorld& world)
{
    constexpr int kIterations = 2;
    constexpr btScalar kMaxError2 = btScalar(4.0);

    for (int iteration = 0; iteration < kIterations; ++iteration) {
        for (const PmxJointRuntime& runtime : world.joint_runtimes) {
            if (!runtime.locked_translation || !runtime.constraint) {
                continue;
            }
            if (runtime.rigid_a < 0 || runtime.rigid_b < 0 ||
                runtime.rigid_a >= static_cast<int>(world.bodies.size()) ||
                runtime.rigid_b >= static_cast<int>(world.bodies.size())) {
                continue;
            }

            btRigidBody& body_a = *world.bodies[runtime.rigid_a];
            btRigidBody& body_b = *world.bodies[runtime.rigid_b];
            const btTransform anchor_a = body_a.getWorldTransform() * runtime.constraint->getFrameOffsetA();
            const btTransform anchor_b = body_b.getWorldTransform() * runtime.constraint->getFrameOffsetB();
            const btVector3 error = anchor_a.getOrigin() - anchor_b.getOrigin();
            const btScalar error2 = error.length2();
            if (error2 < kLockedJointCorrectionMinError2 || error2 > kMaxError2) {
                continue;
            }

            const btVector3 correction = error * kLockedJointCorrectionFactor;

            if (is_dynamic_mode(world.body_modes[runtime.rigid_b])) {
                btTransform transform = body_b.getWorldTransform();
                transform.setOrigin(transform.getOrigin() + correction);
                set_body_transform(body_b, transform);
                damp_correction_velocity(body_b, correction);
                body_b.activate(true);
            }
            else if (is_dynamic_mode(world.body_modes[runtime.rigid_a])) {
                btTransform transform = body_a.getWorldTransform();
                transform.setOrigin(transform.getOrigin() - correction);
                set_body_transform(body_a, transform);
                damp_correction_velocity(body_a, -correction);
                body_a.activate(true);
            }
        }
    }
}

void stabilize_resting_bodies(PmxWorld& world)
{
    for (size_t i = 0; i < world.bodies.size(); ++i) {
        if (!is_dynamic_mode(world.body_modes[i])) {
            continue;
        }

        btRigidBody& body = *world.bodies[i];
        const btTransform current = body.getWorldTransform();
        if (body.getLinearVelocity().length2() >= kLinearRestVelocity2 ||
            body.getAngularVelocity().length2() >= kAngularRestVelocity2 ||
            transform_changed(current, world.last_transforms[i], kRestMove2, kRestAngle2)) {
            world.last_transforms[i] = current;
            world.resting_frames[i] = 0;
            continue;
        }

        if (world.resting_frames[i] < kRestFramesToSleep) {
            ++world.resting_frames[i];
            continue;
        }

        body.clearForces();
        body.setLinearVelocity(btVector3(0.0f, 0.0f, 0.0f));
        body.setAngularVelocity(btVector3(0.0f, 0.0f, 0.0f));
        body.setActivationState(ISLAND_SLEEPING);
    }
}

} // namespace

int pmx_bullet_api_version()
{
    return kApiVersion;
}

void* pmx_bullet_create_world()
{
    try {
        return new PmxWorld();
    }
    catch (...) {
        return nullptr;
    }
}

void pmx_bullet_destroy_world(void* world)
{
    delete static_cast<PmxWorld*>(world);
}

void pmx_bullet_set_gravity(void* world, float x, float y, float z)
{
    if (!world) {
        return;
    }
    PmxWorld& pmx_world = *static_cast<PmxWorld*>(world);
    pmx_world.dynamics_world->setGravity(btVector3(x, y, z));
    wake_dynamic_bodies(pmx_world);
}

int pmx_bullet_set_solver_iterations(void* world_ptr, int iterations)
{
    if (!world_ptr) {
        return 0;
    }
    PmxWorld& world = *static_cast<PmxWorld*>(world_ptr);
    const int clamped = std::max(1, std::min(iterations, 128));
    world.solver_iterations = clamped;
    world.dynamics_world->getSolverInfo().m_numIterations = clamped;
    for (auto& constraint : world.constraints) {
        constraint->setOverrideNumSolverIterations(clamped);
    }
    return 1;
}

int pmx_bullet_set_joint_quality(
    void* world_ptr,
    int use_frame_offset,
    float joint_stop_erp,
    float joint_stop_cfm,
    float locked_joint_stop_erp,
    float locked_joint_stop_cfm)
{
    if (!world_ptr) {
        return 0;
    }
    PmxWorld& world = *static_cast<PmxWorld*>(world_ptr);
    world.use_frame_offset = use_frame_offset != 0;
    world.joint_stop_erp = joint_stop_erp;
    world.joint_stop_cfm = joint_stop_cfm;
    world.locked_joint_stop_erp = locked_joint_stop_erp;
    world.locked_joint_stop_cfm = locked_joint_stop_cfm;

    for (auto& runtime : world.joint_runtimes) {
        if (runtime.constraint == nullptr) {
            continue;
        }
        runtime.constraint->setUseFrameOffset(world.use_frame_offset);
        apply_joint_quality(world, runtime, *runtime.constraint);
    }
    return 1;
}

int pmx_bullet_set_stabilization(
    void* world_ptr,
    int locked_joint_pullback,
    int resting_body_stabilization)
{
    if (!world_ptr) {
        return 0;
    }
    PmxWorld& world = *static_cast<PmxWorld*>(world_ptr);
    world.use_locked_joint_pullback = locked_joint_pullback != 0;
    world.use_resting_body_stabilization = resting_body_stabilization != 0;
    return 1;
}

int pmx_bullet_add_rigid_bodies(void* world_ptr, const PmxBtRigidDesc* bodies, int count)
{
    if (!world_ptr || !bodies || count < 0) {
        return 0;
    }

    PmxWorld& world = *static_cast<PmxWorld*>(world_ptr);
    for (int i = 0; i < count; ++i) {
        const PmxBtRigidDesc& desc = bodies[i];
        btCollisionShape* shape = make_shape(world, desc);
        const btTransform transform = to_transform(desc.position, desc.rotation);
        const bool dynamic = is_dynamic_mode(desc.mode);
        const btScalar mass = dynamic ? btMax(desc.mass, 0.0001f) : 0.0f;

        btVector3 inertia(0.0f, 0.0f, 0.0f);
        if (dynamic) {
            shape->calculateLocalInertia(mass, inertia);
        }

        auto motion_state = std::make_unique<btDefaultMotionState>(transform);
        btRigidBody::btRigidBodyConstructionInfo info(mass, motion_state.get(), shape, inertia);
        info.m_friction = desc.friction;
        info.m_restitution = desc.restitution;
        info.m_linearDamping = desc.linear_damping;
        info.m_angularDamping = desc.angular_damping;
        info.m_additionalDamping = true;

        auto body = std::make_unique<btRigidBody>(info);
        if (dynamic) {
            body->setSleepingThresholds(btScalar(0.0), btScalar(0.0));
            body->setActivationState(DISABLE_DEACTIVATION);
        }
        else {
            body->setActivationState(DISABLE_DEACTIVATION);
            body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
        }

        world.dynamics_world->addRigidBody(body.get(), desc.collision_group, desc.collision_mask);
        body->setUserIndex(static_cast<int>(world.bodies.size()));
        world.initial_transforms.push_back(transform);
        world.last_transforms.push_back(transform);
        world.resting_frames.push_back(0);
        world.body_modes.push_back(desc.mode);
        world.body_model_ids.push_back(-1);
        world.motion_states.push_back(std::move(motion_state));
        world.bodies.push_back(std::move(body));
    }
    return 1;
}

int pmx_bullet_set_body_model_ids(void* world_ptr, int start_body, int count, int model_index)
{
    if (!world_ptr || start_body < 0 || count < 0 || model_index < 0) {
        return 0;
    }

    PmxWorld& world = *static_cast<PmxWorld*>(world_ptr);
    if (start_body + count > static_cast<int>(world.body_model_ids.size())) {
        return 0;
    }

    for (int i = 0; i < count; ++i) {
        world.body_model_ids[start_body + i] = model_index;
    }
    refresh_world_pairs(world);
    return 1;
}

int pmx_bullet_set_disabled_model_pairs(void* world_ptr, const PmxBtModelPair* pairs, int count)
{
    if (!world_ptr || count < 0 || (count > 0 && !pairs)) {
        return 0;
    }

    PmxWorld& world = *static_cast<PmxWorld*>(world_ptr);
    world.overlap_filter.disabled_model_pairs.clear();
    for (int i = 0; i < count; ++i) {
        const int model_a = pairs[i].model_a;
        const int model_b = pairs[i].model_b;
        if (model_a < 0 || model_b < 0 || model_a == model_b) {
            return 0;
        }
        world.overlap_filter.disabled_model_pairs.insert(
            PmxOverlapFilter::ordered_model_pair(model_a, model_b));
    }
    clean_all_body_pairs(world);
    remove_disabled_model_pairs(world);
    clear_disabled_model_manifolds(world);
    refresh_world_pairs(world);
    remove_disabled_model_pairs(world);
    clear_disabled_model_manifolds(world);
    return 1;
}

int pmx_bullet_set_cross_model_body_pair_filter_enabled(void* world_ptr, int enabled)
{
    if (!world_ptr) {
        return 0;
    }

    PmxWorld& world = *static_cast<PmxWorld*>(world_ptr);
    world.overlap_filter.use_cross_model_body_pair_filter = enabled != 0;
    clean_all_body_pairs(world);
    remove_disabled_model_pairs(world);
    clear_disabled_model_manifolds(world);
    refresh_world_pairs(world);
    remove_disabled_model_pairs(world);
    clear_disabled_model_manifolds(world);
    return 1;
}

int pmx_bullet_set_enabled_cross_model_body_pairs(void* world_ptr, const PmxBtBodyPair* pairs, int count)
{
    if (!world_ptr || count < 0 || (count > 0 && !pairs)) {
        return 0;
    }

    PmxWorld& world = *static_cast<PmxWorld*>(world_ptr);
    world.overlap_filter.enabled_cross_model_body_pairs.clear();
    for (int i = 0; i < count; ++i) {
        const int body_a = pairs[i].body_a;
        const int body_b = pairs[i].body_b;
        if (body_a < 0 || body_b < 0 || body_a == body_b ||
            body_a >= static_cast<int>(world.body_model_ids.size()) ||
            body_b >= static_cast<int>(world.body_model_ids.size())) {
            return 0;
        }
        const int model_a = world.body_model_ids[body_a];
        const int model_b = world.body_model_ids[body_b];
        if (model_a < 0 || model_b < 0 || model_a == model_b) {
            continue;
        }
        world.overlap_filter.enabled_cross_model_body_pairs.insert(
            PmxOverlapFilter::ordered_index_pair(body_a, body_b));
    }
    clean_all_body_pairs(world);
    remove_disabled_model_pairs(world);
    clear_disabled_model_manifolds(world);
    refresh_world_pairs(world);
    remove_disabled_model_pairs(world);
    clear_disabled_model_manifolds(world);
    return 1;
}

int pmx_bullet_add_non_collision_pairs(void* world_ptr, const PmxBtNonCollisionPair* pairs, int count)
{
    if (!world_ptr || !pairs || count < 0) {
        return 0;
    }

    PmxWorld& world = *static_cast<PmxWorld*>(world_ptr);
    for (int i = 0; i < count; ++i) {
        const int rigid_a = pairs[i].rigid_a;
        const int rigid_b = pairs[i].rigid_b;
        if (rigid_a < 0 || rigid_b < 0 ||
            rigid_a >= static_cast<int>(world.bodies.size()) ||
            rigid_b >= static_cast<int>(world.bodies.size())) {
            return 0;
        }

        btRigidBody& body_a = *world.bodies[rigid_a];
        btRigidBody& body_b = *world.bodies[rigid_b];
        world.overlap_filter.add(body_a.getBroadphaseHandle(), body_b.getBroadphaseHandle());
        clean_body_pairs(world, body_a);
        clean_body_pairs(world, body_b);
    }
    return 1;
}

int pmx_bullet_add_joints(void* world_ptr, const PmxBtJointDesc* joints, int count)
{
    if (!world_ptr || !joints || count < 0) {
        return 0;
    }

    PmxWorld& world = *static_cast<PmxWorld*>(world_ptr);
    for (int i = 0; i < count; ++i) {
        const PmxBtJointDesc& desc = joints[i];
        if (desc.rigid_a < 0 || desc.rigid_b < 0 ||
            desc.rigid_a >= static_cast<int>(world.bodies.size()) ||
            desc.rigid_b >= static_cast<int>(world.bodies.size())) {
            return 0;
        }

        btRigidBody& body_a = *world.bodies[desc.rigid_a];
        btRigidBody& body_b = *world.bodies[desc.rigid_b];
        const btTransform joint_world = to_transform(desc.position, desc.rotation);
        const btTransform frame_a = body_a.getWorldTransform().inverse() * joint_world;
        const btTransform frame_b = body_b.getWorldTransform().inverse() * joint_world;

        const bool locked_translation = is_locked_translation_joint(desc);
        PmxJointRuntime runtime;
        runtime.rigid_a = desc.rigid_a;
        runtime.rigid_b = desc.rigid_b;
        runtime.locked_translation = locked_translation;
        runtime.joint_stop_erp = desc.joint_stop_erp;
        runtime.joint_stop_cfm = desc.joint_stop_cfm;
        runtime.locked_joint_stop_erp = desc.locked_joint_stop_erp;
        runtime.locked_joint_stop_cfm = desc.locked_joint_stop_cfm;

        auto constraint = std::make_unique<btGeneric6DofSpringConstraint>(body_a, body_b, frame_a, frame_b, true);
        constraint->setUseFrameOffset(world.use_frame_offset);
        constraint->setLinearLowerLimit(to_vec3(desc.linear_lower));
        constraint->setLinearUpperLimit(to_vec3(desc.linear_upper));
        constraint->setAngularLowerLimit(to_vec3(desc.angular_lower));
        constraint->setAngularUpperLimit(to_vec3(desc.angular_upper));
        apply_joint_quality(world, runtime, *constraint);

        for (int axis = 0; axis < 3; ++axis) {
            if (desc.spring_linear[axis] > 0.0f) {
                constraint->enableSpring(axis, true);
                constraint->setStiffness(axis, desc.spring_linear[axis]);
                constraint->setDamping(axis, desc.spring_linear_damping[axis]);
            }
            if (desc.spring_angular[axis] > 0.0f) {
                const int angular_axis = axis + 3;
                constraint->enableSpring(angular_axis, true);
                constraint->setStiffness(angular_axis, desc.spring_angular[axis]);
                constraint->setDamping(angular_axis, desc.spring_angular_damping[axis]);
            }
        }

        constraint->setOverrideNumSolverIterations(world.solver_iterations);
        constraint->setEquilibriumPoint();
        world.dynamics_world->addConstraint(constraint.get(), desc.disable_collisions != 0);
        runtime.constraint = constraint.get();
        world.joint_runtimes.push_back(runtime);
        world.constraints.push_back(std::move(constraint));
    }
    return 1;
}

int pmx_bullet_temporal_kinematic_init(void* world_ptr, const PmxBtBodyTransform* transforms, int count)
{
    if (!world_ptr || !transforms || count < 0) {
        return 0;
    }

    PmxWorld& world = *static_cast<PmxWorld*>(world_ptr);
    std::vector<int> old_flags(world.bodies.size());
    for (size_t i = 0; i < world.bodies.size(); ++i) {
        btRigidBody& body = *world.bodies[i];
        old_flags[i] = body.getCollisionFlags();
        body.setCollisionFlags(old_flags[i] | btCollisionObject::CF_KINEMATIC_OBJECT);
        refresh_body_proxy(world, body);
    }
    for (int i = 0; i < count; ++i) {
        if (!set_kinematic_transform(world, transforms[i])) {
            return 0;
        }
    }
    refresh_world_pairs(world);
    world.dynamics_world->stepSimulation(0.0f, 0, 1.0f / 60.0f);
    for (size_t i = 0; i < world.bodies.size(); ++i) {
        world.bodies[i]->setCollisionFlags(old_flags[i]);
        world.bodies[i]->clearForces();
        world.bodies[i]->setLinearVelocity(btVector3(0.0f, 0.0f, 0.0f));
        world.bodies[i]->setAngularVelocity(btVector3(0.0f, 0.0f, 0.0f));
        refresh_body_proxy(world, *world.bodies[i]);
        reset_rest_state(world, static_cast<int>(i), world.bodies[i]->getWorldTransform());
        if (is_dynamic_mode(world.body_modes[i])) {
            world.bodies[i]->forceActivationState(ACTIVE_TAG);
            world.bodies[i]->activate(true);
        }
    }
    refresh_world_pairs(world);
    return 1;
}

int pmx_bullet_set_kinematic_transforms(void* world_ptr, const PmxBtBodyTransform* transforms, int count)
{
    if (!world_ptr || !transforms || count < 0) {
        return 0;
    }

    PmxWorld& world = *static_cast<PmxWorld*>(world_ptr);
    std::vector<int> changed_kinematic_indices;
    for (int i = 0; i < count; ++i) {
        bool changed = true;
        const int body_index = transforms[i].index;
        if (body_index >= 0 && body_index < static_cast<int>(world.bodies.size()) &&
            !is_dynamic_mode(world.body_modes[body_index])) {
            const btTransform next = to_transform(transforms[i].position, transforms[i].rotation);
            changed = transform_changed(
                next,
                world.bodies[body_index]->getWorldTransform(),
                kKinematicWakeMove2,
                kKinematicWakeAngle2);
            if (changed) {
                changed_kinematic_indices.push_back(body_index);
            }
        }
        if (!set_kinematic_transform(world, transforms[i], changed)) {
            return 0;
        }
    }
    if (!changed_kinematic_indices.empty()) {
        refresh_world_pairs(world);
        wake_related_dynamic_bodies(world, changed_kinematic_indices);
    }
    return 1;
}

int pmx_bullet_freeze_body_transforms(void* world_ptr, const PmxBtBodyTransform* transforms, int count)
{
    if (!world_ptr || !transforms || count < 0) {
        return 0;
    }

    PmxWorld& world = *static_cast<PmxWorld*>(world_ptr);
    for (int i = 0; i < count; ++i) {
        const int body_index = transforms[i].index;
        if (!valid_body_index(world, body_index)) {
            return 0;
        }
        btRigidBody& body = *world.bodies[body_index];
        const btTransform transform = to_transform(transforms[i].position, transforms[i].rotation);
        set_body_transform(body, transform);
        body.clearForces();
        body.setLinearVelocity(btVector3(0.0f, 0.0f, 0.0f));
        body.setAngularVelocity(btVector3(0.0f, 0.0f, 0.0f));
        refresh_body_proxy(world, body);
        reset_rest_state(world, body_index, transform);
        if (is_dynamic_mode(world.body_modes[body_index])) {
            body.forceActivationState(ISLAND_SLEEPING);
        }
    }
    refresh_world_pairs(world);
    return 1;
}

int pmx_bullet_step(void* world_ptr, float fixed_timestep, int max_substeps)
{
    if (!world_ptr || fixed_timestep <= 0.0f || max_substeps < 1) {
        return 0;
    }

    PmxWorld& world = *static_cast<PmxWorld*>(world_ptr);
    remove_disabled_model_pairs(world);
    clear_disabled_model_manifolds(world);
    world.dynamics_world->stepSimulation(fixed_timestep, max_substeps, fixed_timestep);
    remove_disabled_model_pairs(world);
    clear_disabled_model_manifolds(world);
    if (world.use_locked_joint_pullback) {
        pullback_locked_joints(world);
        refresh_world_pairs(world);
        remove_disabled_model_pairs(world);
        clear_disabled_model_manifolds(world);
    }
    if (world.use_resting_body_stabilization) {
        stabilize_resting_bodies(world);
    }
    return 1;
}

int pmx_bullet_prewarm(void* world_ptr, int steps, float timestep)
{
    if (!world_ptr || steps < 0 || timestep <= 0.0f) {
        return 0;
    }

    PmxWorld& world = *static_cast<PmxWorld*>(world_ptr);
    for (int i = 0; i < steps; ++i) {
        world.dynamics_world->stepSimulation(timestep, 1, timestep);
        if (world.use_locked_joint_pullback) {
            pullback_locked_joints(world);
            refresh_world_pairs(world);
        }
    }
    for (size_t i = 0; i < world.bodies.size(); ++i) {
        btRigidBody& body = *world.bodies[i];
        body.clearForces();
        body.setLinearVelocity(btVector3(0.0f, 0.0f, 0.0f));
        body.setAngularVelocity(btVector3(0.0f, 0.0f, 0.0f));
        reset_rest_state(world, static_cast<int>(i), body.getWorldTransform());
        if (is_dynamic_mode(world.body_modes[i])) {
            body.forceActivationState(ACTIVE_TAG);
            body.activate(true);
        }
    }
    return 1;
}

int pmx_bullet_get_body_transforms(void* world_ptr, PmxBtBodyTransform* transforms, int count)
{
    if (!world_ptr || !transforms || count < 0) {
        return 0;
    }

    PmxWorld& world = *static_cast<PmxWorld*>(world_ptr);
    const int body_count = static_cast<int>(world.bodies.size());
    const int write_count = btMin(count, body_count);

    for (int i = 0; i < write_count; ++i) {
        btTransform transform;
        world.bodies[i]->getMotionState()->getWorldTransform(transform);
        const btVector3 origin = transform.getOrigin();
        const btQuaternion rotation = transform.getRotation();
        transforms[i].index = i;
        transforms[i].position[0] = origin.x();
        transforms[i].position[1] = origin.y();
        transforms[i].position[2] = origin.z();
        transforms[i].rotation[0] = rotation.x();
        transforms[i].rotation[1] = rotation.y();
        transforms[i].rotation[2] = rotation.z();
        transforms[i].rotation[3] = rotation.w();
    }
    return 1;
}

int pmx_bullet_reset(void* world_ptr)
{
    if (!world_ptr) {
        return 0;
    }

    PmxWorld& world = *static_cast<PmxWorld*>(world_ptr);
    for (size_t i = 0; i < world.bodies.size(); ++i) {
        btRigidBody& body = *world.bodies[i];
        set_body_transform(body, world.initial_transforms[i]);
        body.clearForces();
        body.setLinearVelocity(btVector3(0.0f, 0.0f, 0.0f));
        body.setAngularVelocity(btVector3(0.0f, 0.0f, 0.0f));
        refresh_body_proxy(world, body);
        reset_rest_state(world, static_cast<int>(i), world.initial_transforms[i]);
        if (is_dynamic_mode(world.body_modes[i])) {
            body.forceActivationState(ACTIVE_TAG);
            body.activate(true);
        }
    }
    refresh_world_pairs(world);
    return 1;
}
