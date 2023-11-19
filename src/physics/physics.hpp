#pragma once
#include "collision_grid.hpp"
#include "physic_object.hpp"
#include "engine/common/utils.hpp"
#include "engine/common/index_vector.hpp"
#include "thread_pool/thread_pool.hpp"


struct PhysicSolver
{
    CIVector<PhysicObject> objects;
    CollisionGrid          grid;
    Vec2                   world_size;
    Vec2                   gravity = {0.0f, 20.0f};

    // Simulation solving pass count
    float           response_coef = 1.0f;
    float           eps = 0.0001f;
    float           radius = 1.0f;
    uint32_t        sub_steps;

    PhysicSolver(IVec2 size)
        : grid{size.x, size.y}
        , world_size{to<float>(size.x), to<float>(size.y)}
        , sub_steps{8}
    {}

    // Add a new object to the solver
    uint64_t createObject(Vec2 pos)
    {
        return objects.emplace_back(pos);
    }

    // Checks if two atoms are colliding and if so create a new contact
    void solveContact(PhysicObject& obj_1, PhysicObject& obj_2)
    {
        const Vec2 o2_o1  = obj_1.position - obj_2.position;
        const float dist2 = o2_o1.x * o2_o1.x + o2_o1.y * o2_o1.y;
        if (dist2 < 1.0f && dist2 > eps) {
            const float dist          = sqrt(dist2);
            // Radius are all equal to 1.0f
            const float delta  = response_coef * 0.5f * (1.0f - dist);
            const Vec2 col_vec = (o2_o1 / dist) * delta;
            obj_1.position += col_vec;
            obj_2.position -= col_vec;
        }
    }

    void find_collisions() {
        for (auto& obj_1 : objects) {
            for (auto& obj_2 : objects) {
                if (&obj_1 != &obj_2){
                    solveContact(obj_1, obj_2);
                }
            }
        }
    }

    void update_physics(float dt){
        for (auto& obj : objects) {
            // Add gravity
            obj.acceleration += gravity;
            // Apply Verlet integration
            obj.update(dt);
            // Apply map borders collisions
            const float margin = 2.0f;
            if (obj.position.x > world_size.x - margin) {
                obj.position.x = world_size.x - margin;
            } else if (obj.position.x < margin) {
                obj.position.x = margin;
            }
            if (obj.position.y > world_size.y - margin) {
                obj.position.y = world_size.y - margin;
            } else if (obj.position.y < margin) {
                obj.position.y = margin;
            }
        }

    }

    void update_physics_substeps(float dt){
        const float sub_dt = dt / float(sub_steps);
        for (uint32_t i{sub_steps}; i--;){
            update_physics(sub_dt);
        }
    }
};
