#pragma once
#include "collision_grid.hpp"
#include "physic_object.hpp"
#include "engine/common/utils.hpp"
#include "engine/common/index_vector.hpp"
#include "thread_pool/thread_pool.hpp"


struct PhysicSolver
{
    CIVector<PhysicObject> objects;
    CollisionGrid grid;
    Vec2 world_size;
    Vec2 gravity = {0.0f, 20.0f};

    // Simulation solving pass count
    uint32_t        sub_steps;
    tp::ThreadPool& thread_pool;

    // Initialize the physics solver
    PhysicSolver(IVec2 size, tp::ThreadPool& pool) :
        grid{size.x, size.y},
        world_size{to<float>(size.x), to<float>(size.y)},
        sub_steps{8},
        thread_pool{pool}
        {grid.clear();}

    void solveCellCollisions(uint32_t atom_idx, const CollisionCell& c){
        constexpr float response_coef = 1.0f;
        constexpr float eps           = 0.0001f;
        for (uint32_t i{0}; i < c.object_count; ++i) {
            PhysicObject& obj_1 = objects.data[atom_idx];
            PhysicObject& obj_2 = objects.data[c.object_ids[i]];
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
    }

    void solveCollisionThreaded(uint32_t start, uint32_t end)
    {
        for (uint32_t idx{start}; idx < end; ++idx) {
            for (uint32_t i{0}; i < grid.data[idx].object_count; ++i) {
                const uint32_t atom_idx = grid.data[idx].object_ids[i];
                solveCellCollisions(atom_idx, grid.data[idx - 1]);
                solveCellCollisions(atom_idx, grid.data[idx]);
                solveCellCollisions(atom_idx, grid.data[idx + 1]);
                solveCellCollisions(atom_idx, grid.data[idx + grid.height - 1]);
                solveCellCollisions(atom_idx, grid.data[idx + grid.height    ]);
                solveCellCollisions(atom_idx, grid.data[idx + grid.height + 1]);
                solveCellCollisions(atom_idx, grid.data[idx - grid.height - 1]);
                solveCellCollisions(atom_idx, grid.data[idx - grid.height    ]);
                solveCellCollisions(atom_idx, grid.data[idx - grid.height + 1]);
            }
        }
    }

    void solveCollisions() {
        // setting up varaibles for the grid
        const uint32_t thread_count = thread_pool.m_thread_count;
        const uint32_t num_slices = thread_count * 2;
        const uint32_t slice_size = (grid.width / num_slices) * grid.height;
        const uint32_t start_remainder = (2 * (thread_count - 1) + 2) * slice_size;
        // Finds collisions in two passes to avoid data races

        // First pass (Even Slices)
        for (uint32_t i{0}; i < thread_count; ++i) {
            thread_pool.addTask([this, i, slice_size]{
                // Defines The area the thread will process
                uint32_t const start{2 * i * slice_size};
                uint32_t const end  {start + slice_size};
                solveCollisionThreaded(start, end);
            });
        }
        // Then process the remaining area
        if (start_remainder < grid.data.size()) {
            // same process as above, but the start is the first cell of the remaining cells that did not cleanly divide into the threads
            thread_pool.addTask([this, start_remainder]{
                solveCollisionThreaded(start_remainder, to<uint32_t>(grid.data.size()));
            });
        }
        // synchronize the threads
        thread_pool.waitForCompletion();

        // Second pass (Odd Slices)
        for (uint32_t i{0}; i < thread_count; ++i) {
            thread_pool.addTask([this, i, slice_size]{
                // Defines The area the thread will process
                uint32_t const start{(2 * i + 1) * slice_size};
                uint32_t const end  {start + slice_size};
                solveCollisionThreaded(start, end);
            });
        }
        thread_pool.waitForCompletion();

    }

    // Add a new object to the solver
    uint64_t addObject(const PhysicObject& object)
    {
        return objects.push_back(object);
    }

    // Add a new object to the solver
    uint64_t createObject(Vec2 pos)
    {
        return objects.emplace_back(pos);
    }

    void update(float dt)
    {
        // Perform the sub steps
        const float sub_dt = dt / static_cast<float>(sub_steps);
        for (uint32_t i(sub_steps); i--;) {
            addGridObjects();
            solveCollisions();
            updateObjects_multi(sub_dt);
        }
    }

    void addGridObjects() {
        grid.clear();
        uint32_t overlap_spacing = 0;
        for (const PhysicObject& obj : objects.data) {
            if (obj.position.x > 1.0f && obj.position.x < world_size.x - 1.0f &&
                obj.position.y > 1.0f && obj.position.y < world_size.y - 1.0f) {
                    grid.addObject(to<int32_t>(obj.position.x), to<int32_t>(obj.position.y), overlap_spacing);
                }
            overlap_spacing++;
        }
    }

    void updateObjects_multi(float dt)
    {
        thread_pool.dispatch(to<uint32_t>(objects.size()), [&](uint32_t start, uint32_t end){
            for (uint32_t i{start}; i < end; ++i) {
                PhysicObject& obj = objects.data[i];
                // Apply acceleration
                obj.acceleration += gravity;
                // Update object with physics package
                obj.update(dt);
                // Border Collisions
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
        });
    }
};
