#pragma once
#include <cstdint>
#include "engine/common/vec.hpp"
#include "engine/common/grid.hpp"

struct CollisionCell{
	static constexpr uint8_t cell_capacity = 4;
	static constexpr uint8_t max_idx = cell_capacity - 1;

	uint32_t object_count = 0;
	uint32_t object_ids[cell_capacity] = {};

	CollisionCell() = default;

	// Adds object to cell
	// I genuinely don't understand the logic of this code
	void addObject(uint32_t id){
		object_ids[object_count] = id;
		object_count += object_count < max_idx;
	}

	//clears the cell
	void clear(){
		object_count = 0u;
	}

	// void remove(uint32_t id) {
	// 	for (uint32_t i{0}; i < object_count; i++){
	// 		if (objects[i] == id) {
	// 			objects[i] = objects[object_count - 1];
	// 			--object_count;
	// 			return;
	// 		}
	// 	}
	// }
};

struct CollisionGrid : public Grid<CollisionCell>
{
	CollisionGrid(int32_t width, int32_t height)
		: Grid<CollisionCell>(width, height)
	{}

	bool addObject(uint32_t x, uint32_t y, uint32_t atom)
	{
		const uint32_t id = x * height + y;
		// Add to grid
		data[id].addObject(atom);
		return true;
	}

	void clear()
	{
		for (auto& c : data) {
            c.clear();
        }
	}
};
