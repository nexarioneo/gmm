#pragma once

#include <cstdint>

#include <godot_cpp/classes/cylinder_mesh.hpp>
#include <godot_cpp/classes/ref.hpp>
#include <godot_cpp/variant/packed_color_array.hpp>
#include <godot_cpp/variant/packed_float32_array.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>
#include <godot_cpp/variant/rid.hpp>

namespace godot::MotionMatching {

class MotionMatchingDebugCylinders {
public:

	static constexpr int32_t MAX_CYLINDERS = 4096;

	MotionMatchingDebugCylinders() = default;
	~MotionMatchingDebugCylinders() { dispose(); }

	MotionMatchingDebugCylinders(const MotionMatchingDebugCylinders &) = delete;
	MotionMatchingDebugCylinders &operator=(const MotionMatchingDebugCylinders &) = delete;
	MotionMatchingDebugCylinders(MotionMatchingDebugCylinders &&) = default;
	MotionMatchingDebugCylinders &operator=(MotionMatchingDebugCylinders &&) = default;

	void ensure(const RID &p_scenario);

	void set_material(const RID &p_material);

	void draw_cylinders(const PackedVector3Array &p_points,
			const PackedFloat32Array &p_radii,
			const PackedColorArray &p_colors);

	void hide();

	void dispose();

	bool is_ready() const { return m_built; }
	int32_t capacity() const { return m_capacity; }

private:
	void allocate_locked(int32_t p_want);

	static constexpr int32_t kInitialCapacity = 64;
	static constexpr int32_t kRadialSegments = 10;
	static constexpr int32_t kRings = 1;

	static constexpr float kEps = 1e-6f;

	Ref<CylinderMesh> m_cylinder;
	RID m_mesh_rid;
	RID m_multimesh;
	RID m_instance;
	RID m_scenario;
	RID m_material_rid;
	int32_t m_capacity = 0;
	bool m_built = false;
};

}
