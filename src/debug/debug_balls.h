#pragma once

#include <cstdint>

#include <godot_cpp/classes/ref.hpp>
#include <godot_cpp/classes/sphere_mesh.hpp>
#include <godot_cpp/variant/packed_color_array.hpp>
#include <godot_cpp/variant/packed_float32_array.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>
#include <godot_cpp/variant/rid.hpp>

namespace godot::MotionMatching {

class MotionMatchingDebugBalls {
public:

	static constexpr int32_t MAX_BALLS = 4096;

	MotionMatchingDebugBalls() = default;
	~MotionMatchingDebugBalls() { dispose(); }

	MotionMatchingDebugBalls(const MotionMatchingDebugBalls &) = delete;
	MotionMatchingDebugBalls &operator=(const MotionMatchingDebugBalls &) = delete;
	MotionMatchingDebugBalls(MotionMatchingDebugBalls &&) = default;
	MotionMatchingDebugBalls &operator=(MotionMatchingDebugBalls &&) = default;

	void ensure(const RID &p_scenario);

	void set_material(const RID &p_material);

	void draw_balls(const PackedVector3Array &p_centers,
			const PackedFloat32Array &p_sizes,
			const PackedColorArray &p_colors);

	void hide();

	void dispose();

	bool is_ready() const { return m_built; }
	int32_t capacity() const { return m_capacity; }

private:
	void allocate_locked(int32_t p_want);

	static constexpr int32_t kInitialCapacity = 64;
	static constexpr int32_t kSphereRadialSegments = 12;
	static constexpr int32_t kSphereRings = 6;

	Ref<SphereMesh> m_sphere;
	RID m_mesh_rid;
	RID m_multimesh;
	RID m_instance;
	RID m_scenario;
	RID m_material_rid;
	int32_t m_capacity = 0;
	bool m_built = false;
};

}
