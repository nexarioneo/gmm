#pragma once

#include <godot_cpp/classes/mesh.hpp>
#include <godot_cpp/variant/packed_color_array.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>
#include <godot_cpp/variant/rid.hpp>

namespace godot::MotionMatching {

class MotionMatchingDebugLines {
public:
	MotionMatchingDebugLines() = default;
	~MotionMatchingDebugLines() = default;

	MotionMatchingDebugLines(const MotionMatchingDebugLines &) = delete;
	MotionMatchingDebugLines &operator=(const MotionMatchingDebugLines &) = delete;

	void draw_lines(const PackedVector3Array &points, const PackedColorArray &colors, RID line_mesh, RID line_instance);
};

}
