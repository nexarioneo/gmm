#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>

#include <godot_cpp/variant/color.hpp>
#include <godot_cpp/variant/packed_color_array.hpp>
#include <godot_cpp/variant/packed_float32_array.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/vector2.hpp>
#include <godot_cpp/variant/vector3.hpp>

namespace godot::MotionMatching {

inline Vector3 NormalizedSafe(const Vector3 &v, float tol = 0.001f) {
	return v.length() <= tol ? Vector3() : v / v.length();
}

inline Vector2 NormalizedSafe(const Vector2 &v, float tol = 0.001f) {
	return v.length() <= tol ? Vector2() : v / v.length();
}

using FeatureId = std::string;

static constexpr size_t InvalidIndex = std::numeric_limits<size_t>::max();

static constexpr uint32_t InvalidIndex32 = std::numeric_limits<uint32_t>::max();

struct DebugDrawOutput {
	PackedVector3Array line_points;
	PackedColorArray line_colors;
	PackedVector3Array ball_centers;
	PackedFloat32Array ball_sizes;
	PackedColorArray ball_colors;
	PackedVector3Array cylinder_points;
	PackedFloat32Array cylinder_radii;
	PackedColorArray cylinder_colors;
	String overlay_text;

	void clear() {
		line_points.clear();
		line_colors.clear();
		ball_centers.clear();
		ball_sizes.clear();
		ball_colors.clear();
		cylinder_points.clear();
		cylinder_radii.clear();
		cylinder_colors.clear();
		overlay_text = String();
	}
};

}
