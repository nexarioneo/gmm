#pragma once

#include <godot_cpp/variant/color.hpp>
#include <godot_cpp/variant/quaternion.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include "../debug/MotionMatchingCompat.h"

namespace godot::MotionMatching {

void DebugDrawVelocity(DebugDrawOutput &debugDisplay,
	const Vector3 &jointPosition,
	const Vector3 &velocity,
	const Color &color);

Vector3 CalculateLinearVelocity(const Vector3 &prevPosition,
	const Vector3 &currentPosition,
	float deltaTime);

Vector3 CalculateAngularVelocity(const Quaternion &prevRotation,
	const Quaternion &currentRotation,
	float deltaTime);

}
