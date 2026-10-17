#include "Velocity.h"

#include <cmath>

namespace godot::MotionMatching {

void DebugDrawVelocity(DebugDrawOutput &debugDisplay,
	const Vector3 &jointPosition,
	const Vector3 &velocity,
	const Color &color) {
	debugDisplay.line_points.push_back(jointPosition);
	debugDisplay.line_points.push_back(jointPosition + velocity);
	debugDisplay.line_colors.push_back(color);
	debugDisplay.line_colors.push_back(color);
}

Vector3 CalculateLinearVelocity(const Vector3 &prevPosition,
	const Vector3 &currentPosition,
	float deltaTime) {
	if (deltaTime <= 1e-6f) {
		return Vector3();
	}
	return (currentPosition - prevPosition) / deltaTime;
}

Vector3 CalculateAngularVelocity(const Quaternion &prevRotation,
	const Quaternion &currentRotation,
	float deltaTime) {
	if (deltaTime <= 1e-6f) {
		return Vector3();
	}
	Quaternion delta = (currentRotation * prevRotation.inverse()).normalized();
	if (delta.w < 0.0f) {
		delta = -delta;
	}
	float w = delta.w < -1.0f ? -1.0f : (delta.w > 1.0f ? 1.0f : delta.w);
	const float angle = 2.0f * std::acos(w);
	if (angle <= 1e-6f) {
		return Vector3();
	}
	const float sinHalf = std::sqrt(1.0f - w * w);
	Vector3 axis(delta.x, delta.y, delta.z);
	if (sinHalf > 1e-6f) {
		axis /= sinHalf;
	} else {
		axis = Vector3(1.0f, 0.0f, 0.0f);
	}
	return axis * (angle / deltaTime);
}

}
