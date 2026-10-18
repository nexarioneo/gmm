#pragma once

#include <godot_cpp/variant/quaternion.hpp>
#include <godot_cpp/variant/vector3.hpp>

class Transform {
public:
	godot::Vector3 m_position;
	godot::Quaternion m_rotation;

	Transform() = default;
	Transform(const godot::Vector3 &position, const godot::Quaternion &rotation)
		: m_position(position)
		, m_rotation(rotation) {
	}
	Transform(const Transform &) = default;
	Transform &operator=(const Transform &) = default;
	~Transform() = default;

	static Transform CreateIdentity() { return Transform(); }

	void Identity() {
		m_position = godot::Vector3();
		m_rotation = godot::Quaternion();
	}

	Transform Inversed() const {
		const godot::Quaternion invRotation = m_rotation.inverse();
		Transform result;
		result.m_rotation = invRotation;
		result.m_position = invRotation.xform(-m_position);
		return result;
	}

	godot::Vector3 TransformPoint(const godot::Vector3 &point) const {
		return m_rotation.xform(point) + m_position;
	}

	godot::Vector3 TransformVector(const godot::Vector3 &vector) const {
		return m_rotation.xform(vector);
	}

	void Blend(const Transform &destTransform, float weight) {
		m_position = m_position.lerp(destTransform.m_position, weight);
		m_rotation = m_rotation.slerp(destTransform.m_rotation, weight);
	}
};