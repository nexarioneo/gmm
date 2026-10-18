#include "core/math/query_vector.h"

#include <godot_cpp/core/class_db.hpp>

namespace godot::MotionMatching {

void QueryVector::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_size"), &QueryVector::get_size);
	ClassDB::bind_method(D_METHOD("resize", "new_size"), &QueryVector::resize);
	ClassDB::bind_method(D_METHOD("set_vector2", "value", "offset"), &QueryVector::set_vector2);
	ClassDB::bind_method(D_METHOD("set_vector3", "value", "offset"), &QueryVector::set_vector3);
	ClassDB::bind_method(D_METHOD("get_vector2", "offset"), &QueryVector::get_vector2);
	ClassDB::bind_method(D_METHOD("get_vector3", "offset"), &QueryVector::get_vector3);
	ClassDB::bind_method(D_METHOD("get_data"), &QueryVector::get_data);
	ClassDB::bind_method(D_METHOD("set_data", "data"), &QueryVector::set_data);
	ClassDB::bind_method(D_METHOD("get_element", "offset"), &QueryVector::get_element);
	ClassDB::bind_method(D_METHOD("set_element", "offset", "value"), &QueryVector::set_element);
}

void QueryVector::SetVector2(const Vector2 &value, size_t offset) {
	if (offset + 2 > m_data.size()) {
		return;
	}
	m_data[offset + 0] = value.x;
	m_data[offset + 1] = value.y;
}

void QueryVector::SetVector3(const Vector3 &value, size_t offset) {
	if (offset + 3 > m_data.size()) {
		return;
	}
	m_data[offset + 0] = value.x;
	m_data[offset + 1] = value.y;
	m_data[offset + 2] = value.z;
}

Vector2 QueryVector::GetVector2(size_t offset) const {
	if (offset + 2 > m_data.size()) {
		return Vector2();
	}
	return Vector2(m_data[offset + 0], m_data[offset + 1]);
}

Vector3 QueryVector::GetVector3(size_t offset) const {
	if (offset + 3 > m_data.size()) {
		return Vector3();
	}
	return Vector3(m_data[offset + 0], m_data[offset + 1], m_data[offset + 2]);
}

PackedFloat32Array QueryVector::get_data() const {
	PackedFloat32Array out;
	out.resize((int64_t)m_data.size());
	for (size_t i = 0; i < m_data.size(); ++i) {
		out[(int64_t)i] = m_data[i];
	}
	return out;
}

void QueryVector::set_data(const PackedFloat32Array &p_data) {
	m_data.resize(p_data.size());
	for (int64_t i = 0; i < p_data.size(); ++i) {
		m_data[(size_t)i] = p_data[i];
	}
}

float QueryVector::get_element(int64_t offset) const {
	if (offset < 0 || static_cast<size_t>(offset) >= m_data.size()) {
		return 0.0f;
	}
	return m_data[static_cast<size_t>(offset)];
}

void QueryVector::set_element(int64_t offset, float value) {
	if (offset < 0 || static_cast<size_t>(offset) >= m_data.size()) {
		return;
	}
	m_data[static_cast<size_t>(offset)] = value;
}

int64_t QueryVector::get_size() const {
	return static_cast<int64_t>(GetSize());
}

void QueryVector::resize(int64_t p_new_size) {
	if (p_new_size < 0) {
		Resize(0);
		return;
	}
	Resize(static_cast<size_t>(p_new_size));
}

void QueryVector::set_vector2(const Vector2 &value, int64_t offset) {
	if (offset < 0 || static_cast<size_t>(offset) + 2 > m_data.size()) {
		return;
	}
	SetVector2(value, static_cast<size_t>(offset));
}

void QueryVector::set_vector3(const Vector3 &value, int64_t offset) {
	if (offset < 0 || static_cast<size_t>(offset) + 3 > m_data.size()) {
		return;
	}
	SetVector3(value, static_cast<size_t>(offset));
}

Vector2 QueryVector::get_vector2(int64_t offset) const {
	if (offset < 0 || static_cast<size_t>(offset) + 2 > m_data.size()) {
		return Vector2();
	}
	return GetVector2(static_cast<size_t>(offset));
}

Vector3 QueryVector::get_vector3(int64_t offset) const {
	if (offset < 0 || static_cast<size_t>(offset) + 3 > m_data.size()) {
		return Vector3();
	}
	return GetVector3(static_cast<size_t>(offset));
}

}
