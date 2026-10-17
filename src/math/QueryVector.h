#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/packed_float32_array.hpp>
#include <godot_cpp/variant/vector2.hpp>
#include <godot_cpp/variant/vector3.hpp>

namespace godot {
namespace MotionMatching {

class QueryVector : public RefCounted {
	GDCLASS(QueryVector, RefCounted)

private:
	std::vector<float> m_data;

protected:
	static void _bind_methods();

public:
	virtual ~QueryVector() = default;

	size_t GetSize() const { return m_data.size(); }
	void Resize(size_t newSize) { m_data.resize(newSize); }

	void SetVector2(const Vector2 &value, size_t offset);
	void SetVector3(const Vector3 &value, size_t offset);

	Vector2 GetVector2(size_t offset) const;
	Vector3 GetVector3(size_t offset) const;

	std::vector<float> &GetData() { return m_data; }
	const std::vector<float> &GetData() const { return m_data; }

	PackedFloat32Array get_data() const;
	void set_data(const PackedFloat32Array &p_data);
	float get_element(int64_t offset) const;
	void set_element(int64_t offset, float value);

	int64_t get_size() const;
	void resize(int64_t p_new_size);
	void set_vector2(const Vector2 &value, int64_t offset);
	void set_vector3(const Vector3 &value, int64_t offset);
	Vector2 get_vector2(int64_t offset) const;
	Vector3 get_vector3(int64_t offset) const;
};

}
}
