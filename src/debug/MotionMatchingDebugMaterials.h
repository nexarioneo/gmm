#pragma once

#include <godot_cpp/classes/ref.hpp>
#include <godot_cpp/variant/rid.hpp>

namespace godot {
class StandardMaterial3D;
}

namespace godot::MotionMatching {

class MotionMatchingDebugMaterials {
public:
	MotionMatchingDebugMaterials() = default;
	~MotionMatchingDebugMaterials() = default;

	MotionMatchingDebugMaterials(const MotionMatchingDebugMaterials &) = delete;
	MotionMatchingDebugMaterials &operator=(const MotionMatchingDebugMaterials &) = delete;

	void ensure();

	void dispose();

	bool is_ready() const;

	RID get_line_material() const;
	RID get_ball_material() const;
	RID get_cylinder_material() const;

private:
	Ref<StandardMaterial3D> line_material;
	Ref<StandardMaterial3D> ball_material;
	Ref<StandardMaterial3D> cylinder_material;
};

}
