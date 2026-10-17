#include "MotionMatchingDebugMaterials.h"

#include <godot_cpp/classes/base_material3d.hpp>
#include <godot_cpp/classes/material.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>
#include <godot_cpp/variant/color.hpp>

namespace godot::MotionMatching {

namespace {

Ref<StandardMaterial3D> MakeDebugMaterial() {
	Ref<StandardMaterial3D> mat;
	mat.instantiate();
	mat->set_albedo(Color(1.0f, 1.0f, 1.0f, 1.0f));
	mat->set_shading_mode(BaseMaterial3D::SHADING_MODE_UNSHADED);
	mat->set_flag(BaseMaterial3D::FLAG_ALBEDO_FROM_VERTEX_COLOR, true);
	mat->set_transparency(BaseMaterial3D::TRANSPARENCY_ALPHA);
	mat->set_flag(BaseMaterial3D::FLAG_DISABLE_DEPTH_TEST, true);
	mat->set_render_priority(Material::RENDER_PRIORITY_MAX);
	mat->set_cull_mode(BaseMaterial3D::CULL_DISABLED);
	return mat;
}

}

void MotionMatchingDebugMaterials::ensure() {
	if (line_material.is_valid() && ball_material.is_valid() && cylinder_material.is_valid()) {
		return;
	}
	if (line_material.is_null()) {
		line_material = MakeDebugMaterial();
	}
	if (ball_material.is_null()) {
		ball_material = MakeDebugMaterial();
	}
	if (cylinder_material.is_null()) {
		cylinder_material = MakeDebugMaterial();
	}
}

void MotionMatchingDebugMaterials::dispose() {

	line_material.unref();
	ball_material.unref();
	cylinder_material.unref();
}

bool MotionMatchingDebugMaterials::is_ready() const {
	return line_material.is_valid() && ball_material.is_valid() && cylinder_material.is_valid();
}

RID MotionMatchingDebugMaterials::get_line_material() const {
	return line_material.is_valid() ? line_material->get_rid() : RID();
}

RID MotionMatchingDebugMaterials::get_ball_material() const {
	return ball_material.is_valid() ? ball_material->get_rid() : RID();
}

RID MotionMatchingDebugMaterials::get_cylinder_material() const {
	return cylinder_material.is_valid() ? cylinder_material->get_rid() : RID();
}

}
