#include "debug/debug_cylinders.h"

#include <godot_cpp/classes/rendering_server.hpp>
#include <godot_cpp/core/math.hpp>
#include <godot_cpp/variant/basis.hpp>
#include <godot_cpp/variant/color.hpp>
#include <godot_cpp/variant/quaternion.hpp>
#include <godot_cpp/variant/transform3d.hpp>
#include <godot_cpp/variant/vector3.hpp>

namespace godot::MotionMatching {

void MotionMatchingDebugCylinders::ensure(const RID &p_scenario) {
	RenderingServer *rs = RenderingServer::get_singleton();
	if (rs == nullptr) {
		return;
	}
	if (m_built) {

		if (p_scenario.is_valid() && p_scenario != m_scenario) {
			m_scenario = p_scenario;
			if (m_instance.is_valid()) {
				rs->instance_set_scenario(m_instance, m_scenario);
			}
		}
		return;
	}

	m_cylinder.instantiate();
	m_cylinder->set_top_radius(1.0f);
	m_cylinder->set_bottom_radius(1.0f);
	m_cylinder->set_height(1.0f);
	m_cylinder->set_radial_segments(kRadialSegments);
	m_cylinder->set_rings(kRings);
	m_cylinder->set_cap_top(true);
	m_cylinder->set_cap_bottom(true);
	m_mesh_rid = m_cylinder->get_rid();

	m_multimesh = rs->multimesh_create();
	allocate_locked(kInitialCapacity);
	rs->multimesh_set_mesh(m_multimesh, m_mesh_rid);
	rs->multimesh_set_visible_instances(m_multimesh, 0);

	m_instance = rs->instance_create();
	rs->instance_set_base(m_instance, m_multimesh);
	m_scenario = p_scenario;
	if (m_scenario.is_valid()) {
		rs->instance_set_scenario(m_instance, m_scenario);
	}

	m_built = true;
}

void MotionMatchingDebugCylinders::allocate_locked(int32_t p_want) {
	RenderingServer *rs = RenderingServer::get_singleton();
	if (rs == nullptr || !m_multimesh.is_valid()) {
		return;
	}
	int32_t want = p_want < 1 ? 1 : p_want;
	if (want > MAX_CYLINDERS) {
		want = MAX_CYLINDERS;
	}

	rs->multimesh_allocate_data(m_multimesh, want,
		RenderingServer::MULTIMESH_TRANSFORM_3D, true);
	rs->multimesh_set_mesh(m_multimesh, m_mesh_rid);
	m_capacity = want;
}

void MotionMatchingDebugCylinders::draw_cylinders(const PackedVector3Array &p_points,
		const PackedFloat32Array &p_radii,
		const PackedColorArray &p_colors) {
	RenderingServer *rs = RenderingServer::get_singleton();
	if (rs == nullptr || !m_built) {
		return;
	}
	if (!m_multimesh.is_valid()) {
		return;
	}

	int64_t n_seg = p_points.size() / 2;
	if ((int64_t)p_radii.size() < n_seg) {
		n_seg = p_radii.size();
	}
	if ((int64_t)p_colors.size() < n_seg) {
		n_seg = p_colors.size();
	}
	if (n_seg > MAX_CYLINDERS) {
		n_seg = MAX_CYLINDERS;
	}
	if (n_seg <= 0) {
		hide();
		return;
	}

	const int32_t count = static_cast<int32_t>(n_seg);
	if (count > m_capacity) {

		const int64_t grown = (int64_t)count + (int64_t)(count / 2);
		const int32_t want = grown > MAX_CYLINDERS ? MAX_CYLINDERS : static_cast<int32_t>(grown);
		allocate_locked(want < count ? count : want);
	}

	static const Vector3 kUp(0.0f, 1.0f, 0.0f);

	for (int32_t i = 0; i < count; ++i) {
		const Vector3 a = p_points[i * 2];
		const Vector3 b = p_points[i * 2 + 1];
		const float radius = p_radii[i];

		const Vector3 axis = b - a;
		const float h = axis.length();

		if (!(h > kEps) || !(radius > 0.0f) || !Math::is_finite(h) || !Math::is_finite(radius)) {

			Vector3 mid = (a + b) * 0.5f;
			if (!mid.is_finite()) {
				mid = Vector3(0.0f, 0.0f, 0.0f);
			}
			const Basis zero = Basis().scaled(Vector3(0.0f, 0.0f, 0.0f));
			rs->multimesh_instance_set_transform(m_multimesh, i, Transform3D(zero, mid));
			rs->multimesh_instance_set_color(m_multimesh, i, Color(0.0f, 0.0f, 0.0f, 0.0f));
			continue;
		}

		const Vector3 axis_norm = axis / h;

		const Quaternion q(kUp, axis_norm);
		const Basis rot(q);
		const Basis scl = Basis::from_scale(Vector3(radius, h, radius));
		const Basis basis = rot * scl;
		const Vector3 mid = (a + b) * 0.5f;
		rs->multimesh_instance_set_transform(m_multimesh, i, Transform3D(basis, mid));
		rs->multimesh_instance_set_color(m_multimesh, i, p_colors[i]);
	}
	rs->multimesh_set_visible_instances(m_multimesh, count);
}

void MotionMatchingDebugCylinders::set_material(const RID &p_material) {
	if (!m_built || !m_mesh_rid.is_valid()) {
		return;
	}
	if (m_material_rid == p_material) {
		return;
	}
	RenderingServer *rs = RenderingServer::get_singleton();
	if (rs == nullptr) {
		return;
	}
	rs->mesh_surface_set_material(m_mesh_rid, 0, p_material);
	m_material_rid = p_material;
}

void MotionMatchingDebugCylinders::hide() {
	if (!m_built || !m_multimesh.is_valid()) {
		return;
	}
	RenderingServer *rs = RenderingServer::get_singleton();
	if (rs == nullptr) {
		return;
	}
	rs->multimesh_set_visible_instances(m_multimesh, 0);
}

void MotionMatchingDebugCylinders::dispose() {
	RenderingServer *rs = RenderingServer::get_singleton();
	if (rs != nullptr) {
		if (m_instance.is_valid()) {

			rs->instance_set_base(m_instance, RID());
			rs->free_rid(m_instance);
		}
		if (m_multimesh.is_valid()) {
			rs->free_rid(m_multimesh);
		}
	}
	m_instance = RID();
	m_multimesh = RID();

	m_mesh_rid = RID();
	m_cylinder.unref();
	m_scenario = RID();
	m_capacity = 0;
	m_built = false;
}

}
