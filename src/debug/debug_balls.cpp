#include "debug/debug_balls.h"

#include <godot_cpp/classes/rendering_server.hpp>
#include <godot_cpp/core/math.hpp>
#include <godot_cpp/variant/basis.hpp>
#include <godot_cpp/variant/color.hpp>
#include <godot_cpp/variant/transform3d.hpp>
#include <godot_cpp/variant/vector3.hpp>

namespace godot::MotionMatching {

void MotionMatchingDebugBalls::ensure(const RID &p_scenario) {
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

	m_sphere.instantiate();
	m_sphere->set_radius(1.0f);
	m_sphere->set_height(2.0f);
	m_sphere->set_radial_segments(kSphereRadialSegments);
	m_sphere->set_rings(kSphereRings);
	m_mesh_rid = m_sphere->get_rid();

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

void MotionMatchingDebugBalls::allocate_locked(int32_t p_want) {
	RenderingServer *rs = RenderingServer::get_singleton();
	if (rs == nullptr || !m_multimesh.is_valid()) {
		return;
	}
	int32_t want = p_want < 1 ? 1 : p_want;
	if (want > MAX_BALLS) {
		want = MAX_BALLS;
	}

	rs->multimesh_allocate_data(m_multimesh, want,
		RenderingServer::MULTIMESH_TRANSFORM_3D, true);
	rs->multimesh_set_mesh(m_multimesh, m_mesh_rid);
	m_capacity = want;
}

void MotionMatchingDebugBalls::draw_balls(const PackedVector3Array &p_centers,
		const PackedFloat32Array &p_sizes,
		const PackedColorArray &p_colors) {
	RenderingServer *rs = RenderingServer::get_singleton();
	if (rs == nullptr || !m_built) {
		return;
	}
	if (!m_multimesh.is_valid()) {
		return;
	}

	int64_t n = p_centers.size();
	if ((int64_t)p_sizes.size() < n) {
		n = p_sizes.size();
	}
	if ((int64_t)p_colors.size() < n) {
		n = p_colors.size();
	}
	if (n > MAX_BALLS) {
		n = MAX_BALLS;
	}
	if (n <= 0) {
		hide();
		return;
	}

	const int32_t count = static_cast<int32_t>(n);
	if (count > m_capacity) {

		const int64_t grown = (int64_t)count + (int64_t)(count / 2);
		const int32_t want = grown > MAX_BALLS ? MAX_BALLS : static_cast<int32_t>(grown);
		allocate_locked(want < count ? count : want);
	}

	for (int32_t i = 0; i < count; ++i) {
		const float s = p_sizes[i];
		const Vector3 center = p_centers[i];

		if (!(s > 0.0f) || Math::is_nan(s)) {

			const Basis zero = Basis().scaled(Vector3(0.0f, 0.0f, 0.0f));
			rs->multimesh_instance_set_transform(m_multimesh, i, Transform3D(zero, center));
			rs->multimesh_instance_set_color(m_multimesh, i, Color(0.0f, 0.0f, 0.0f, 0.0f));
			continue;
		}

		const Basis scaled = Basis().scaled(Vector3(s, s, s));
		rs->multimesh_instance_set_transform(m_multimesh, i, Transform3D(scaled, center));
		rs->multimesh_instance_set_color(m_multimesh, i, p_colors[i]);
	}
	rs->multimesh_set_visible_instances(m_multimesh, count);
}

void MotionMatchingDebugBalls::set_material(const RID &p_material) {
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

void MotionMatchingDebugBalls::hide() {
	if (!m_built || !m_multimesh.is_valid()) {
		return;
	}
	RenderingServer *rs = RenderingServer::get_singleton();
	if (rs == nullptr) {
		return;
	}
	rs->multimesh_set_visible_instances(m_multimesh, 0);
}

void MotionMatchingDebugBalls::dispose() {
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
	m_sphere.unref();
	m_scenario = RID();
	m_capacity = 0;
	m_built = false;
}

}
