#include "MotionMatchingDebugRenderer.h"

#include <cstdint>

#include <godot_cpp/classes/rendering_server.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

namespace godot::MotionMatching {

void MotionMatchingDebugRenderer::render(const DebugDrawOutput &p_output, const RID &p_scenario) {
	RenderingServer *rs = RenderingServer::get_singleton();
	if (rs == nullptr) {
		return;
	}
	if (!p_scenario.is_valid()) {
		hide_all();
		return;
	}

	if (p_scenario != m_scenario) {
		m_scenario = p_scenario;
		if (m_line_instance.is_valid()) {
			rs->instance_set_scenario(m_line_instance, m_scenario);
		}
	}

	m_materials.ensure();
	m_balls.ensure(p_scenario);
	m_cylinders.ensure(p_scenario);
	m_balls.set_material(m_materials.get_ball_material());
	m_cylinders.set_material(m_materials.get_cylinder_material());

	if (!m_line_mesh.is_valid()) {
		m_line_mesh = rs->mesh_create();
	}
	if (!m_line_instance.is_valid()) {
		m_line_instance = rs->instance_create();
		rs->instance_set_base(m_line_instance, m_line_mesh);
	}
	rs->instance_set_scenario(m_line_instance, m_scenario);

	if (p_output.line_points.is_empty() && p_output.ball_centers.is_empty() && p_output.cylinder_points.is_empty()) {
		hide_all();
		return;
	}

	if (!m_warned_truncate) {
		int64_t want_balls = p_output.ball_centers.size();
		if ((int64_t)p_output.ball_sizes.size() < want_balls) {
			want_balls = p_output.ball_sizes.size();
		}
		if ((int64_t)p_output.ball_colors.size() < want_balls) {
			want_balls = p_output.ball_colors.size();
		}
		int64_t want_cylinders = (int64_t)p_output.cylinder_points.size() / 2;
		if ((int64_t)p_output.cylinder_radii.size() < want_cylinders) {
			want_cylinders = p_output.cylinder_radii.size();
		}
		if ((int64_t)p_output.cylinder_colors.size() < want_cylinders) {
			want_cylinders = p_output.cylinder_colors.size();
		}
		if (want_balls > MotionMatchingDebugBalls::MAX_BALLS || want_cylinders > MotionMatchingDebugCylinders::MAX_CYLINDERS) {
			UtilityFunctions::push_warning(String("MotionMatchingDebugRenderer: debug draw count exceeds MAX, truncating."));
			m_warned_truncate = true;
		}
	}

	m_lines.draw_lines(p_output.line_points, p_output.line_colors, m_line_mesh, m_line_instance);
	if (rs->mesh_get_surface_count(m_line_mesh) > 0) {
		const RID line_mat = m_materials.get_line_material();
		if (line_mat.is_valid()) {
			rs->mesh_surface_set_material(m_line_mesh, 0, line_mat);
		}
		rs->instance_set_visible(m_line_instance, true);
	} else {
		rs->instance_set_visible(m_line_instance, false);
	}

	m_balls.draw_balls(p_output.ball_centers, p_output.ball_sizes, p_output.ball_colors);
	m_cylinders.draw_cylinders(p_output.cylinder_points, p_output.cylinder_radii, p_output.cylinder_colors);
}

void MotionMatchingDebugRenderer::clear() {
	hide_all();
}

void MotionMatchingDebugRenderer::dispose() {
	m_balls.dispose();
	m_cylinders.dispose();
	m_materials.dispose();

	RenderingServer *rs = RenderingServer::get_singleton();
	if (rs != nullptr) {
		if (m_line_instance.is_valid()) {

			rs->instance_set_base(m_line_instance, RID());
			rs->free_rid(m_line_instance);
		}
		if (m_line_mesh.is_valid()) {
			rs->free_rid(m_line_mesh);
		}
	}
	m_line_instance = RID();
	m_line_mesh = RID();
	m_scenario = RID();
	m_warned_truncate = false;
}

void MotionMatchingDebugRenderer::hide_all() {
	m_balls.hide();
	m_cylinders.hide();

	RenderingServer *rs = RenderingServer::get_singleton();
	if (rs == nullptr) {
		return;
	}

	if (m_line_mesh.is_valid()) {
		rs->mesh_clear(m_line_mesh);
	}
	if (m_line_instance.is_valid()) {
		rs->instance_set_visible(m_line_instance, false);
	}
}

}
