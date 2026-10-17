#pragma once

#include <godot_cpp/classes/project_settings.hpp>

namespace godot::MotionMatching {

struct MotionMatchingSystem {
	static void ensure_project_settings() {
		ProjectSettings *settings = ProjectSettings::get_singleton();
		if (!settings) {
			return;
		}
		if (!settings->has_setting("motion_matching/debug_draw")) {
			settings->set_setting("motion_matching/debug_draw", false);
		}
		if (!settings->has_setting("motion_matching/debug_draw_velocity_scale")) {
			settings->set_setting("motion_matching/debug_draw_velocity_scale", 0.1);
		}
		if (!settings->has_setting("motion_matching/debug_draw_query_pose")) {
			settings->set_setting("motion_matching/debug_draw_query_pose", false);
		}
		if (!settings->has_setting("motion_matching/debug_draw_query_velocities")) {
			settings->set_setting("motion_matching/debug_draw_query_velocities", false);
		}
		if (!settings->has_setting("motion_matching/use_kd_tree")) {
			settings->set_setting("motion_matching/use_kd_tree", true);
		}
		if (!settings->has_setting("motion_matching/multi_threaded_initialization")) {
			settings->set_setting("motion_matching/multi_threaded_initialization", true);
		}
	}
};

}
