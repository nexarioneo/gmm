#pragma once

#include <godot_cpp/variant/rid.hpp>

#include "debug/compat.h"
#include "debug/debug_balls.h"
#include "debug/debug_cylinders.h"
#include "debug/debug_lines.h"
#include "debug/debug_materials.h"

namespace godot::MotionMatching {

class MotionMatchingDebugRenderer {
public:
	MotionMatchingDebugRenderer() = default;
	~MotionMatchingDebugRenderer() { dispose(); }

	MotionMatchingDebugRenderer(const MotionMatchingDebugRenderer &) = delete;
	MotionMatchingDebugRenderer &operator=(const MotionMatchingDebugRenderer &) = delete;
	MotionMatchingDebugRenderer(MotionMatchingDebugRenderer &&) = delete;
	MotionMatchingDebugRenderer &operator=(MotionMatchingDebugRenderer &&) = delete;

	void render(const DebugDrawOutput &p_output, const RID &p_scenario);

	void clear();

	void dispose();

private:
	void hide_all();

	MotionMatchingDebugLines m_lines;
	MotionMatchingDebugBalls m_balls;
	MotionMatchingDebugCylinders m_cylinders;
	MotionMatchingDebugMaterials m_materials;

	RID m_line_mesh;
	RID m_line_instance;
	RID m_scenario;

	bool m_warned_truncate = false;
};

}
