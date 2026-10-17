#include "MotionMatchingDebugLines.h"

#include <godot_cpp/classes/rendering_server.hpp>
#include <godot_cpp/variant/array.hpp>

namespace godot::MotionMatching {

void MotionMatchingDebugLines::draw_lines(const PackedVector3Array &points, const PackedColorArray &colors, RID line_mesh, RID line_instance) {
	(void)line_instance;

	RenderingServer *rs = RenderingServer::get_singleton();
	if (rs == nullptr) {
		return;
	}
	if (!line_mesh.is_valid()) {
		return;
	}

	rs->mesh_clear(line_mesh);

	int64_t valid = points.size() < colors.size() ? points.size() : colors.size();
	valid -= (valid % 2);
	if (valid < 2) {
		return;
	}

	PackedVector3Array verts;
	verts.resize(valid);
	PackedColorArray cols;
	cols.resize(valid);
	for (int64_t i = 0; i < valid; ++i) {
		verts[i] = points[i];
		cols[i] = colors[i];
	}

	Array arrays;
	arrays.resize(Mesh::ARRAY_MAX);
	arrays[Mesh::ARRAY_VERTEX] = verts;
	arrays[Mesh::ARRAY_COLOR] = cols;

	rs->mesh_add_surface_from_arrays(line_mesh, RenderingServer::PRIMITIVE_LINES, arrays);
}

}
