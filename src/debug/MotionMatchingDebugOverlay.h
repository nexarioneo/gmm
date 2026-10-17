#pragma once

#include <godot_cpp/variant/string.hpp>

namespace godot::MotionMatching {

class MotionMatchingDebugOverlay {
public:
	MotionMatchingDebugOverlay() = default;
	~MotionMatchingDebugOverlay() = default;

	void set_text(const String &t);

	String get_text() const;

	void clear();

	void set_editor_preview(bool p_enabled);
	bool get_editor_preview() const;

private:
	String last_text;
	bool editor_preview = false;
};

}
