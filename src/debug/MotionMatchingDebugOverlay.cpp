#include "MotionMatchingDebugOverlay.h"

namespace godot::MotionMatching {

void MotionMatchingDebugOverlay::set_text(const String &t) {
	last_text = t;
}

String MotionMatchingDebugOverlay::get_text() const {
	return last_text;
}

void MotionMatchingDebugOverlay::clear() {
	last_text = String();
}

void MotionMatchingDebugOverlay::set_editor_preview(bool p_enabled) {
	editor_preview = p_enabled;
}

bool MotionMatchingDebugOverlay::get_editor_preview() const {
	return editor_preview;
}

}
