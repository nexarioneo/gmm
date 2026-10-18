#pragma once

#include <string>
#include <vector>

#include <godot_cpp/classes/control.hpp>
#include <godot_cpp/variant/color.hpp>
#include <godot_cpp/variant/vector2.hpp>

namespace godot::MotionMatching {

class MotionMatchingHistogramView : public Control {
	GDCLASS(MotionMatchingHistogramView, Control)

public:
	struct Bar {
		std::string m_label;
		float m_value = 0.0f;
		Color m_color;
	};

	MotionMatchingHistogramView();

	void set_title(const String &title) { m_title = title; }
	void set_bars(const std::vector<Bar> &bars);

	void _draw() override;

protected:
	static void _bind_methods();

private:
	String m_title;
	std::vector<Bar> m_bars;
};

}
