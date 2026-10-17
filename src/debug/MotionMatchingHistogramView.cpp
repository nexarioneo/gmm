#include "MotionMatchingHistogramView.h"

namespace godot::MotionMatching {

MotionMatchingHistogramView::MotionMatchingHistogramView() {
	set_custom_minimum_size(Vector2(0.0f, 120.0f));
	set_v_size_flags(Control::SIZE_EXPAND_FILL);
}

void MotionMatchingHistogramView::_bind_methods() {
}

void MotionMatchingHistogramView::set_bars(const std::vector<Bar> &bars) {
	m_bars = bars;
	queue_redraw();
}

void MotionMatchingHistogramView::_draw() {
	const Vector2 area = get_size();

	draw_rect(Rect2(Vector2(), area), Color(0.08f, 0.08f, 0.1f, 1.0f));

	float maxValue = 0.0f;
	for (const Bar &bar : m_bars) {
		if (bar.m_value > maxValue) {
			maxValue = bar.m_value;
		}
	}

	const float topPad = 20.0f;
	const float bottomPad = 4.0f;
	const float plotHeight = area.y - topPad - bottomPad;
	if (!m_bars.empty() && maxValue > 0.0f && plotHeight > 0.0f) {
		const float slotWidth = area.x / (float)m_bars.size();
		const float barWidth = slotWidth > 2.0f ? slotWidth - 1.0f : slotWidth;
		for (size_t i = 0; i < m_bars.size(); ++i) {
			const float height = (m_bars[i].m_value / maxValue) * plotHeight;
			const float x = (float)i * slotWidth;
			draw_rect(Rect2(Vector2(x, topPad + plotHeight - height), Vector2(barWidth, height)), m_bars[i].m_color);
		}
	}

	Ref<Font> font = get_theme_default_font();
	const int fontSize = get_theme_default_font_size();
	if (font.is_valid()) {
		draw_string(font, Vector2(4.0f, 14.0f), m_title + String(" (max ") + String::num(maxValue, 2) + String(")"));
	}
}

}
