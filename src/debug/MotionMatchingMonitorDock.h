#pragma once

#include <godot_cpp/classes/check_button.hpp>
#include <godot_cpp/classes/editor_plugin.hpp>
#include <godot_cpp/classes/grid_container.hpp>
#include <godot_cpp/classes/label.hpp>
#include <godot_cpp/classes/v_box_container.hpp>

#include "MotionMatchingHistogramView.h"
#include "MotionMatchingMonitor.h"

namespace godot::MotionMatching {

class MotionMatchingMonitorPanel : public VBoxContainer {
	GDCLASS(MotionMatchingMonitorPanel, VBoxContainer)

public:
	MotionMatchingMonitorPanel();

	void set_monitor(const MotionMatchingMonitor *monitor) { m_monitor = monitor; }
	void build_ui();

	void _process(double delta) override;

protected:
	static void _bind_methods();

private:
	Label *make_value_label(GridContainer *grid, const char *caption);

	const MotionMatchingMonitor *m_monitor = nullptr;

	CheckButton *m_showPerf = nullptr;
	CheckButton *m_showCosts = nullptr;

	Label *m_dbMem = nullptr;
	Label *m_dbMinutes = nullptr;
	Label *m_dbFrames = nullptr;
	Label *m_dbMotions = nullptr;
	Label *m_fmMem = nullptr;
	Label *m_fmFrames = nullptr;
	Label *m_fmDims = nullptr;
	Label *m_kdMem = nullptr;
	Label *m_kdNodes = nullptr;
	Label *m_kdDims = nullptr;

	MotionMatchingHistogramView *m_perfView = nullptr;
	MotionMatchingHistogramView *m_costView = nullptr;
};

class MotionMatchingMonitorDock : public EditorPlugin {
	GDCLASS(MotionMatchingMonitorDock, EditorPlugin)

public:
	MotionMatchingMonitorDock();
	~MotionMatchingMonitorDock() override;

	void _enter_tree() override;
	void _exit_tree() override;

protected:
	static void _bind_methods();

private:
	MotionMatchingMonitor m_monitor;
	MotionMatchingMonitorPanel *m_panel = nullptr;
};

}
