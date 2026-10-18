#include "debug/monitor_dock.h"

namespace godot::MotionMatching {

MotionMatchingMonitorPanel::MotionMatchingMonitorPanel() {
	set_process(true);
}

void MotionMatchingMonitorPanel::_bind_methods() {
}

Label *MotionMatchingMonitorPanel::make_value_label(GridContainer *grid, const char *caption) {
	Label *name = memnew(Label);
	name->set_text(caption);
	grid->add_child(name);
	Label *value = memnew(Label);
	value->set_text("-");
	value->set_horizontal_alignment(HORIZONTAL_ALIGNMENT_RIGHT);
	value->set_h_size_flags(Control::SIZE_EXPAND_FILL);
	grid->add_child(value);
	return value;
}

static GridContainer *make_section(VBoxContainer *parent, const char *title) {
	Label *header = memnew(Label);
	header->set_text(title);
	parent->add_child(header);
	GridContainer *grid = memnew(GridContainer);
	grid->set_columns(2);
	parent->add_child(grid);
	return grid;
}

void MotionMatchingMonitorPanel::_process(double ) {
	if (m_monitor == nullptr || m_dbMem == nullptr || m_showPerf == nullptr || m_showCosts == nullptr || m_perfView == nullptr || m_costView == nullptr) {
		return;
	}
	const MotionMatchingMonitorRequests::FrameDatabaseInfo db = m_monitor->CopyFrameDatabaseInfo();
	m_dbMem->set_text(String::num((float)db.m_memoryUsedInBytes / 1024.0f / 1024.0f, 2) + String(" MB"));
	m_dbMinutes->set_text(String::num(db.m_motionDataInSeconds / 60.0f, 2) + String(" minutes"));
	m_dbFrames->set_text(String::num_int64((int64_t)db.m_numFrames));
	m_dbMotions->set_text(String::num_int64((int64_t)db.m_numMotions));

	const MotionMatchingMonitorRequests::FeatureMatrixInfo fm = m_monitor->CopyFeatureMatrixInfo();
	m_fmMem->set_text(String::num((float)fm.m_memoryUsedInBytes / 1024.0f / 1024.0f, 2) + String(" MB"));
	m_fmFrames->set_text(String::num_int64((int64_t)fm.m_numFrames));
	m_fmDims->set_text(String::num_int64((int64_t)fm.m_numDimensions));

	const MotionMatchingMonitorRequests::KdTreeInfo kd = m_monitor->CopyKdTreeInfo();
	m_kdMem->set_text(String::num((float)kd.m_memoryUsedInBytes / 1024.0f / 1024.0f, 2) + String(" MB"));
	m_kdNodes->set_text(String::num_int64((int64_t)kd.m_numNodes));
	m_kdDims->set_text(String::num_int64((int64_t)kd.m_numDimensions));

	const bool showPerf = m_showPerf->is_pressed();
	const bool showCosts = m_showCosts->is_pressed();

	const MotionMatchingMonitor::Histogram perf = m_monitor->CopyPerformanceStats();
	if (showPerf) {
		std::vector<MotionMatchingHistogramView::Bar> bars;
		bars.reserve(perf.m_values.size());
		for (const auto &entry : perf.m_values) {
			MotionMatchingHistogramView::Bar bar;
			bar.m_label = entry.m_label;
			bar.m_value = entry.m_value;
			bar.m_color = entry.m_color;
			bars.push_back(bar);
		}
		m_perfView->set_bars(bars);
	}
	m_perfView->set_visible(showPerf);

	const MotionMatchingMonitor::Histogram costs = m_monitor->CopyFeatureCosts();
	if (showCosts) {
		std::vector<MotionMatchingHistogramView::Bar> bars;
		bars.reserve(costs.m_values.size());
		for (const auto &entry : costs.m_values) {
			MotionMatchingHistogramView::Bar bar;
			bar.m_label = entry.m_label;
			bar.m_value = entry.m_value;
			bar.m_color = entry.m_color;
			bars.push_back(bar);
		}
		m_costView->set_bars(bars);
	}
	m_costView->set_visible(showCosts);
}

MotionMatchingMonitorDock::MotionMatchingMonitorDock() {
}

MotionMatchingMonitorDock::~MotionMatchingMonitorDock() {
	if (m_panel != nullptr) {
		remove_control_from_docks(m_panel);
		memdelete(m_panel);
		m_panel = nullptr;
	}
}

void MotionMatchingMonitorDock::_bind_methods() {
}

void MotionMatchingMonitorPanel::build_ui() {
	set_name("Motion Matching");

	m_showPerf = memnew(CheckButton);
	m_showPerf->set_text("Performance Statistics");
	m_showPerf->set_pressed(true);
	add_child(m_showPerf);

	m_showCosts = memnew(CheckButton);
	m_showCosts->set_text("Feature Costs");
	m_showCosts->set_pressed(true);
	add_child(m_showCosts);

	GridContainer *dbGrid = make_section(this, "Motion Database");
	m_dbMem = make_value_label(dbGrid, "Memory Usage:");
	m_dbMinutes = make_value_label(dbGrid, "Motion Data:");
	m_dbFrames = make_value_label(dbGrid, "Num Frames:");
	m_dbMotions = make_value_label(dbGrid, "Num Motions:");

	GridContainer *fmGrid = make_section(this, "Feature Matrix");
	m_fmMem = make_value_label(fmGrid, "Memory Usage:");
	m_fmFrames = make_value_label(fmGrid, "Num Frames:");
	m_fmDims = make_value_label(fmGrid, "Num Feature Components:");

	GridContainer *kdGrid = make_section(this, "Kd-Tree");
	m_kdMem = make_value_label(kdGrid, "Memory Usage:");
	m_kdNodes = make_value_label(kdGrid, "Num Nodes:");
	m_kdDims = make_value_label(kdGrid, "Num Dimensions:");

	m_perfView = memnew(MotionMatchingHistogramView);
	m_perfView->set_title("Performance Statistics");
	add_child(m_perfView);

	m_costView = memnew(MotionMatchingHistogramView);
	m_costView->set_title("Feature Costs");
	add_child(m_costView);

	set_process(true);
}

void MotionMatchingMonitorDock::_enter_tree() {
	if (m_panel == nullptr) {
		m_panel = memnew(MotionMatchingMonitorPanel);
		m_panel->set_monitor(&m_monitor);
		m_panel->build_ui();
		add_control_to_dock(DOCK_SLOT_RIGHT_UL, m_panel);
	}
}

void MotionMatchingMonitorDock::_exit_tree() {
	if (m_panel != nullptr) {
		remove_control_from_docks(m_panel);
		memdelete(m_panel);
		m_panel = nullptr;
	}
}

}
