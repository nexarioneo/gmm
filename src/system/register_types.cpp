#include "register_types.h"

#include <gdextension_interface.h>
#include <godot_cpp/classes/editor_plugin_registration.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/godot.hpp>

#include "../runtime/EventData.h"
#include "../debug/MotionMatchingMonitorDock.h"
#include "../nodes/MotionMatching.h"
#include "../nodes/MotionMatchingNode.h"
#include "../features/Feature.h"
#include "../features/FeatureAngularVelocity.h"
#include "../features/FeaturePosition.h"
#include "../features/FeatureSchema.h"
#include "../features/FeatureTrajectory.h"
#include "../features/FeatureVelocity.h"
#include "../math/QueryVector.h"

using namespace godot;

void initialize_motion_matching_module(ModuleInitializationLevel p_level) {
	if (p_level == MODULE_INITIALIZATION_LEVEL_SCENE) {
		GDREGISTER_CLASS(MotionMatching::QueryVector);
		GDREGISTER_CLASS(MotionMatching::FeatureSchema);
		GDREGISTER_CLASS(MotionMatching::EventData);
		GDREGISTER_CLASS(MotionMatching::DiscardFrameEventData);
		GDREGISTER_CLASS(MotionMatching::TagEventData);

		GDREGISTER_ABSTRACT_CLASS(MotionMatching::Feature);
		GDREGISTER_CLASS(MotionMatching::FeaturePosition);
		GDREGISTER_CLASS(MotionMatching::FeatureVelocity);
		GDREGISTER_CLASS(MotionMatching::FeatureAngularVelocity);
		GDREGISTER_CLASS(MotionMatching::FeatureTrajectory);
		GDREGISTER_CLASS(MotionMatching::MotionMatchingNode);
		GDREGISTER_CLASS(MotionMatching::MotionMatching);
	}

	if (p_level == MODULE_INITIALIZATION_LEVEL_EDITOR) {
		GDREGISTER_CLASS(MotionMatching::MotionMatchingHistogramView);
		GDREGISTER_CLASS(MotionMatching::MotionMatchingMonitorPanel);
		GDREGISTER_CLASS(MotionMatching::MotionMatchingMonitorDock);
		EditorPlugins::add_by_type<MotionMatching::MotionMatchingMonitorDock>();
	}
}

void uninitialize_motion_matching_module(ModuleInitializationLevel p_level) {
}

extern "C" {
GDExtensionBool GDE_EXPORT motion_matching_library_init(GDExtensionInterfaceGetProcAddress p_get_proc_address, GDExtensionClassLibraryPtr p_library, GDExtensionInitialization *r_initialization) {
	GDExtensionBinding::InitObject init_obj(p_get_proc_address, p_library, r_initialization);
	init_obj.register_initializer(initialize_motion_matching_module);
	init_obj.register_terminator(uninitialize_motion_matching_module);
	init_obj.set_minimum_library_initialization_level(MODULE_INITIALIZATION_LEVEL_SCENE);
	return init_obj.init();
}
}
