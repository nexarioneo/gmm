#include "nodes/motion_matching.h"

#include <cstdint>

#include <godot_cpp/classes/object.hpp>
#include <godot_cpp/classes/project_settings.hpp>
#include <godot_cpp/classes/skeleton3d.hpp>
#include <godot_cpp/variant/callable.hpp>
#include <godot_cpp/classes/viewport.hpp>
#include <godot_cpp/classes/world3d.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/memory.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#include "features/feature.h"
#include "features/feature_schema_default.h"
#include "database/motion_matching_data.h"
#include "runtime/motion_matching_instance.h"
#include "system/motion_matching_system.h"

#include "actor/actor_instance.h"
#include "animation/motion.h"
#include "animation/motion_instance.h"
#include "animation/motion_set.h"
#include "pose/pose.h"
#include "core/types/transform.h"
#include "pose/transform_data.h"

namespace godot::MotionMatching {

MotionMatching::MotionMatching() {
	m_core = Ref<MotionMatchingNode>(memnew(MotionMatchingNode));
	if (m_core.is_valid() && !m_core->is_connected("motion_event_fired", Callable(this, "_on_core_motion_event"))) {
		m_core->connect("motion_event_fired", Callable(this, "_on_core_motion_event"));
	}
	m_dataDirty = true;
}

void MotionMatching::_on_core_motion_event(const String &tag) {
	emit_signal("motion_event_fired", tag);
}

MotionMatching::~MotionMatching() {

	if (!m_core.is_null() && m_core->is_visualize_feature_schema()) {
		m_debugData.reset();
	} else if (m_debugData) {
		m_debugData.reset();
	}
}

void MotionMatching::_ready() {
	MotionMatchingSystem::ensure_project_settings();
	if (m_core.is_null()) {
		m_core = Ref<MotionMatchingNode>(memnew(MotionMatchingNode));
	}
	if (m_core.is_valid() && !m_core->is_connected("motion_event_fired", Callable(this, "_on_core_motion_event"))) {
		m_core->connect("motion_event_fired", Callable(this, "_on_core_motion_event"));
	}

	Ref<FeatureSchema> schema = m_core->get_feature_schema();
	if (schema.is_null()) {
		Ref<FeatureSchema> fresh;
		fresh.instantiate();
		m_core->set_feature_schema(fresh);
	} else if (schema->GetNumFeatures() == 0) {
		m_core->reinit();
	} else {
		m_core->reinit();
	}
	if (m_core->is_visualize_feature_schema() && m_debugVisFeatureSchema.is_null()) {
		Ref<FeatureSchema> s = m_core->get_feature_schema();
		if (s.is_valid()) {
			m_debugVisFeatureSchema = s;
		}
	}
	m_dataDirty = false;
}

void MotionMatching::_process(double p_delta) {
	MotionMatchingSystem::ensure_project_settings();
	if (m_core.is_null()) {
		return;
	}
	if (m_dataDirty) {
		m_core->reinit();
		m_dataDirty = false;
	}

	const float delta = static_cast<float>(p_delta);
	Vector3 facing = m_core->get_target_facing_dir();
	facing.y = 0.0f;
	facing = NormalizedSafe(facing);
	m_core->Update(delta, m_core->get_target_position(), facing, m_core->get_use_facing_dir());
	m_core->PostUpdate(delta);
	m_core->Output();

	MotionMatchingInstance *instance = m_core->m_uniqueData.m_instance;
	const bool hasError = m_core->m_uniqueData.m_hasError;
	if (!hasError && instance != nullptr) {
		Pose outputPose;
		instance->Output(outputPose);
		::godot::Skeleton3D *skeleton = resolve_skeleton();
		if (skeleton) {
			const int32_t boneCount = skeleton->get_bone_count();
			const size_t numTransforms = outputPose.GetNumTransforms();
			for (int32_t i = 0; i < boneCount; ++i) {
				if (static_cast<size_t>(i) >= numTransforms) {
					break;
				}
				const Transform jointTransform = outputPose.GetLocalSpaceTransform(static_cast<size_t>(i));
				skeleton->set_bone_pose_position(i, jointTransform.m_position);
				skeleton->set_bone_pose_rotation(i, jointTransform.m_rotation);
			}
		}
	}

	bool projectDebug = false;
	if (ProjectSettings *ps = ProjectSettings::get_singleton()) {
		projectDebug = ps->get_setting("motion_matching/debug_draw", false).booleanize();
	}
	if (m_core->is_debug_draw_schema_enabled() || projectDebug) {

		Viewport *viewport = get_viewport();
		if (viewport == nullptr) {
			if (m_drawActive) {
				m_renderer.clear();
				m_drawActive = false;
			}
			return;
		}
		Ref<World3D> world = viewport->find_world_3d();
		if (world.is_null()) {
			if (m_drawActive) {
				m_renderer.clear();
				m_drawActive = false;
			}
			return;
		}
		if (world->get_scenario().is_valid() == false) {
			if (m_drawActive) {
				m_renderer.clear();
				m_drawActive = false;
			}
			return;
		}
		m_debugOutput.clear();
		debug_draw_frame_features(m_debugOutput);
		if (instance && !hasError) {
			instance->DebugDraw(m_debugOutput);
		}
		m_renderer.render(m_debugOutput, world->get_scenario());
		m_drawActive = true;
	} else if (m_drawActive) {
		m_renderer.clear();
		m_drawActive = false;
	}
}

void MotionMatching::_exit_tree() {
	m_renderer.clear();
	m_renderer.dispose();
	m_drawActive = false;
}

void MotionMatching::debug_draw_frame_features(DebugDrawOutput &p_output) {

	if (m_core.is_null() || !m_core->is_debug_draw_schema_enabled() || m_debugVisFeatureSchema.is_null()) {
		return;
	}

	MotionInstance *motionInstance = nullptr;
	{
		const ActorInstance &actorInstance = m_core->m_uniqueData.m_actorInstance;
		MotionSystem *motionSystem = actorInstance.GetMotionSystem();
		if (motionSystem && motionSystem->GetIsPlaying()) {
			motionInstance = motionSystem->GetMotionInstance(0);
		}
	}

	if (motionInstance) {
		Motion *motion = motionInstance->GetMotion();
		ActorInstance *actorInstance = motionInstance->GetActorInstance();
		if (!motion || !actorInstance || !actorInstance->GetTransformData() || !actorInstance->GetTransformData()->GetCurrentPose()) {
			m_lastMotionInstance = motionInstance;
			return;
		}
		const FeatureSchema &featureSchema = *m_debugVisFeatureSchema.ptr();
		const Pose &currentPose = *actorInstance->GetTransformData()->GetCurrentPose();

		if (!m_debugData ||
			m_lastMotionInstance != motionInstance) {
			m_debugData = std::make_unique<MotionMatchingData>(featureSchema);

			MotionMatchingData::InitSettings settings;
			settings.m_actorInstance = actorInstance;
			settings.m_motionList.emplace_back(motionInstance->GetMotion());
			m_debugData->Init(settings);
		}

		const size_t currentFrame = m_debugData->GetFrameDatabase().FindFrameIndex(motion, motionInstance->GetCurrentTime());
		if (currentFrame != InvalidIndex) {
			p_output.overlay_text = String("Frame = ") + String::num_int64((int64_t)currentFrame);

			for (const Ref<Feature> &feature : featureSchema.GetFeatures()) {
				if (feature.is_valid() && feature->GetDebugDrawEnabled()) {
					feature->DebugDraw(p_output, currentPose, m_debugData->GetFeatureMatrix(), m_debugData->GetFeatureTransformer(), currentFrame);
				}
			}
		}
	} else {

		m_debugData.reset();
		motionInstance = nullptr;
	}
	m_lastMotionInstance = motionInstance;
}

void MotionMatching::clear_debug_draw() {
	m_renderer.clear();
	m_drawActive = false;
}

::godot::Skeleton3D *MotionMatching::resolve_skeleton() const {
	if (!is_inside_tree()) {
		return nullptr;
	}

	if (!m_skeletonPath.is_empty()) {
		Node *node = get_node_or_null(m_skeletonPath);
		::godot::Skeleton3D *viaPath = Object::cast_to<::godot::Skeleton3D>(node);
		if (viaPath) {
			return viaPath;
		}
	}

	TypedArray<Node> found = find_children("*", "Skeleton3D", true, false);
	for (int64_t i = 0; i < found.size(); ++i) {
		Object *obj = found[i];
		::godot::Skeleton3D *skeleton = Object::cast_to<::godot::Skeleton3D>(obj);
		if (skeleton) {
			return skeleton;
		}
	}
	return nullptr;
}

void MotionMatching::SetDebugDrawFeatureSchema(FeatureSchema *p_featureSchema) {
	m_debugVisFeatureSchema = Ref<FeatureSchema>(p_featureSchema);
}

FeatureSchema *MotionMatching::GetDebugDrawFeatureSchema() const {
	return m_debugVisFeatureSchema.ptr();
}

void MotionMatching::reinit() {
	if (m_core.is_valid()) {

		Skeleton3D *skeleton = resolve_skeleton();
		m_core->SetBuildSkeleton(skeleton, skeleton);
		m_core->reinit();
	}
	refresh_joint_hints();
	m_dataDirty = false;
}

void MotionMatching::refresh_joint_hints() {
	PackedStringArray joints;
	if (Skeleton3D *skeleton = resolve_skeleton()) {
		const int32_t boneCount = skeleton->get_bone_count();
		for (int32_t i = 0; i < boneCount; ++i) {
			joints.push_back(skeleton->get_bone_name(i));
		}
	}

	if (m_core.is_valid()) {
		Ref<FeatureSchema> schema = m_core->get_feature_schema();
		if (schema.is_valid()) {
			for (const Ref<Feature> &feature : schema->GetFeatures()) {
				if (feature.is_valid()) {
					feature->SetAvailableJoints(joints);
				}
			}
		}
	}
	notify_property_list_changed();
}

void MotionMatching::toggle_debug_draw_schema() {
	if (m_core.is_valid()) {
		m_core->toggle_debug_draw_schema();
		if (m_core->is_debug_draw_schema_enabled() && m_debugVisFeatureSchema.is_null()) {
			Ref<FeatureSchema> s = m_core->get_feature_schema();
			if (s.is_valid()) {
				m_debugVisFeatureSchema = s;
			}
		}
		if (!m_core->is_debug_draw_schema_enabled()) {
			m_debugData.reset();
		}
	}
}

bool MotionMatching::is_debug_draw_schema_enabled() const {
	if (m_core.is_valid()) {
		return m_core->is_debug_draw_schema_enabled();
	}
	return false;
}

float MotionMatching::get_update_time_ms() const {
	if (m_core.is_valid()) {
		return m_core->get_update_time_ms();
	}
	return 0.0f;
}

float MotionMatching::get_post_update_time_ms() const {
	if (m_core.is_valid()) {
		return m_core->get_post_update_time_ms();
	}
	return 0.0f;
}

float MotionMatching::get_output_time_ms() const {
	if (m_core.is_valid()) {
		return m_core->get_output_time_ms();
	}
	return 0.0f;
}

Ref<FeatureSchema> MotionMatching::GetFeatureSchema() const {
	if (m_core.is_valid()) {
		return m_core->GetFeatureSchema();
	}
	return Ref<FeatureSchema>();
}

float MotionMatching::GetLowestCostSearchFrequency() const {
	if (m_core.is_valid()) {
		return m_core->GetLowestCostSearchFrequency();
	}
	return 5.0f;
}

size_t MotionMatching::GetSampleRate() const {
	if (m_core.is_valid()) {
		return m_core->GetSampleRate();
	}
	return 30;
}

TrajectoryQuery::EMode MotionMatching::GetTrajectoryQueryMode() const {
	if (m_core.is_valid()) {
		return m_core->GetTrajectoryQueryMode();
	}
	return TrajectoryQuery::MODE_TARGETDRIVEN;
}

float MotionMatching::GetPathRadius() const {
	if (m_core.is_valid()) {
		return m_core->GetPathRadius();
	}
	return 1.0f;
}

float MotionMatching::GetPathSpeed() const {
	if (m_core.is_valid()) {
		return m_core->GetPathSpeed();
	}
	return 1.0f;
}

bool MotionMatching::GetNormalizeData() const {
	if (m_core.is_valid()) {
		return m_core->GetNormalizeData();
	}
	return false;
}

MotionMatchingData::FeatureScalerType MotionMatching::GetFeatureScalerType() const {
	if (m_core.is_valid()) {
		return m_core->GetFeatureScalerType();
	}
	return MotionMatchingData::StandardScalerType;
}

float MotionMatching::GetFeatureMin() const {
	if (m_core.is_valid()) {
		return m_core->GetFeatureMin();
	}
	return 0.0f;
}

float MotionMatching::GetFeatureMax() const {
	if (m_core.is_valid()) {
		return m_core->GetFeatureMax();
	}
	return 1.0f;
}

bool MotionMatching::GetClipFeatures() const {
	if (m_core.is_valid()) {
		return m_core->GetClipFeatures();
	}
	return false;
}

size_t MotionMatching::GetMaxKdTreeDepth() const {
	if (m_core.is_valid()) {
		return m_core->GetMaxKdTreeDepth();
	}
	return 15;
}

size_t MotionMatching::GetMinFramesPerKdTreeNode() const {
	if (m_core.is_valid()) {
		return m_core->GetMinFramesPerKdTreeNode();
	}
	return 1000;
}

bool MotionMatching::GetMirror() const {
	if (m_core.is_valid()) {
		return m_core->GetMirror();
	}
	return false;
}

const Array &MotionMatching::GetMotions() const {
	static const Array s_empty;
	if (m_core.is_valid()) {
		return m_core->GetMotions();
	}
	return s_empty;
}

void MotionMatching::set_search_frequency(float p_value) {
	if (m_core.is_valid()) {
		m_core->set_search_frequency(p_value);
	}
}

float MotionMatching::get_search_frequency() const {
	if (m_core.is_valid()) {
		return m_core->get_search_frequency();
	}
	return 5.0f;
}

void MotionMatching::set_sample_rate(int64_t p_value) {
	if (m_core.is_valid()) {
		m_core->set_sample_rate(p_value);
	}
	m_dataDirty = true;
}

int64_t MotionMatching::get_sample_rate() const {
	if (m_core.is_valid()) {
		return m_core->get_sample_rate();
	}
	return 30;
}

void MotionMatching::set_trajectory_query_mode(int64_t p_value) {
	if (m_core.is_valid()) {
		m_core->set_trajectory_query_mode(p_value);
		notify_property_list_changed();
	}
}

int64_t MotionMatching::get_trajectory_query_mode() const {
	if (m_core.is_valid()) {
		return m_core->get_trajectory_query_mode();
	}
	return 0;
}

void MotionMatching::set_path_radius(float p_value) {
	if (m_core.is_valid()) {
		m_core->set_path_radius(p_value);
	}
}

float MotionMatching::get_path_radius() const {
	if (m_core.is_valid()) {
		return m_core->get_path_radius();
	}
	return 1.0f;
}

void MotionMatching::set_path_speed(float p_value) {
	if (m_core.is_valid()) {
		m_core->set_path_speed(p_value);
	}
}

float MotionMatching::get_path_speed() const {
	if (m_core.is_valid()) {
		return m_core->get_path_speed();
	}
	return 1.0f;
}

void MotionMatching::set_normalize_data(bool p_value) {
	if (m_core.is_valid()) {
		m_core->set_normalize_data(p_value);
		notify_property_list_changed();
	}
	m_dataDirty = true;
}

bool MotionMatching::get_normalize_data() const {
	if (m_core.is_valid()) {
		return m_core->get_normalize_data();
	}
	return false;
}

void MotionMatching::set_feature_scaler_type(int64_t p_value) {
	if (m_core.is_valid()) {
		m_core->set_feature_scaler_type(p_value);
		notify_property_list_changed();
	}
	m_dataDirty = true;
}

int64_t MotionMatching::get_feature_scaler_type() const {
	if (m_core.is_valid()) {
		return m_core->get_feature_scaler_type();
	}
	return 0;
}

void MotionMatching::set_feature_min(float p_value) {
	if (m_core.is_valid()) {
		m_core->set_feature_min(p_value);
	}
	m_dataDirty = true;
}

float MotionMatching::get_feature_min() const {
	if (m_core.is_valid()) {
		return m_core->get_feature_min();
	}
	return 0.0f;
}

void MotionMatching::set_feature_max(float p_value) {
	if (m_core.is_valid()) {
		m_core->set_feature_max(p_value);
	}
	m_dataDirty = true;
}

float MotionMatching::get_feature_max() const {
	if (m_core.is_valid()) {
		return m_core->get_feature_max();
	}
	return 1.0f;
}

void MotionMatching::set_clip_features(bool p_value) {
	if (m_core.is_valid()) {
		m_core->set_clip_features(p_value);
	}
	m_dataDirty = true;
}

bool MotionMatching::get_clip_features() const {
	if (m_core.is_valid()) {
		return m_core->get_clip_features();
	}
	return false;
}

void MotionMatching::set_max_kd_tree_depth(int64_t p_value) {
	if (m_core.is_valid()) {
		m_core->set_max_kd_tree_depth(p_value);
	}
	m_dataDirty = true;
}

int64_t MotionMatching::get_max_kd_tree_depth() const {
	if (m_core.is_valid()) {
		return m_core->get_max_kd_tree_depth();
	}
	return 15;
}

void MotionMatching::set_min_frames_per_kd_tree_node(int64_t p_value) {
	if (m_core.is_valid()) {
		m_core->set_min_frames_per_kd_tree_node(p_value);
	}
	m_dataDirty = true;
}

int64_t MotionMatching::get_min_frames_per_kd_tree_node() const {
	if (m_core.is_valid()) {
		return m_core->get_min_frames_per_kd_tree_node();
	}
	return 1000;
}

void MotionMatching::set_mirror(bool p_value) {
	if (m_core.is_valid()) {
		m_core->set_mirror(p_value);
	}
	m_dataDirty = true;
}

bool MotionMatching::get_mirror() const {
	if (m_core.is_valid()) {
		return m_core->get_mirror();
	}
	return false;
}

void MotionMatching::set_feature_schema(const Ref<FeatureSchema> &p_schema) {
	if (m_core.is_valid()) {
		m_core->set_feature_schema(p_schema);
	}
	refresh_joint_hints();
	m_dataDirty = true;
}

Ref<FeatureSchema> MotionMatching::get_feature_schema() const {
	if (m_core.is_valid()) {
		return m_core->get_feature_schema();
	}
	return Ref<FeatureSchema>();
}

void MotionMatching::set_motions(const Array &p_motions) {
	if (m_core.is_valid()) {
		m_core->set_motions(p_motions);
	}
	m_dataDirty = true;
}

Array MotionMatching::get_motions() const {
	if (m_core.is_valid()) {
		return m_core->get_motions();
	}
	return Array();
}

void MotionMatching::set_visualize_feature_schema(bool p_value) {
	if (m_core.is_valid()) {
		m_core->set_visualize_feature_schema(p_value);
		if (m_core->is_visualize_feature_schema() && m_debugVisFeatureSchema.is_null()) {
			Ref<FeatureSchema> s = m_core->get_feature_schema();
			if (s.is_valid()) {
				m_debugVisFeatureSchema = s;
			}
		}
		if (!m_core->is_visualize_feature_schema()) {
			m_debugData.reset();
		}
	}
}

bool MotionMatching::is_visualize_feature_schema() const {
	if (m_core.is_valid()) {
		return m_core->is_visualize_feature_schema();
	}
	return false;
}

void MotionMatching::set_target_position(const Vector3 &p_value) {
	if (m_core.is_valid()) {
		m_core->set_target_position(p_value);
	}
}

Vector3 MotionMatching::get_target_position() const {
	if (m_core.is_valid()) {
		return m_core->get_target_position();
	}
	return Vector3();
}

void MotionMatching::set_target_facing_dir(const Vector3 &p_value) {
	if (m_core.is_valid()) {
		m_core->set_target_facing_dir(p_value);
	}
}

Vector3 MotionMatching::get_target_facing_dir() const {
	if (m_core.is_valid()) {
		return m_core->get_target_facing_dir();
	}
	return Vector3(0.0f, 0.0f, -1.0f);
}

void MotionMatching::set_use_facing_dir(bool p_value) {
	if (m_core.is_valid()) {
		m_core->set_use_facing_dir(p_value);
	}
}

bool MotionMatching::get_use_facing_dir() const {
	if (m_core.is_valid()) {
		return m_core->get_use_facing_dir();
	}
	return false;
}

void MotionMatching::set_skeleton_path(const NodePath &p_path) {
	m_skeletonPath = p_path;
	m_dataDirty = true;
	refresh_joint_hints();
}

NodePath MotionMatching::get_skeleton_path() const {
	return m_skeletonPath;
}

bool MotionMatching::_set(const StringName &p_name, const Variant &p_value) {
	const String key = String(p_name);
	if (key == "skeleton_path") {
		set_skeleton_path(p_value);
		return true;
	}
	if (m_core.is_valid() && m_core->_set(p_name, p_value)) {
		notify_property_list_changed();
		return true;
	}
	return false;
}

bool MotionMatching::_get(const StringName &p_name, Variant &r_ret) const {
	const String key = String(p_name);
	if (key == "skeleton_path") {
		r_ret = m_skeletonPath;
		return true;
	}
	if (m_core.is_valid()) {
		return m_core->_get(p_name, r_ret);
	}
	return false;
}

void MotionMatching::_get_property_list(List<PropertyInfo> *p_list) const {
	if (m_core.is_valid()) {
		m_core->_get_property_list(p_list);
	}
	p_list->push_back(PropertyInfo(Variant::NODE_PATH, "skeleton_path", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "Skeleton3D"));
}

void MotionMatching::_validate_property(PropertyInfo &p_property) const {
	if (m_core.is_valid()) {
		m_core->_validate_property(p_property);
	}
}

void MotionMatching::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_search_frequency", "value"), &MotionMatching::set_search_frequency);
	ClassDB::bind_method(D_METHOD("get_search_frequency"), &MotionMatching::get_search_frequency);
	ClassDB::bind_method(D_METHOD("set_sample_rate", "value"), &MotionMatching::set_sample_rate);
	ClassDB::bind_method(D_METHOD("get_sample_rate"), &MotionMatching::get_sample_rate);
	ClassDB::bind_method(D_METHOD("set_trajectory_query_mode", "value"), &MotionMatching::set_trajectory_query_mode);
	ClassDB::bind_method(D_METHOD("get_trajectory_query_mode"), &MotionMatching::get_trajectory_query_mode);
	ClassDB::bind_method(D_METHOD("set_path_radius", "value"), &MotionMatching::set_path_radius);
	ClassDB::bind_method(D_METHOD("get_path_radius"), &MotionMatching::get_path_radius);
	ClassDB::bind_method(D_METHOD("set_path_speed", "value"), &MotionMatching::set_path_speed);
	ClassDB::bind_method(D_METHOD("get_path_speed"), &MotionMatching::get_path_speed);
	ClassDB::bind_method(D_METHOD("set_normalize_data", "value"), &MotionMatching::set_normalize_data);
	ClassDB::bind_method(D_METHOD("get_normalize_data"), &MotionMatching::get_normalize_data);
	ClassDB::bind_method(D_METHOD("set_feature_scaler_type", "value"), &MotionMatching::set_feature_scaler_type);
	ClassDB::bind_method(D_METHOD("get_feature_scaler_type"), &MotionMatching::get_feature_scaler_type);
	ClassDB::bind_method(D_METHOD("set_feature_min", "value"), &MotionMatching::set_feature_min);
	ClassDB::bind_method(D_METHOD("get_feature_min"), &MotionMatching::get_feature_min);
	ClassDB::bind_method(D_METHOD("set_feature_max", "value"), &MotionMatching::set_feature_max);
	ClassDB::bind_method(D_METHOD("get_feature_max"), &MotionMatching::get_feature_max);
	ClassDB::bind_method(D_METHOD("set_clip_features", "value"), &MotionMatching::set_clip_features);
	ClassDB::bind_method(D_METHOD("get_clip_features"), &MotionMatching::get_clip_features);
	ClassDB::bind_method(D_METHOD("set_max_kd_tree_depth", "value"), &MotionMatching::set_max_kd_tree_depth);
	ClassDB::bind_method(D_METHOD("get_max_kd_tree_depth"), &MotionMatching::get_max_kd_tree_depth);
	ClassDB::bind_method(D_METHOD("set_min_frames_per_kd_tree_node", "value"), &MotionMatching::set_min_frames_per_kd_tree_node);
	ClassDB::bind_method(D_METHOD("get_min_frames_per_kd_tree_node"), &MotionMatching::get_min_frames_per_kd_tree_node);
	ClassDB::bind_method(D_METHOD("set_mirror", "value"), &MotionMatching::set_mirror);
	ClassDB::bind_method(D_METHOD("get_mirror"), &MotionMatching::get_mirror);
	ClassDB::bind_method(D_METHOD("set_feature_schema", "schema"), &MotionMatching::set_feature_schema);
	ClassDB::bind_method(D_METHOD("get_feature_schema"), &MotionMatching::get_feature_schema);
	ClassDB::bind_method(D_METHOD("set_motions", "motions"), &MotionMatching::set_motions);
	ClassDB::bind_method(D_METHOD("get_motions"), &MotionMatching::get_motions);
	ClassDB::bind_method(D_METHOD("set_visualize_feature_schema", "value"), &MotionMatching::set_visualize_feature_schema);
	ClassDB::bind_method(D_METHOD("is_visualize_feature_schema"), &MotionMatching::is_visualize_feature_schema);
	ClassDB::bind_method(D_METHOD("set_target_position", "value"), &MotionMatching::set_target_position);
	ClassDB::bind_method(D_METHOD("get_target_position"), &MotionMatching::get_target_position);
	ClassDB::bind_method(D_METHOD("set_target_facing_dir", "value"), &MotionMatching::set_target_facing_dir);
	ClassDB::bind_method(D_METHOD("get_target_facing_dir"), &MotionMatching::get_target_facing_dir);
	ClassDB::bind_method(D_METHOD("set_use_facing_dir", "value"), &MotionMatching::set_use_facing_dir);
	ClassDB::bind_method(D_METHOD("get_use_facing_dir"), &MotionMatching::get_use_facing_dir);
	ClassDB::bind_method(D_METHOD("set_skeleton_path", "path"), &MotionMatching::set_skeleton_path);
	ClassDB::bind_method(D_METHOD("get_skeleton_path"), &MotionMatching::get_skeleton_path);
	ClassDB::bind_method(D_METHOD("resolve_skeleton"), &MotionMatching::resolve_skeleton);
	ClassDB::bind_method(D_METHOD("set_debug_draw_feature_schema", "feature_schema"), &MotionMatching::SetDebugDrawFeatureSchema);
	ClassDB::bind_method(D_METHOD("get_debug_draw_feature_schema"), &MotionMatching::GetDebugDrawFeatureSchema);
	ClassDB::bind_method(D_METHOD("clear_debug_draw"), &MotionMatching::clear_debug_draw);
	ClassDB::bind_method(D_METHOD("reinit"), &MotionMatching::reinit);
	ClassDB::bind_method(D_METHOD("toggle_debug_draw_schema"), &MotionMatching::toggle_debug_draw_schema);
	ClassDB::bind_method(D_METHOD("is_debug_draw_schema_enabled"), &MotionMatching::is_debug_draw_schema_enabled);
	ClassDB::bind_method(D_METHOD("get_update_time_ms"), &MotionMatching::get_update_time_ms);
	ClassDB::bind_method(D_METHOD("get_post_update_time_ms"), &MotionMatching::get_post_update_time_ms);
	ClassDB::bind_method(D_METHOD("get_output_time_ms"), &MotionMatching::get_output_time_ms);
	ClassDB::bind_method(D_METHOD("_on_core_motion_event", "tag"), &MotionMatching::_on_core_motion_event);

	ADD_SIGNAL(MethodInfo("motion_event_fired", PropertyInfo(Variant::STRING, "tag")));

	ClassDB::bind_integer_constant(get_class_static(), "TrajectoryMode", "TARGET_DRIVEN", TARGET_DRIVEN);
	ClassDB::bind_integer_constant(get_class_static(), "TrajectoryMode", "AUTOMATIC", AUTOMATIC);
	ClassDB::bind_integer_constant(get_class_static(), "ScalerType", "STANDARD", STANDARD);
	ClassDB::bind_integer_constant(get_class_static(), "ScalerType", "MIN_MAX", MIN_MAX);
}

}
