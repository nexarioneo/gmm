#include "MotionMatchingNode.h"

#include <cstdint>

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/memory.hpp>
#include <godot_cpp/classes/object.hpp>
#include <godot_cpp/classes/animation.hpp>
#include <godot_cpp/classes/time.hpp>
#include <godot_cpp/variant/color.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#include "../features/FeatureSchemaDefault.h"
#include "../database/MotionMatchingData.h"
#include "../runtime/EventData.h"
#include "../runtime/MotionMatchingInstance.h"
#include "../debug/MotionMatchingCompat.h"

#include "ActorInstance.h"
#include "AnimGraphInstance.h"
#include "Motion.h"
#include "MotionInstance.h"
#include "MotionSet.h"
#include "Pose.h"
#include "Transform.h"

#ifdef GMM_MONITOR_ENABLED

#include "../debug/MotionMatchingMonitorBus.h"
#endif

namespace godot::MotionMatching {

MotionMatchingNode::MotionMatchingNode() {

	add_input(INPUT_TARGET_POS);
	add_input(INPUT_TARGET_FACING_DIR);
	add_input(INPUT_USE_FACING_DIR);

}

MotionMatchingNode::~MotionMatchingNode() {

	if (m_visualizeFeatureSchema) {
		m_visualizeFeatureSchema = false;

	}
}

double MotionMatchingNode::_process(double p_time, bool p_seek, bool p_is_external_seeking, bool p_test_only) {
	(void)p_seek;
	(void)p_is_external_seeking;
	(void)p_test_only;

	const float delta = static_cast<float>(p_time);
	if (m_dataDirty) {
		reinit();
	}
	Vector3 facing = m_targetFacingDir;
	facing.y = 0.0f;
	facing = NormalizedSafe(facing);
	Update(delta, m_targetPos, facing, m_useFacingDir);
	PostUpdate(delta);
	Output();
	return static_cast<double>(m_currentPlayTime);
}

void MotionMatchingNode::UniqueData::Clear() {
	if (m_instance) {
		memdelete(m_instance);
		m_instance = nullptr;
	}
	if (m_data) {
		memdelete(m_data);
		m_data = nullptr;
	}
	m_motionSet.Clear();
	m_hasError = false;
}

void MotionMatchingNode::SetBuildSkeleton(Skeleton3D *skeleton, Node3D *root) {
	m_buildSkeleton = skeleton;
	m_buildRoot = root;
}

void MotionMatchingNode::UniqueData::Update(MotionMatchingNode *node) {

	Clear();
	if (node == nullptr || node->m_featureSchema.is_null()) {
		m_hasError = true;
		return;
	}

	m_data = memnew(MotionMatchingData(*node->m_featureSchema.ptr()));
	m_instance = memnew(MotionMatchingInstance());

	if (node->m_featureSchema->GetNumFeatures() == 0) {
		UtilityFunctions::push_warning("Motion Matching: Feature schema is empty; add features (e.g. FeaturePosition) to build the database.");
		m_hasError = true;
		return;
	}

	ActorInstance *actorInstance = nullptr;
	if (node->m_buildSkeleton != nullptr) {
		m_actorInstance.LinkActor(node->m_buildSkeleton,
			node->m_buildRoot != nullptr ? node->m_buildRoot : node->m_buildSkeleton);
		actorInstance = &m_actorInstance;
	} else if (node->m_motions.is_empty() && node->m_featureSchema.is_null()) {

		m_hasError = true;
		return;
	} else {
		UtilityFunctions::push_warning("Motion Matching: No skeleton linked; set Skeleton Path on the MotionMatching node.");
		m_hasError = true;
		return;
	}

	Time *timeSingleton = Time::get_singleton();
	const uint64_t timeBefore = timeSingleton ? timeSingleton->get_ticks_msec() : 0;

	UtilityFunctions::print("Motion Matching: Importing motion database...");
	MotionMatchingData::InitSettings settings;
	settings.m_actorInstance = actorInstance;
	settings.m_frameImportSettings.m_sampleRate = node->m_sampleRate;
	settings.m_importMirrored = node->m_mirror;
	settings.m_maxKdTreeDepth = node->m_maxKdTreeDepth;
	settings.m_minFramesPerKdTreeNode = node->m_minFramesPerKdTreeNode;
	settings.m_normalizeData = node->m_normalizeData;
	settings.m_featureScalerType = node->m_featureScalerType;
	settings.m_featureTansformerSettings.m_featureMin = node->m_featureMin;
	settings.m_featureTansformerSettings.m_featureMax = node->m_featureMax;
	settings.m_featureTansformerSettings.m_clip = node->m_clipFeatures;

	if (node->m_motions.is_empty()) {
		UtilityFunctions::push_warning("Motion Matching: No motions assigned to MotionMatchingNode; motion database will be empty.");
	} else {
		for (int i = 0; i < node->m_motions.size(); ++i) {
			Animation *animation = Object::cast_to<Animation>(node->m_motions[i]);
			if (animation == nullptr) {
				UtilityFunctions::push_warning("Motion Matching: Motions entry is not an Animation; skipped.");
				continue;
			}
			Motion *motion = m_motionSet.AddMotion(animation);
			if (motion != nullptr) {
				settings.m_motionList.emplace_back(motion);
			} else {
				UtilityFunctions::push_warning("Motion Matching: Failed to register motion for motions entry.");
			}
		}
	}

	UtilityFunctions::print("Motion Matching: Initializing motion matching...");
	if (!m_data->Init(settings)) {
		UtilityFunctions::push_error("Motion Matching: Failed to initialize motion matching for MotionMatchingNode!");
		m_hasError = true;
		return;
	}

	UtilityFunctions::print("Motion Matching: Initializing instance...");
	MotionMatchingInstance::InitSettings initSettings;
	initSettings.m_actorInstance = actorInstance;
	initSettings.m_data = m_data;
	m_instance->Init(initSettings);

	Time *timeSingletonAfter = Time::get_singleton();
	const uint64_t timeAfter = timeSingletonAfter ? timeSingletonAfter->get_ticks_msec() : timeBefore;
	const float initTime = static_cast<float>(timeAfter >= timeBefore ? timeAfter - timeBefore : 0) / 1000.0f;
	const size_t memUsage = m_data->GetFrameDatabase().CalcMemoryUsageInBytes();
	UtilityFunctions::print(String("Motion Matching: Finished in ") + String::num(initTime, 2) + String(" seconds (mem usage=") + String::num_int64((int64_t)memUsage) + String(" bytes or ") + String::num(static_cast<float>(memUsage) / (1024.0f * 1024.0f), 2) + String(" mb)"));

	m_hasError = false;
}

void MotionMatchingNode::Update(float timePassedInSeconds, const Vector3 &targetPos, const Vector3 &targetFacingDir, bool useFacingDir) {
	StampTick();

	if (m_uniqueData.m_hasError || m_uniqueData.m_instance == nullptr) {
		m_updateTimeInMs = 0.0f;
		m_postUpdateTimeInMs = 0.0f;
		m_outputTimeInMs = 0.0f;
		return;
	}

	MotionMatchingInstance *instance = m_uniqueData.m_instance;
	instance->Update(timePassedInSeconds, targetPos, targetFacingDir, useFacingDir, m_trajectoryQueryMode, m_pathRadius, m_pathSpeed);

	if (instance->GetMotionInstance()) {
		m_preSyncTime = instance->GetMotionInstance()->GetCurrentTime();
	}
	m_currentPlayTime = instance->GetNewMotionTime();

	if (m_preSyncTime > m_currentPlayTime) {
		m_preSyncTime = m_currentPlayTime;
	}

	m_updateTimeInMs = TickDeltaSeconds() * 1000.0f;
}

void MotionMatchingNode::PostUpdate(float timePassedInSeconds) {
	StampTick();

	if (m_uniqueData.m_hasError || m_uniqueData.m_instance == nullptr) {
		return;
	}
	MotionMatchingInstance *instance = m_uniqueData.m_instance;

	MotionInstance *motionInstance = instance->GetMotionInstance();
	if (motionInstance) {

		m_eventBuffer.Clear();
		motionInstance->UpdateByTimeValues(m_preSyncTime, m_currentPlayTime, m_eventBuffer);
		m_currentPlayTime = motionInstance->GetCurrentTime();

		for (size_t i = 0; i < m_eventBuffer.GetNumFired(); ++i) {
			const MotionEventBuffer::FiredEvent &fired = m_eventBuffer.GetFired(i);
			if (fired.data) {
				if (const TagEventData *tag = dynamic_cast<const TagEventData *>(fired.data.get())) {
					emit_signal("motion_event_fired", tag->get_tag());
				}
			}
		}
		m_eventBuffer.Clear();
	}

	instance->PostUpdate(timePassedInSeconds);

	m_trajectoryDelta = instance->GetMotionExtractionDelta();
	m_postUpdateTimeInMs = TickDeltaSeconds() * 1000.0f;
}

void MotionMatchingNode::Output() {
	StampTick();

	if (m_uniqueData.m_hasError || m_uniqueData.m_instance == nullptr) {
		return;
	}

	MotionMatchingInstance *instance = m_uniqueData.m_instance;
	instance->SetLowestCostSearchFrequency(m_lowestCostSearchFrequency);

	ActorInstance *actorInstance = instance->GetActorInstance();
	AnimGraphPose *outputPose = m_posePool.RequestPose(actorInstance);
	if (outputPose != nullptr) {
		if (actorInstance != nullptr) {
			outputPose->GetPose().InitFromBindPose(actorInstance);
		}
		instance->Output(outputPose->GetPose());
		m_outputPose = outputPose->GetPose();
		m_posePool.FreePose(outputPose);
	}

	m_outputTimeInMs = TickDeltaSeconds() * 1000.0f;
	{
#ifdef GMM_MONITOR_ENABLED
		MotionMatchingMonitorRequestBus::Broadcast(&MotionMatchingMonitorRequests::PushPerformanceHistogramValue, "Update", m_updateTimeInMs);
		MotionMatchingMonitorRequestBus::Broadcast(&MotionMatchingMonitorRequests::PushPerformanceHistogramValue, "Post Update", m_postUpdateTimeInMs);
		MotionMatchingMonitorRequestBus::Broadcast(&MotionMatchingMonitorRequests::PushPerformanceHistogramValue, "Output", m_outputTimeInMs);
#endif
	}
}

bool MotionMatchingNode::IsTrajectoryPathVisible() const {

	return m_trajectoryQueryMode != TrajectoryQuery::MODE_TARGETDRIVEN;
}

bool MotionMatchingNode::IsFeatureScalerTypeVisible() const {

	return m_normalizeData;
}

bool MotionMatchingNode::IsMinMaxVisible() const {

	return m_normalizeData && m_featureScalerType == MotionMatchingData::MinMaxScalerType;
}

void MotionMatchingNode::reinit() {

	m_uniqueData.Update(this);
	m_dataDirty = false;
}

void MotionMatchingNode::toggle_debug_draw_schema() {

	m_visualizeFeatureSchema = !m_visualizeFeatureSchema;
}

void MotionMatchingNode::StampTick() {
	Time *timeSingleton = Time::get_singleton();
	m_tickStartMsec = timeSingleton ? timeSingleton->get_ticks_msec() : 0;
}

float MotionMatchingNode::TickDeltaSeconds() const {
	Time *timeSingleton = Time::get_singleton();
	if (!timeSingleton || timeSingleton->get_ticks_msec() < m_tickStartMsec) {
		return 0.0f;
	}
	return static_cast<float>(timeSingleton->get_ticks_msec() - m_tickStartMsec) / 1000.0f;
}

void MotionMatchingNode::set_search_frequency(float p_value) {
	m_lowestCostSearchFrequency = (p_value > 0.0f) ? p_value : 5.0f;
}

float MotionMatchingNode::get_search_frequency() const {
	return m_lowestCostSearchFrequency;
}

void MotionMatchingNode::set_sample_rate(int64_t p_value) {
	if (p_value < 1) {
		m_sampleRate = 1;
	} else if (p_value > 240) {
		m_sampleRate = 240;
	} else {
		m_sampleRate = static_cast<size_t>(p_value);
	}
	m_dataDirty = true;
}

int64_t MotionMatchingNode::get_sample_rate() const {
	return static_cast<int64_t>(m_sampleRate);
}

void MotionMatchingNode::set_trajectory_query_mode(int64_t p_value) {
	m_trajectoryQueryMode = static_cast<TrajectoryQuery::EMode>(p_value);
	notify_property_list_changed();
}

int64_t MotionMatchingNode::get_trajectory_query_mode() const {
	return static_cast<int64_t>(m_trajectoryQueryMode);
}

void MotionMatchingNode::set_path_radius(float p_value) {
	m_pathRadius = p_value;
}

float MotionMatchingNode::get_path_radius() const {
	return m_pathRadius;
}

void MotionMatchingNode::set_path_speed(float p_value) {
	m_pathSpeed = p_value;
}

float MotionMatchingNode::get_path_speed() const {
	return m_pathSpeed;
}

void MotionMatchingNode::set_normalize_data(bool p_value) {
	m_normalizeData = p_value;
	notify_property_list_changed();
	m_dataDirty = true;
}

bool MotionMatchingNode::get_normalize_data() const {
	return m_normalizeData;
}

void MotionMatchingNode::set_feature_scaler_type(int64_t p_value) {
	m_featureScalerType = static_cast<MotionMatchingData::FeatureScalerType>(p_value);
	notify_property_list_changed();
	m_dataDirty = true;
}

int64_t MotionMatchingNode::get_feature_scaler_type() const {
	return static_cast<int64_t>(m_featureScalerType);
}

void MotionMatchingNode::set_feature_min(float p_value) {
	m_featureMin = p_value;
	m_dataDirty = true;
}

float MotionMatchingNode::get_feature_min() const {
	return m_featureMin;
}

void MotionMatchingNode::set_feature_max(float p_value) {
	m_featureMax = p_value;
	m_dataDirty = true;
}

float MotionMatchingNode::get_feature_max() const {
	return m_featureMax;
}

void MotionMatchingNode::set_clip_features(bool p_value) {
	m_clipFeatures = p_value;
	m_dataDirty = true;
}

bool MotionMatchingNode::get_clip_features() const {
	return m_clipFeatures;
}

void MotionMatchingNode::set_max_kd_tree_depth(int64_t p_value) {
	m_maxKdTreeDepth = static_cast<size_t>(p_value);
	m_dataDirty = true;
}

int64_t MotionMatchingNode::get_max_kd_tree_depth() const {
	return static_cast<int64_t>(m_maxKdTreeDepth);
}

void MotionMatchingNode::set_min_frames_per_kd_tree_node(int64_t p_value) {
	m_minFramesPerKdTreeNode = static_cast<size_t>(p_value);
	m_dataDirty = true;
}

int64_t MotionMatchingNode::get_min_frames_per_kd_tree_node() const {
	return static_cast<int64_t>(m_minFramesPerKdTreeNode);
}

void MotionMatchingNode::set_mirror(bool p_value) {

	m_mirror = p_value;
	m_dataDirty = true;
}

bool MotionMatchingNode::get_mirror() const {
	return m_mirror;
}

void MotionMatchingNode::set_feature_schema(const Ref<FeatureSchema> &p_schema) {
	m_featureSchema = p_schema;
	m_dataDirty = true;
}

Ref<FeatureSchema> MotionMatchingNode::get_feature_schema() const {
	return m_featureSchema;
}

void MotionMatchingNode::set_motions(const Array &p_motions) {
	m_motions = p_motions;
	m_dataDirty = true;
}

Array MotionMatchingNode::get_motions() const {
	return m_motions;
}

void MotionMatchingNode::set_visualize_feature_schema(bool p_value) {
	m_visualizeFeatureSchema = p_value;
}

bool MotionMatchingNode::is_visualize_feature_schema() const {
	return m_visualizeFeatureSchema;
}

void MotionMatchingNode::set_target_position(const Vector3 &p_value) {
	m_targetPos = p_value;
}

Vector3 MotionMatchingNode::get_target_position() const {
	return m_targetPos;
}

void MotionMatchingNode::set_target_facing_dir(const Vector3 &p_value) {
	m_targetFacingDir = p_value;
}

Vector3 MotionMatchingNode::get_target_facing_dir() const {
	return m_targetFacingDir;
}

void MotionMatchingNode::set_use_facing_dir(bool p_value) {
	m_useFacingDir = p_value;
}

bool MotionMatchingNode::get_use_facing_dir() const {
	return m_useFacingDir;
}

bool MotionMatchingNode::_set(const StringName &p_name, const Variant &p_value) {
	const String key = String(p_name);
	if (key == "search_frequency") {
		set_search_frequency(float(p_value));
		return true;
	}
	if (key == "sample_rate") {
		set_sample_rate(int64_t(p_value));
		return true;
	}
	if (key == "trajectory_query_mode") {
		set_trajectory_query_mode(int64_t(p_value));
		return true;
	}
	if (key == "path_radius") {
		set_path_radius(float(p_value));
		return true;
	}
	if (key == "path_speed") {
		set_path_speed(float(p_value));
		return true;
	}
	if (key == "normalize_data") {
		set_normalize_data(bool(p_value));
		return true;
	}
	if (key == "feature_scaler_type") {
		set_feature_scaler_type(int64_t(p_value));
		return true;
	}
	if (key == "feature_min") {
		set_feature_min(float(p_value));
		return true;
	}
	if (key == "feature_max") {
		set_feature_max(float(p_value));
		return true;
	}
	if (key == "clip_features") {
		set_clip_features(bool(p_value));
		return true;
	}
	if (key == "max_kd_tree_depth") {
		set_max_kd_tree_depth(int64_t(p_value));
		return true;
	}
	if (key == "min_frames_per_kd_tree_node") {
		set_min_frames_per_kd_tree_node(int64_t(p_value));
		return true;
	}
	if (key == "mirror") {
		set_mirror(bool(p_value));
		return true;
	}
	if (key == "feature_schema") {
		Object *obj = p_value;
		set_feature_schema(Ref<FeatureSchema>(Object::cast_to<FeatureSchema>(obj)));
		return true;
	}
	if (key == "motions") {
		set_motions(Array(p_value));
		return true;
	}
	if (key == "visualize_feature_schema") {
		set_visualize_feature_schema(bool(p_value));
		return true;
	}
	return false;
}

bool MotionMatchingNode::_get(const StringName &p_name, Variant &r_ret) const {
	const String key = String(p_name);
	if (key == "search_frequency") {
		r_ret = m_lowestCostSearchFrequency;
		return true;
	}
	if (key == "sample_rate") {
		r_ret = static_cast<int64_t>(m_sampleRate);
		return true;
	}
	if (key == "trajectory_query_mode") {
		r_ret = static_cast<int64_t>(m_trajectoryQueryMode);
		return true;
	}
	if (key == "path_radius") {
		r_ret = m_pathRadius;
		return true;
	}
	if (key == "path_speed") {
		r_ret = m_pathSpeed;
		return true;
	}
	if (key == "normalize_data") {
		r_ret = m_normalizeData;
		return true;
	}
	if (key == "feature_scaler_type") {
		r_ret = static_cast<int64_t>(m_featureScalerType);
		return true;
	}
	if (key == "feature_min") {
		r_ret = m_featureMin;
		return true;
	}
	if (key == "feature_max") {
		r_ret = m_featureMax;
		return true;
	}
	if (key == "clip_features") {
		r_ret = m_clipFeatures;
		return true;
	}
	if (key == "max_kd_tree_depth") {
		r_ret = static_cast<int64_t>(m_maxKdTreeDepth);
		return true;
	}
	if (key == "min_frames_per_kd_tree_node") {
		r_ret = static_cast<int64_t>(m_minFramesPerKdTreeNode);
		return true;
	}
	if (key == "mirror") {
		r_ret = m_mirror;
		return true;
	}
	if (key == "feature_schema") {
		r_ret = m_featureSchema.is_null() ? Variant() : Variant(m_featureSchema.ptr());
		return true;
	}
	if (key == "motions") {
		r_ret = m_motions;
		return true;
	}
	if (key == "visualize_feature_schema") {
		r_ret = m_visualizeFeatureSchema;
		return true;
	}
	return false;
}

void MotionMatchingNode::_get_property_list(List<PropertyInfo> *p_list) const {

	p_list->push_back(PropertyInfo(Variant::FLOAT, "search_frequency", PROPERTY_HINT_RANGE, "0.001,1e38,0.05"));
	p_list->push_back(PropertyInfo(Variant::INT, "sample_rate", PROPERTY_HINT_RANGE, "1,240,1"));
	p_list->push_back(PropertyInfo(Variant::INT, "trajectory_query_mode", PROPERTY_HINT_ENUM, "Target-driven,Automatic (Demo)"));
	p_list->push_back(PropertyInfo(Variant::FLOAT, "path_radius", PROPERTY_HINT_RANGE, "0.0001,1e38,0.01"));
	p_list->push_back(PropertyInfo(Variant::FLOAT, "path_speed", PROPERTY_HINT_RANGE, "0.0001,1e38,0.01"));
	p_list->push_back(PropertyInfo(Variant::NIL, "Data Normalization", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_GROUP));
	p_list->push_back(PropertyInfo(Variant::BOOL, "normalize_data", PROPERTY_HINT_NONE,
		"Normalize Data: Normalize feature data for more intuitive control over weighting the cost factors."));
	p_list->push_back(PropertyInfo(Variant::INT, "feature_scaler_type", PROPERTY_HINT_ENUM, "Standard Scaler,Min-max Scaler"));
	p_list->push_back(PropertyInfo(Variant::FLOAT, "feature_min", PROPERTY_HINT_NONE,
		"Feature Minimum: Minimum value after data transformation."));
	p_list->push_back(PropertyInfo(Variant::FLOAT, "feature_max", PROPERTY_HINT_NONE,
		"Feature Maximum: Maximum value after data transformation."));
	p_list->push_back(PropertyInfo(Variant::BOOL, "clip_features", PROPERTY_HINT_NONE,
		"Clip Features: Clip feature values for outliers to the above range."));
	p_list->push_back(PropertyInfo(Variant::NIL, "Acceleration Structure", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_GROUP));
	p_list->push_back(PropertyInfo(Variant::INT, "max_kd_tree_depth", PROPERTY_HINT_RANGE, "1,20,1"));
	p_list->push_back(PropertyInfo(Variant::INT, "min_frames_per_kd_tree_node", PROPERTY_HINT_RANGE, "1,100000,1"));
	p_list->push_back(PropertyInfo(Variant::BOOL, "mirror"));
	p_list->push_back(PropertyInfo(Variant::OBJECT, "feature_schema", PROPERTY_HINT_RESOURCE_TYPE, "FeatureSchema"));
	p_list->push_back(PropertyInfo(Variant::BOOL, "visualize_feature_schema", PROPERTY_HINT_NONE,
		"Visualize Feature Schema: Toggle debug draw of the feature schema."));
	p_list->push_back(PropertyInfo(Variant::ARRAY, "motions", PROPERTY_HINT_NONE,
		"Motions: Animations to import frames from (O3DE: MotionSet motion ids)."));
}

void MotionMatchingNode::_validate_property(PropertyInfo &p_property) const {

	const String pname = p_property.name;
	if (pname == "path_radius" || pname == "path_speed") {
		if (!IsTrajectoryPathVisible()) {
			p_property.usage = PROPERTY_USAGE_NO_EDITOR;
		}
	} else if (pname == "feature_scaler_type") {
		if (!IsFeatureScalerTypeVisible()) {
			p_property.usage = PROPERTY_USAGE_NO_EDITOR;
		}
	} else if (pname == "feature_min" || pname == "feature_max" || pname == "clip_features") {
		if (!IsMinMaxVisible()) {
			p_property.usage = PROPERTY_USAGE_NO_EDITOR;
		}
	}
}

void MotionMatchingNode::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_search_frequency", "value"), &MotionMatchingNode::set_search_frequency);
	ClassDB::bind_method(D_METHOD("get_search_frequency"), &MotionMatchingNode::get_search_frequency);
	ClassDB::bind_method(D_METHOD("set_sample_rate", "value"), &MotionMatchingNode::set_sample_rate);
	ClassDB::bind_method(D_METHOD("get_sample_rate"), &MotionMatchingNode::get_sample_rate);
	ClassDB::bind_method(D_METHOD("set_trajectory_query_mode", "value"), &MotionMatchingNode::set_trajectory_query_mode);
	ClassDB::bind_method(D_METHOD("get_trajectory_query_mode"), &MotionMatchingNode::get_trajectory_query_mode);
	ClassDB::bind_method(D_METHOD("set_path_radius", "value"), &MotionMatchingNode::set_path_radius);
	ClassDB::bind_method(D_METHOD("get_path_radius"), &MotionMatchingNode::get_path_radius);
	ClassDB::bind_method(D_METHOD("set_path_speed", "value"), &MotionMatchingNode::set_path_speed);
	ClassDB::bind_method(D_METHOD("get_path_speed"), &MotionMatchingNode::get_path_speed);
	ClassDB::bind_method(D_METHOD("set_normalize_data", "value"), &MotionMatchingNode::set_normalize_data);
	ClassDB::bind_method(D_METHOD("get_normalize_data"), &MotionMatchingNode::get_normalize_data);
	ClassDB::bind_method(D_METHOD("set_feature_scaler_type", "value"), &MotionMatchingNode::set_feature_scaler_type);
	ClassDB::bind_method(D_METHOD("get_feature_scaler_type"), &MotionMatchingNode::get_feature_scaler_type);
	ClassDB::bind_method(D_METHOD("set_feature_min", "value"), &MotionMatchingNode::set_feature_min);
	ClassDB::bind_method(D_METHOD("get_feature_min"), &MotionMatchingNode::get_feature_min);
	ClassDB::bind_method(D_METHOD("set_feature_max", "value"), &MotionMatchingNode::set_feature_max);
	ClassDB::bind_method(D_METHOD("get_feature_max"), &MotionMatchingNode::get_feature_max);
	ClassDB::bind_method(D_METHOD("set_clip_features", "value"), &MotionMatchingNode::set_clip_features);
	ClassDB::bind_method(D_METHOD("get_clip_features"), &MotionMatchingNode::get_clip_features);
	ClassDB::bind_method(D_METHOD("set_max_kd_tree_depth", "value"), &MotionMatchingNode::set_max_kd_tree_depth);
	ClassDB::bind_method(D_METHOD("get_max_kd_tree_depth"), &MotionMatchingNode::get_max_kd_tree_depth);
	ClassDB::bind_method(D_METHOD("set_min_frames_per_kd_tree_node", "value"), &MotionMatchingNode::set_min_frames_per_kd_tree_node);
	ClassDB::bind_method(D_METHOD("get_min_frames_per_kd_tree_node"), &MotionMatchingNode::get_min_frames_per_kd_tree_node);
	ClassDB::bind_method(D_METHOD("set_mirror", "value"), &MotionMatchingNode::set_mirror);
	ClassDB::bind_method(D_METHOD("get_mirror"), &MotionMatchingNode::get_mirror);
	ClassDB::bind_method(D_METHOD("set_feature_schema", "schema"), &MotionMatchingNode::set_feature_schema);
	ClassDB::bind_method(D_METHOD("get_feature_schema"), &MotionMatchingNode::get_feature_schema);
	ClassDB::bind_method(D_METHOD("set_motions", "motions"), &MotionMatchingNode::set_motions);
	ClassDB::bind_method(D_METHOD("get_motions"), &MotionMatchingNode::get_motions);
	ClassDB::bind_method(D_METHOD("set_visualize_feature_schema", "value"), &MotionMatchingNode::set_visualize_feature_schema);
	ClassDB::bind_method(D_METHOD("is_visualize_feature_schema"), &MotionMatchingNode::is_visualize_feature_schema);
	ClassDB::bind_method(D_METHOD("set_target_position", "value"), &MotionMatchingNode::set_target_position);
	ClassDB::bind_method(D_METHOD("get_target_position"), &MotionMatchingNode::get_target_position);
	ClassDB::bind_method(D_METHOD("set_target_facing_dir", "value"), &MotionMatchingNode::set_target_facing_dir);
	ClassDB::bind_method(D_METHOD("get_target_facing_dir"), &MotionMatchingNode::get_target_facing_dir);
	ClassDB::bind_method(D_METHOD("set_use_facing_dir", "value"), &MotionMatchingNode::set_use_facing_dir);
	ClassDB::bind_method(D_METHOD("get_use_facing_dir"), &MotionMatchingNode::get_use_facing_dir);
	ClassDB::bind_method(D_METHOD("reinit"), &MotionMatchingNode::reinit);
	ClassDB::bind_method(D_METHOD("toggle_debug_draw_schema"), &MotionMatchingNode::toggle_debug_draw_schema);
	ClassDB::bind_method(D_METHOD("is_debug_draw_schema_enabled"), &MotionMatchingNode::is_debug_draw_schema_enabled);
	ClassDB::bind_method(D_METHOD("get_update_time_ms"), &MotionMatchingNode::get_update_time_ms);
	ClassDB::bind_method(D_METHOD("get_post_update_time_ms"), &MotionMatchingNode::get_post_update_time_ms);
	ClassDB::bind_method(D_METHOD("get_output_time_ms"), &MotionMatchingNode::get_output_time_ms);

	ClassDB::bind_integer_constant(get_class_static(), "TrajectoryMode", "TARGET_DRIVEN", TARGET_DRIVEN);
	ClassDB::bind_integer_constant(get_class_static(), "TrajectoryMode", "AUTOMATIC", AUTOMATIC);
	ClassDB::bind_integer_constant(get_class_static(), "ScalerType", "STANDARD", STANDARD);
	ClassDB::bind_integer_constant(get_class_static(), "ScalerType", "MIN_MAX", MIN_MAX);

	ADD_SIGNAL(MethodInfo("motion_event_fired", PropertyInfo(Variant::STRING, "tag")));
}

}
