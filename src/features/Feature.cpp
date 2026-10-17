#include "Feature.h"

#include <atomic>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <random>

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/core/math.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#include "Actor.h"
#include "ActorInstance.h"
#include "MmNode.h"
#include "Skeleton.h"
#include "../database/Frame.h"

namespace godot::MotionMatching {

Feature::ExtractFeatureContext::ExtractFeatureContext(FeatureMatrix &featureMatrix, AnimGraphPosePool &posePool)
	: m_featureMatrix(featureMatrix)
	, m_posePool(posePool) {
}

Feature::QueryVectorContext::QueryVectorContext(const Pose &currentPose, const TrajectoryQuery &trajectoryQuery)
	: m_currentPose(currentPose)
	, m_trajectoryQuery(trajectoryQuery) {
}

Feature::FrameCostContext::FrameCostContext(const QueryVector &queryVector, const FeatureMatrix &featureMatrix)
	: m_queryVector(queryVector)
	, m_featureMatrix(featureMatrix) {
}

bool Feature::Init(const InitSettings &settings) {
	if (settings.m_actorInstance == nullptr) {
		return false;
	}
	const Actor *actor = settings.m_actorInstance->GetActor();
	if (actor == nullptr) {
		return false;
	}
	const Skeleton *skeleton = actor->GetSkeleton();
	if (skeleton == nullptr) {
		return false;
	}

	const MmNode *joint = skeleton->FindNodeByNameNoCase(m_jointName.c_str());
	m_jointIndex = joint ? joint->GetNodeIndex() : InvalidIndex;
	if (m_jointIndex == InvalidIndex) {
		UtilityFunctions::push_error(String("Motion Matching: Feature::Init(): Cannot find index for joint named '") + String(m_jointName.c_str()) + String("'."));
		return false;
	}

	const MmNode *relativeToJoint = skeleton->FindNodeByNameNoCase(m_relativeToJointName.c_str());
	m_relativeToNodeIndex = relativeToJoint ? relativeToJoint->GetNodeIndex() : InvalidIndex;
	if (m_relativeToNodeIndex == InvalidIndex) {
		UtilityFunctions::push_error(String("Motion Matching: Feature::Init(): Cannot find index for joint named '") + String(m_relativeToJointName.c_str()) + String("'."));
		return false;
	}

	if (m_name.empty()) {
		const String defaultName = get_class().replace("Feature", "") + " (" + String(m_jointName.c_str()) + ")";
		m_name = std::string(defaultName.utf8().get_data());
	}
	return true;
}

void Feature::SetDebugDrawColor(const Color &color) {
	m_debugColor = color;
}

Color Feature::GetDebugDrawColor() const {
	return m_debugColor;
}

void Feature::SetDebugDrawEnabled(bool enabled) {
	m_debugDrawEnabled = enabled;
}

bool Feature::GetDebugDrawEnabled() const {
	return m_debugDrawEnabled;
}

float Feature::CalculateFrameCost([[maybe_unused]] size_t frameIndex, [[maybe_unused]] const FrameCostContext &context) const {
	assert(false && "Feature::CalculateFrameCost(): Not implemented for the given feature.");
	return 0.0f;
}

void Feature::SetRelativeToNodeIndex(size_t nodeIndex) {
	m_relativeToNodeIndex = nodeIndex;
}

int64_t Feature::get_column_offset() const {
	return static_cast<int64_t>(GetColumnOffset());
}

void Feature::set_feature_name(const String &p_name) {
	m_name = std::string(p_name.utf8().get_data());
}

String Feature::get_feature_name() const {
	return String(m_name.c_str());
}

void Feature::set_joint_name_text(const String &p_name) {
	m_jointName = std::string(p_name.utf8().get_data());
}

String Feature::get_joint_name_text() const {
	return String(m_jointName.c_str());
}

void Feature::set_relative_to_joint_name_text(const String &p_name) {
	m_relativeToJointName = std::string(p_name.utf8().get_data());
}

String Feature::get_relative_to_joint_name_text() const {
	return String(m_relativeToJointName.c_str());
}

void Feature::set_residual_type(int64_t p_type) {
	m_residualType = (p_type == Squared) ? Squared : Absolute;
}

int64_t Feature::get_residual_type() const {
	return static_cast<int64_t>(m_residualType);
}

void Feature::set_column_offset(int64_t offset) {
	SetColumnOffset(static_cast<FeatureMatrix::Index>(offset));
}

int64_t Feature::get_relative_to_node_index() const {
	return static_cast<int64_t>(GetRelativeToNodeIndex());
}

void Feature::set_relative_to_node_index(int64_t node_index) {
	SetRelativeToNodeIndex(static_cast<size_t>(node_index));
}

float Feature::GetNormalizedDirectionDifference(const Vector2 &directionA, const Vector2 &directionB) const {

	const float dotProduct = directionA.normalized().dot(directionB.normalized());
	const float normalizedDirectionDifference = (2.0f - (1.0f + dotProduct)) * 0.5f;
	return Math::abs(normalizedDirectionDifference);
}

float Feature::GetNormalizedDirectionDifference(const Vector3 &directionA, const Vector3 &directionB) const {

	const float dotProduct = directionA.normalized().dot(directionB.normalized());
	const float normalizedDirectionDifference = (2.0f - (1.0f + dotProduct)) * 0.5f;
	return Math::abs(normalizedDirectionDifference);
}

float Feature::CalcResidual(float value) const {
	if (m_residualType == ResidualType::Squared) {
		return value * value;
	}

	return Math::abs(value);
}

float Feature::CalcResidual(const Vector3 &a, const Vector3 &b) const {
	const float euclideanDistance = (b - a).length();
	return CalcResidual(euclideanDistance);
}

bool Feature::GetCostFactorVisibility() const {
	return true;
}

FeatureId Feature::GenerateRandomId() {

	static std::atomic<unsigned long long> s_counter{ 0 };
	thread_local std::mt19937_64 rng(std::random_device{}());
	char buf[33];
	snprintf(buf, sizeof(buf), "%016llx%016llx",
		(unsigned long long)rng(), (unsigned long long)++s_counter);
	return FeatureId(buf);
}

void Feature::_bind_methods() {

	ClassDB::bind_method(D_METHOD("set_cost_factor", "cost_factor"), &Feature::SetCostFactor);
	ClassDB::bind_method(D_METHOD("get_cost_factor"), &Feature::GetCostFactor);
	ClassDB::bind_method(D_METHOD("set_feature_name", "name"), &Feature::set_feature_name);
	ClassDB::bind_method(D_METHOD("get_feature_name"), &Feature::get_feature_name);
	ClassDB::bind_method(D_METHOD("set_joint_name_text", "name"), &Feature::set_joint_name_text);
	ClassDB::bind_method(D_METHOD("get_joint_name_text"), &Feature::get_joint_name_text);
	ClassDB::bind_method(D_METHOD("set_relative_to_joint_name_text", "name"), &Feature::set_relative_to_joint_name_text);
	ClassDB::bind_method(D_METHOD("get_relative_to_joint_name_text"), &Feature::get_relative_to_joint_name_text);
	ClassDB::bind_method(D_METHOD("set_residual_type", "t"), &Feature::set_residual_type);
	ClassDB::bind_method(D_METHOD("get_residual_type"), &Feature::get_residual_type);
	ClassDB::bind_method(D_METHOD("set_available_joints", "joints"), &Feature::SetAvailableJoints);
	ClassDB::bind_method(D_METHOD("get_available_joints"), &Feature::GetAvailableJoints);
	ClassDB::bind_method(D_METHOD("set_debug_draw_enabled", "enabled"), &Feature::SetDebugDrawEnabled);
	ClassDB::bind_method(D_METHOD("get_debug_draw_enabled"), &Feature::GetDebugDrawEnabled);
	ClassDB::bind_method(D_METHOD("set_debug_draw_color", "color"), &Feature::SetDebugDrawColor);
	ClassDB::bind_method(D_METHOD("get_debug_draw_color"), &Feature::GetDebugDrawColor);
	ClassDB::bind_method(D_METHOD("set_column_offset", "offset"), &Feature::set_column_offset);
	ClassDB::bind_method(D_METHOD("get_column_offset"), &Feature::get_column_offset);
	ClassDB::bind_method(D_METHOD("set_relative_to_node_index", "node_index"), &Feature::set_relative_to_node_index);
	ClassDB::bind_method(D_METHOD("get_relative_to_node_index"), &Feature::get_relative_to_node_index);
	ClassDB::bind_method(D_METHOD("get_feature_id"), &Feature::get_feature_id);
}

bool Feature::_set(const StringName &p_name, const Variant &p_value) {
	const String key = String(p_name);
	if (key == "name") {
		m_name = std::string(String(p_value).utf8().get_data());
		return true;
	}
	if (key == "joint_name") {
		m_jointName = std::string(String(p_value).utf8().get_data());
		return true;
	}
	if (key == "relative_to_joint_name") {
		m_relativeToJointName = std::string(String(p_value).utf8().get_data());
		return true;
	}
	if (key == "debug_draw") {
		m_debugDrawEnabled = bool(p_value);
		return true;
	}
	if (key == "debug_color") {
		m_debugColor = Color(p_value);
		return true;
	}
	if (key == "cost_factor") {
		m_costFactor = float(p_value);
		return true;
	}
	if (key == "residual_type") {
		m_residualType = (ResidualType)(int64_t)p_value;
		return true;
	}
	if (key == "id") {
		m_id = FeatureId(std::string(String(p_value).utf8().get_data()));
		return true;
	}
	return false;
}

bool Feature::_get(const StringName &p_name, Variant &r_ret) const {
	const String key = String(p_name);
	if (key == "name") {
		r_ret = String(m_name.c_str());
		return true;
	}
	if (key == "joint_name") {
		r_ret = String(m_jointName.c_str());
		return true;
	}
	if (key == "relative_to_joint_name") {
		r_ret = String(m_relativeToJointName.c_str());
		return true;
	}
	if (key == "debug_draw") {
		r_ret = m_debugDrawEnabled;
		return true;
	}
	if (key == "debug_color") {
		r_ret = m_debugColor;
		return true;
	}
	if (key == "cost_factor") {
		r_ret = m_costFactor;
		return true;
	}
	if (key == "residual_type") {
		r_ret = (int64_t)m_residualType;
		return true;
	}
	if (key == "id") {
		r_ret = String(m_id.c_str());
		return true;
	}
	return false;
}

String Feature::GetJointHintString() const {

	String hint;
	for (int64_t i = 0; i < m_availableJoints.size(); ++i) {
		hint += "," + m_availableJoints[i];
	}
	return hint;
}

void Feature::_get_property_list(List<PropertyInfo> *p_list) const {

	p_list->push_back(PropertyInfo(Variant::STRING, "name"));

	const bool hasJoints = !m_availableJoints.is_empty();
	p_list->push_back(PropertyInfo(Variant::STRING, "joint_name",
		hasJoints ? PROPERTY_HINT_ENUM_SUGGESTION : PROPERTY_HINT_NONE,
		hasJoints ? GetJointHintString() : "Joint: The joint to extract the data from."));
	p_list->push_back(PropertyInfo(Variant::STRING, "relative_to_joint_name",
		hasJoints ? PROPERTY_HINT_ENUM_SUGGESTION : PROPERTY_HINT_NONE,
		hasJoints ? GetJointHintString() : "Relative To Joint: When extracting feature data, convert it to relative-space to the given joint."));
	p_list->push_back(PropertyInfo(Variant::BOOL, "debug_draw", PROPERTY_HINT_NONE,
		"Debug Draw: Are debug visualizations enabled for this feature?"));
	p_list->push_back(PropertyInfo(Variant::COLOR, "debug_color", PROPERTY_HINT_NONE,
		"Debug Draw Color: Color used for debug visualizations to identify the feature."));

	uint32_t cost_usage = PROPERTY_USAGE_DEFAULT;
	if (!GetCostFactorVisibility()) {
		cost_usage = PROPERTY_USAGE_STORAGE;
	}
	p_list->push_back(PropertyInfo(Variant::FLOAT, "cost_factor", PROPERTY_HINT_RANGE, "0,100,0.1", cost_usage));

	p_list->push_back(PropertyInfo(Variant::INT, "residual_type", PROPERTY_HINT_ENUM, "Absolute,Squared"));

	p_list->push_back(PropertyInfo(Variant::STRING, "id", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_STORAGE));

	AppendCustomProperties(p_list);
}

}
