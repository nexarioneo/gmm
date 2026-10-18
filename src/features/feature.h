#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include <godot_cpp/classes/resource.hpp>
#include <godot_cpp/core/property_info.hpp>
#include <godot_cpp/templates/list.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/color.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/packed_string_array.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/string_name.hpp>
#include <godot_cpp/variant/variant.hpp>
#include <godot_cpp/variant/vector2.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include "core/math/feature_matrix.h"
#include "debug/compat.h"
#include "core/math/query_vector.h"

class ActorInstance;
class AnimGraphPosePool;
class Motion;
class MotionInstance;
class Pose;

namespace godot::MotionMatching {
class Frame;
class FrameDatabase;
class MotionMatchingInstance;
class TrajectoryQuery;
class FeatureMatrixTransformer;

class Feature : public Resource {
	GDCLASS(Feature, Resource)

public:
	Feature() = default;
	virtual ~Feature() = default;

	struct InitSettings {
		ActorInstance *m_actorInstance = nullptr;
		FeatureMatrix::Index m_featureColumnStartOffset = 0;
	};
	virtual bool Init(const InitSettings &settings);

	struct ExtractFeatureContext {
		ExtractFeatureContext(FeatureMatrix &featureMatrix, AnimGraphPosePool &posePool);

		FrameDatabase *m_frameDatabase = nullptr;
		FeatureMatrix &m_featureMatrix;

		size_t m_frameIndex = InvalidIndex;
		const Pose *m_framePose = nullptr;
		AnimGraphPosePool &m_posePool;

		ActorInstance *m_actorInstance = nullptr;
	};
	virtual void ExtractFeatureValues(const ExtractFeatureContext &context) = 0;

	struct QueryVectorContext {
		QueryVectorContext(const Pose &currentPose, const TrajectoryQuery &trajectoryQuery);

		const Pose &m_currentPose;
		const TrajectoryQuery &m_trajectoryQuery;
		FeatureMatrixTransformer *m_featureTransformer = nullptr;
	};
	virtual void FillQueryVector([[maybe_unused]] QueryVector &queryVector,
		[[maybe_unused]] const QueryVectorContext &context) = 0;

	struct FrameCostContext {
		FrameCostContext(const QueryVector &queryVector, const FeatureMatrix &featureMatrix);

		const QueryVector &m_queryVector;
		const FeatureMatrix &m_featureMatrix;
	};
	virtual float CalculateFrameCost(size_t frameIndex, const FrameCostContext &context) const;

	enum ResidualType {
		Absolute,
		Squared
	};

	void SetCostFactor(float costFactor) { m_costFactor = costFactor; }
	float GetCostFactor() const { return m_costFactor; }
	virtual void DebugDraw([[maybe_unused]] DebugDrawOutput &debugDisplay,
		[[maybe_unused]] const Pose &currentPose,
		[[maybe_unused]] const FeatureMatrix &featureMatrix,
		[[maybe_unused]] const FeatureMatrixTransformer *featureTransformer,
		[[maybe_unused]] size_t frameIndex) {}

	void SetDebugDrawColor(const Color &color);
	Color GetDebugDrawColor() const;

	void SetDebugDrawEnabled(bool enabled);
	bool GetDebugDrawEnabled() const;

	void SetJointName(const std::string &jointName) { m_jointName = jointName; }
	const std::string &GetJointName() const { return m_jointName; }

	void SetRelativeToJointName(const std::string &jointName) { m_relativeToJointName = jointName; }
	const std::string &GetRelativeToJointName() const { return m_relativeToJointName; }

	void SetName(const std::string &name) { m_name = name; }
	const std::string &GetName() const { return m_name; }

	virtual size_t GetNumDimensions() const = 0;
	virtual std::string GetDimensionName([[maybe_unused]] size_t index) const { return "Unknown"; }
	FeatureMatrix::Index GetColumnOffset() const { return m_featureColumnOffset; }
	void SetColumnOffset(FeatureMatrix::Index offset) { m_featureColumnOffset = offset; }

	const FeatureId &GetId() const { return m_id; }
	size_t GetRelativeToNodeIndex() const { return m_relativeToNodeIndex; }
	void SetRelativeToNodeIndex(size_t nodeIndex);

	String get_feature_id() const { return String(m_id.c_str()); }

	void SetAvailableJoints(const PackedStringArray &p_joints) { m_availableJoints = p_joints; }
	PackedStringArray GetAvailableJoints() const { return m_availableJoints; }

	int64_t get_column_offset() const;
	void set_column_offset(int64_t offset);
	int64_t get_relative_to_node_index() const;
	void set_relative_to_node_index(int64_t node_index);

	void set_feature_name(const String &p_name);
	String get_feature_name() const;
	void set_joint_name_text(const String &p_name);
	String get_joint_name_text() const;
	void set_relative_to_joint_name_text(const String &p_name);
	String get_relative_to_joint_name_text() const;
	void set_residual_type(int64_t p_type);
	int64_t get_residual_type() const;

	bool _set(const StringName &p_name, const Variant &p_value);
	bool _get(const StringName &p_name, Variant &r_ret) const;
	void _get_property_list(List<PropertyInfo> *p_list) const;

protected:

	virtual void AppendCustomProperties(List<PropertyInfo> *p_list) const { (void)p_list; }

	static void _bind_methods();

	float GetNormalizedDirectionDifference(const Vector2 &directionA, const Vector2 &directionB) const;
	float GetNormalizedDirectionDifference(const Vector3 &directionA, const Vector3 &directionB) const;

	float CalcResidual(float value) const;
	float CalcResidual(const Vector3 &a, const Vector3 &b) const;

	virtual bool GetCostFactorVisibility() const;

	static FeatureId GenerateRandomId();

	FeatureId m_id = GenerateRandomId();
	std::string m_name;
	std::string m_jointName;
	std::string m_relativeToJointName;
	Color m_debugColor = Color(0.0f, 1.0f, 0.0f);
	bool m_debugDrawEnabled = false;
	float m_costFactor = 1.0f;
	ResidualType m_residualType = ResidualType::Absolute;

	FeatureMatrix::Index m_featureColumnOffset = 0;
	size_t m_relativeToNodeIndex = InvalidIndex;
	size_t m_jointIndex = InvalidIndex;

	PackedStringArray m_availableJoints;
protected:

	String GetJointHintString() const;
};
}
