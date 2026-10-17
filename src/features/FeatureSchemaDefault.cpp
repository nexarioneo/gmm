#include "FeaturePosition.h"
#include "FeatureTrajectory.h"
#include "FeatureVelocity.h"

#include <godot_cpp/core/memory.hpp>
#include <godot_cpp/variant/color.hpp>

#include "FeatureSchemaDefault.h"

namespace godot::MotionMatching {
void DefaultFeatureSchema(FeatureSchema &featureSchema, DefaultFeatureSchemaInitSettings settings) {
	featureSchema.Clear();
	const std::string rootJointName = settings.m_rootJointName;

	Ref<FeatureTrajectory> rootTrajectory = memnew(FeatureTrajectory);
	rootTrajectory->SetJointName(rootJointName);
	rootTrajectory->SetRelativeToJointName(rootJointName);
	rootTrajectory->SetDebugDrawColor(Color(157.0f / 255.0f, 78.0f / 255.0f, 221.0f / 255.0f, 1.0f));
	rootTrajectory->SetDebugDrawEnabled(true);
	featureSchema.AddFeature(rootTrajectory);

	Ref<FeaturePosition> leftFootPosition = memnew(FeaturePosition);
	leftFootPosition->SetName("Left Foot Position");
	leftFootPosition->SetJointName(settings.m_leftFootJointName);
	leftFootPosition->SetRelativeToJointName(rootJointName);
	leftFootPosition->SetDebugDrawColor(Color(255.0f / 255.0f, 173.0f / 255.0f, 173.0f / 255.0f, 1.0f));
	leftFootPosition->SetDebugDrawEnabled(true);
	featureSchema.AddFeature(leftFootPosition);

	Ref<FeaturePosition> rightFootPosition = memnew(FeaturePosition);
	rightFootPosition->SetName("Right Foot Position");
	rightFootPosition->SetJointName(settings.m_rightFootJointName);
	rightFootPosition->SetRelativeToJointName(rootJointName);
	rightFootPosition->SetDebugDrawColor(Color(253.0f / 255.0f, 255.0f / 255.0f, 182.0f / 255.0f, 1.0f));
	rightFootPosition->SetDebugDrawEnabled(true);
	featureSchema.AddFeature(rightFootPosition);

	Ref<FeatureVelocity> leftFootVelocity = memnew(FeatureVelocity);
	leftFootVelocity->SetName("Left Foot Velocity");
	leftFootVelocity->SetJointName(settings.m_leftFootJointName);
	leftFootVelocity->SetRelativeToJointName(rootJointName);
	leftFootVelocity->SetDebugDrawColor(Color(155.0f / 255.0f, 246.0f / 255.0f, 255.0f / 255.0f, 1.0f));
	leftFootVelocity->SetDebugDrawEnabled(true);
	leftFootVelocity->SetCostFactor(0.75f);
	featureSchema.AddFeature(leftFootVelocity);

	Ref<FeatureVelocity> rightFootVelocity = memnew(FeatureVelocity);
	rightFootVelocity->SetName("Right Foot Velocity");
	rightFootVelocity->SetJointName(settings.m_rightFootJointName);
	rightFootVelocity->SetRelativeToJointName(rootJointName);
	rightFootVelocity->SetDebugDrawColor(Color(189.0f / 255.0f, 178.0f / 255.0f, 255.0f / 255.0f, 1.0f));
	rightFootVelocity->SetDebugDrawEnabled(true);
	rightFootVelocity->SetCostFactor(0.75f);
	featureSchema.AddFeature(rightFootVelocity);

	Ref<FeatureVelocity> pelvisVelocity = memnew(FeatureVelocity);
	pelvisVelocity->SetName("Pelvis Velocity");
	pelvisVelocity->SetJointName(settings.m_pelvisJointName);
	pelvisVelocity->SetRelativeToJointName(rootJointName);
	pelvisVelocity->SetDebugDrawColor(Color(185.0f / 255.0f, 255.0f / 255.0f, 175.0f / 255.0f, 1.0f));
	pelvisVelocity->SetDebugDrawEnabled(true);
	featureSchema.AddFeature(pelvisVelocity);
}
}
