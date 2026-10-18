#pragma once

#include <string>

#include "features/feature_schema.h"

namespace godot::MotionMatching {
struct DefaultFeatureSchemaInitSettings {
	std::string m_rootJointName;
	std::string m_leftFootJointName;
	std::string m_rightFootJointName;
	std::string m_pelvisJointName;
};
void DefaultFeatureSchema(FeatureSchema &featureSchema, DefaultFeatureSchemaInitSettings settings);
}
