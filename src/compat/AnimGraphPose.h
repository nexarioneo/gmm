#pragma once

#include "Pose.h"

class AnimGraphPose {
public:
	AnimGraphPose() = default;
	AnimGraphPose(const AnimGraphPose &) = default;
	AnimGraphPose &operator=(const AnimGraphPose &) = default;
	~AnimGraphPose() = default;

	Pose &GetPose() { return m_pose; }

	const Pose &GetPose() const { return m_pose; }

private:
	Pose m_pose;
};
