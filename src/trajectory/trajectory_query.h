#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <godot_cpp/variant/color.hpp>
#include <godot_cpp/variant/quaternion.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include "debug/compat.h"
#include "pose/pose.h"
#include "trajectory/trajectory_history.h"

class ActorInstance;

namespace godot::MotionMatching {
class FeatureTrajectory;

class TrajectoryQuery {
public:
	struct ControlPoint {
		Vector3 m_position;
		Vector3 m_facingDirection;
	};

	enum EMode : uint8_t {
		MODE_TARGETDRIVEN = 0,
		MODE_AUTOMATIC = 1
	};

	void Update(const ActorInstance &actorInstance,
		const FeatureTrajectory *trajectoryFeature,
		const TrajectoryHistory &trajectoryHistory,
		EMode mode,
		const Vector3 &targetPos,
		const Vector3 &targetFacingDir,
		bool useTargetFacingDir,
		float timeDelta,
		float pathRadius,
		float pathSpeed);

	void DebugDraw(DebugDrawOutput &debugDisplay, const Color &color) const;

	const std::vector<ControlPoint> &GetPastControlPoints() const { return m_pastControlPoints; }
	const std::vector<ControlPoint> &GetFutureControlPoints() const { return m_futureControlPoints; }

private:
	static void DebugDrawControlPoints(DebugDrawOutput &debugDisplay,
		const std::vector<ControlPoint> &controlPoints,
		const Color &color);

	void PredictFutureTrajectory(const ActorInstance &actorInstance,
		const FeatureTrajectory *trajectoryFeature,
		const Vector3 &targetPos,
		const Vector3 &targetFacingDir,
		bool useTargetFacingDir);

	std::vector<ControlPoint> m_pastControlPoints;
	std::vector<ControlPoint> m_futureControlPoints;

	float m_positionBias = 2.0f;
	float m_rotationBias = 3.0f;
	float m_deadZone = 0.2f;

	float m_automaticModePhase = 0.0f;
};
}
