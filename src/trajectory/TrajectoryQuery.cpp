#include "TrajectoryQuery.h"

#include <cassert>
#include <cmath>
#include <limits>

#include <godot_cpp/core/math.hpp>

#include "ActorInstance.h"
#include "../features/FeatureTrajectory.h"
#include "Transform.h"

namespace godot::MotionMatching {

static Quaternion ShortestArc(const Vector3 &from, const Vector3 &to) {
	if (from.length() < std::numeric_limits<float>::epsilon() || to.length() < std::numeric_limits<float>::epsilon()) {
		return Quaternion();
	}
	return Quaternion(from, to);
}

Vector3 SampleFunction(float offset, float radius, float phase) {
	phase += 10.7f;
	Vector3 displacement;

	displacement.x = radius * sinf(phase * 0.7f + offset) + radius * 0.75f * cosf(phase * 2.0f + offset * 2.0f);
	displacement.z = radius * cosf(phase * 0.4f + offset);
	return displacement;
}

void TrajectoryQuery::PredictFutureTrajectory(const ActorInstance &actorInstance,
	const FeatureTrajectory *trajectoryFeature,
	const Vector3 &targetPos,
	const Vector3 &targetFacingDir,
	bool useTargetFacingDir) {
	const Vector3 actorInstanceWorldPosition = actorInstance.GetWorldSpaceTransform().m_position;
	const Quaternion actorInstanceWorldRotation = actorInstance.GetWorldSpaceTransform().m_rotation;
	const Vector3 actorInstanceToTarget = (targetPos - actorInstanceWorldPosition);

	if (trajectoryFeature == nullptr) {
		return;
	}
	const size_t numFutureSamples = trajectoryFeature->GetNumFutureSamples();
	if (numFutureSamples == 0) {
		return;
	}
	if (numFutureSamples < 2 || trajectoryFeature->GetFutureTimeRange() <= std::numeric_limits<float>::epsilon()) {
		for (size_t i = 0; i < numFutureSamples; ++i) {
			m_futureControlPoints[i].m_position = actorInstanceWorldPosition;
			m_futureControlPoints[i].m_facingDirection = actorInstanceWorldRotation.xform(trajectoryFeature->GetFacingAxisDir());
		}
		return;
	}
	const float numSections = static_cast<float>(numFutureSamples - 1);

	float linearDisplacementPerSample = 0.0f;
	Quaternion targetFacingDirQuat = actorInstanceWorldRotation;

	const Vector3 &d = actorInstanceToTarget;
	const bool inside = Math::abs(d.x) <= m_deadZone && Math::abs(d.y) <= m_deadZone && Math::abs(d.z) <= m_deadZone;
	if (!inside) {

		assert(trajectoryFeature->GetFutureTimeRange() > std::numeric_limits<float>::epsilon() && "Trajectory feature future time range is too small.");
		const float velocity = actorInstanceToTarget.length() / trajectoryFeature->GetFutureTimeRange();

		linearDisplacementPerSample = (velocity / numSections);
	} else {

		useTargetFacingDir = true;
	}

	if (useTargetFacingDir) {

		targetFacingDirQuat = ShortestArc(trajectoryFeature->GetFacingAxisDir(), targetFacingDir);
	} else {

		targetFacingDirQuat = ShortestArc(trajectoryFeature->GetFacingAxisDir(), actorInstanceToTarget);
	}

	m_futureControlPoints[0].m_position = actorInstanceWorldPosition;
	m_futureControlPoints[0].m_facingDirection = actorInstanceWorldRotation.xform(trajectoryFeature->GetFacingAxisDir());

	if (useTargetFacingDir) {
		for (size_t i = 0; i < numFutureSamples; ++i) {
			const float sampleTime = (numFutureSamples > 1) ? static_cast<float>(i) / static_cast<float>(numFutureSamples - 1) : 0.0f;
			m_futureControlPoints[i].m_position = actorInstanceWorldPosition.lerp(targetPos, sampleTime);
			m_futureControlPoints[i].m_facingDirection = targetFacingDir;
		}
		return;
	}

	for (size_t i = 1; i < numFutureSamples; ++i) {
		const float t = static_cast<float>(i) / numSections;

		{
			const Vector3 prevFacingDir = m_futureControlPoints[i - 1].m_facingDirection;

			const float weight = 1.0f - std::pow(1.0f - t, m_positionBias);

			const Vector3 interpolatedPosDelta = prevFacingDir.lerp(NormalizedSafe(actorInstanceToTarget), weight);

			const Vector3 scaledPosDelta = interpolatedPosDelta * linearDisplacementPerSample;

			m_futureControlPoints[i].m_position = m_futureControlPoints[i - 1].m_position + scaledPosDelta;
		}

		{

			const float weight = 1.0f - std::pow(1.0f - t, m_rotationBias);
			const Quaternion interpolatedRotation = actorInstanceWorldRotation.slerp(targetFacingDirQuat, weight);

			const Vector3 interpolatedFacingDir = interpolatedRotation.xform(trajectoryFeature->GetFacingAxisDir());

			m_futureControlPoints[i].m_facingDirection = NormalizedSafe(interpolatedFacingDir);
		}
	}
}

void TrajectoryQuery::Update(const ActorInstance &actorInstance,
	const FeatureTrajectory *trajectoryFeature,
	const TrajectoryHistory &trajectoryHistory,
	EMode mode,
	const Vector3 &targetPos,
	const Vector3 &targetFacingDir,
	bool useTargetFacingDir,
	float timeDelta,
	float pathRadius,
	float pathSpeed) {
	if (trajectoryFeature == nullptr) {
		return;
	}

	const size_t numPastSamples = trajectoryFeature->GetNumPastSamples();
	m_pastControlPoints.resize(numPastSamples);
	const float pastTimeRange = trajectoryFeature->GetPastTimeRange();

	for (size_t i = 0; i < numPastSamples; ++i) {
		const float sampleTimeNormalized = (numPastSamples > 1) ? i / static_cast<float>(numPastSamples - 1) : 0.0f;
		const Sample sample = trajectoryHistory.Evaluate(sampleTimeNormalized * pastTimeRange);
		m_pastControlPoints[i] = { sample.m_position, sample.m_facingDirection };
	}

	const size_t numFutureSamples = trajectoryFeature->GetNumFutureSamples();
	m_futureControlPoints.resize(numFutureSamples);

	if (mode == MODE_TARGETDRIVEN) {
		PredictFutureTrajectory(actorInstance, trajectoryFeature, targetPos, targetFacingDir, useTargetFacingDir);
	} else {
		m_automaticModePhase += timeDelta * pathSpeed;
		Vector3 base = SampleFunction(0.0f, pathRadius, m_automaticModePhase);
		for (size_t i = 0; i < numFutureSamples; ++i) {
			const float offset = i * 0.1f;
			const Vector3 curSample = SampleFunction(offset, pathRadius, m_automaticModePhase);
			Vector3 displacement = curSample - base;
			m_futureControlPoints[i].m_position = actorInstance.GetWorldSpaceTransform().m_position + displacement;

			const Vector3 deltaSample = SampleFunction(offset + 0.01f, pathRadius, m_automaticModePhase);
			const Vector3 dir = deltaSample - curSample;

			m_futureControlPoints[i].m_facingDirection = NormalizedSafe(dir);
		}
	}
}

void TrajectoryQuery::DebugDraw(DebugDrawOutput &debugDisplay, const Color &color) const {
	DebugDrawControlPoints(debugDisplay, m_pastControlPoints, color);
	DebugDrawControlPoints(debugDisplay, m_futureControlPoints, color);
}

void TrajectoryQuery::DebugDrawControlPoints(DebugDrawOutput &debugDisplay,
	const std::vector<ControlPoint> &controlPoints,
	const Color &color) {
	const float markerSize = 0.02f;

	const size_t numControlPoints = controlPoints.size();
	if (numControlPoints > 1) {
		for (size_t i = 0; i < numControlPoints - 1; ++i) {
			const ControlPoint &current = controlPoints[i];
			const Vector3 &posA = current.m_position;
			const Vector3 &posB = controlPoints[i + 1].m_position;
			const Vector3 diff = posB - posA;

			(void)NormalizedSafe(diff);
			debugDisplay.cylinder_points.push_back(posA);
			debugDisplay.cylinder_points.push_back(posB);
			debugDisplay.cylinder_radii.push_back(0.0025f);
			debugDisplay.cylinder_colors.push_back(color);

			FeatureTrajectory::DebugDrawFacingDirection(debugDisplay, current.m_position, current.m_facingDirection, color);
		}

		for (const ControlPoint &controlPoint : controlPoints) {
			debugDisplay.ball_centers.push_back(controlPoint.m_position);
			debugDisplay.ball_sizes.push_back(markerSize);
			debugDisplay.ball_colors.push_back(color);
			FeatureTrajectory::DebugDrawFacingDirection(debugDisplay, controlPoint.m_position, controlPoint.m_facingDirection, color);
		}
	}
}

}
