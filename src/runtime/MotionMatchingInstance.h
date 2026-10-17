#pragma once

#include <cstddef>
#include <vector>

#include <godot_cpp/classes/ref.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include "../debug/MotionMatchingCompat.h"
#include "../features/Feature.h"
#include "../math/QueryVector.h"
#include "../trajectory/TrajectoryHistory.h"
#include "../trajectory/TrajectoryQuery.h"
#include "Pose.h"
#include "Transform.h"

class ActorInstance;
class Motion;
class MotionInstance;

namespace godot::MotionMatching {
class FeatureTrajectory;
class MotionMatchingData;

class MotionMatchingInstance {
public:
	MotionMatchingInstance() = default;
	virtual ~MotionMatchingInstance();

	struct InitSettings {
		ActorInstance *m_actorInstance = nullptr;
		MotionMatchingData *m_data = nullptr;
	};
	void Init(const InitSettings &settings);

	void DebugDraw(DebugDrawOutput &debugDisplay);

	void Update(float timePassedInSeconds,
		const Vector3 &targetPos,
		const Vector3 &targetFacingDir,
		bool useTargetFacingDir,
		TrajectoryQuery::EMode mode,
		float pathRadius,
		float pathSpeed);
	void PostUpdate(float timeDelta);
	void Output(Pose &outputPose);

	MotionInstance *GetMotionInstance() const { return m_motionInstance; }
	ActorInstance *GetActorInstance() const { return m_actorInstance; }
	MotionMatchingData *GetData() const { return m_data; }

	size_t GetLowestCostFrameIndex() const { return m_lowestCostFrameIndex; }
	void SetLowestCostSearchFrequency(float frequency) { m_lowestCostSearchFrequency = frequency; }
	float GetNewMotionTime() const { return m_newMotionTime; }

	FeatureTrajectory *GetTrajectoryFeature() const { return m_cachedTrajectoryFeature.ptr(); }
	const TrajectoryQuery &GetTrajectoryQuery() const { return m_trajectoryQuery; }
	const TrajectoryHistory &GetTrajectoryHistory() const { return m_trajectoryHistory; }
	const Transform &GetMotionExtractionDelta() const { return m_motionExtractionDelta; }

	MotionInstance *CreateMotionInstance() const;
	void DebugDrawQueryPose(DebugDrawOutput &debugDisplay, bool drawPose, bool drawVelocities) const;
	void SamplePose(MotionInstance *motionInstance, Pose &outputPose);
	void SamplePose(Motion *motion, Pose &outputPose, float sampleTime) const;

	size_t FindLowestCostFrameIndex(const Feature::QueryVectorContext &queryVectorContext, const Feature::FrameCostContext &frameCostContext);

private:

	MotionMatchingData *m_data = nullptr;
	ActorInstance *m_actorInstance = nullptr;
	Pose m_blendSourcePose;
	Pose m_blendTargetPose;
	Pose m_queryPose;
	MotionInstance *m_motionInstance = nullptr;
	MotionInstance *m_prevMotionInstance = nullptr;
	Transform m_motionExtractionDelta;

	Ref<QueryVector> m_queryVector;

	Ref<QueryVector> m_kdTreeQueryVector;
	std::vector<size_t> m_nearestFrames;

	Ref<FeatureTrajectory> m_cachedTrajectoryFeature;
	TrajectoryQuery m_trajectoryQuery;
	TrajectoryHistory m_trajectoryHistory;
	static constexpr float m_trajectorySecsToTrack = 5.0f;

	float m_timeSinceLastFrameSwitch = 0.0f;
	float m_newMotionTime = 0.0f;
	size_t m_lowestCostFrameIndex = InvalidIndex;
	float m_lowestCostSearchFrequency = 5.0f;

	bool m_blending = false;
	float m_blendWeight = 1.0f;
	float m_blendProgressTime = 0.0f;

	std::vector<float> m_tempCosts;
	std::vector<float> m_minCosts;
};
}
