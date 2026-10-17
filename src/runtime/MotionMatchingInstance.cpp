#include "MotionMatchingInstance.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstring>
#include <limits>

#include <godot_cpp/classes/object.hpp>
#include <godot_cpp/classes/project_settings.hpp>
#include <godot_cpp/classes/time.hpp>
#include <godot_cpp/variant/color.hpp>

#include "Actor.h"
#include "ActorInstance.h"
#include "AnimGraphPosePool.h"
#include "EMotionFXManager.h"
#include "Motion.h"
#include "MotionData.h"
#include "MotionInstance.h"
#include "MotionInstancePool.h"
#include "MmNode.h"
#include "Pose.h"
#include "PoseDataJointVelocities.h"
#include "Skeleton.h"
#include "Transform.h"
#include "TransformData.h"
#include "../database/Frame.h"
#include "../database/FrameDatabase.h"
#include "../database/KdTree.h"
#include "../database/MotionMatchingData.h"
#include "../debug/MotionMatchingMonitorBus.h"
#include "../features/Feature.h"
#include "../features/FeatureSchema.h"
#include "../features/FeatureTrajectory.h"
#include "../features/FeatureVelocity.h"
#include "../math/FeatureMatrix.h"
#include "../math/FeatureMatrixTransformer.h"

namespace godot::MotionMatching {

static bool GetMmDebugDraw() {
	ProjectSettings *ps = ProjectSettings::get_singleton();
	if (!ps) {
		return false;
	}
	const bool enabled = ps->get_setting("motion_matching/debug_draw", false);
	return enabled;
}

static bool GetMmDebugDrawQueryPose() {
	ProjectSettings *ps = ProjectSettings::get_singleton();
	if (!ps) {
		return false;
	}
	const bool enabled = ps->get_setting("motion_matching/debug_draw_query_pose", false);
	return enabled;
}

static bool GetMmDebugDrawQueryVelocities() {
	ProjectSettings *ps = ProjectSettings::get_singleton();
	if (!ps) {
		return false;
	}
	const bool enabled = ps->get_setting("motion_matching/debug_draw_query_velocities", false);
	return enabled;
}

static bool GetMmUseKdTree() {
	ProjectSettings *ps = ProjectSettings::get_singleton();
	if (!ps) {
		return true;
	}
	const bool enabled = ps->get_setting("motion_matching/use_kd_tree", true);
	return enabled;
}

MotionMatchingInstance::~MotionMatchingInstance() {

	if (m_motionInstance) {
		GetMotionInstancePool().Free(m_motionInstance);
	}

	if (m_prevMotionInstance) {
		GetMotionInstancePool().Free(m_prevMotionInstance);
	}
}

MotionInstance *MotionMatchingInstance::CreateMotionInstance() const {
	if (!m_data || m_data->GetFrameDatabase().GetNumFrames() == 0 || !m_actorInstance) {
		return nullptr;
	}
	Motion *sourceMotion = m_data->GetFrameDatabase().GetFrame(0).GetSourceMotion();
	if (!sourceMotion) {
		return nullptr;
	}
	MotionInstance *result = GetMotionInstancePool().RequestNew(sourceMotion, m_actorInstance);
	return result;
}

void MotionMatchingInstance::Init(const InitSettings &settings) {
	assert(settings.m_actorInstance && "The actor instance cannot be a nullptr.");
	assert(settings.m_data && "The motion match data cannot be nullptr.");
	if (!settings.m_actorInstance || !settings.m_data) {
		return;
	}

	const FeatureSchema &featureSchema = settings.m_data->GetFeatureSchema();
	for (const Ref<Feature> &feature : featureSchema.GetFeatures()) {
		if (feature.is_valid() && Object::cast_to<FeatureTrajectory>(feature.ptr()) != nullptr) {
			m_cachedTrajectoryFeature = Ref<FeatureTrajectory>(static_cast<FeatureTrajectory *>(feature.ptr()));
			break;
		}
	}

	m_actorInstance = settings.m_actorInstance;
	m_data = settings.m_data;
	if (settings.m_data->GetFrameDatabase().GetNumFrames() == 0) {
		return;
	}

	if (!m_motionInstance) {
		m_motionInstance = CreateMotionInstance();
	}

	if (!m_prevMotionInstance) {
		m_prevMotionInstance = CreateMotionInstance();
	}

	m_blendSourcePose.LinkToActorInstance(m_actorInstance);
	m_blendSourcePose.InitFromBindPose(m_actorInstance);

	m_blendTargetPose.LinkToActorInstance(m_actorInstance);
	m_blendTargetPose.InitFromBindPose(m_actorInstance);

	m_queryPose.LinkToActorInstance(m_actorInstance);
	m_queryPose.InitFromBindPose(m_actorInstance);

	const size_t numValuesInKdTree = m_data->GetKdTree().GetNumDimensions();
	if (m_kdTreeQueryVector.is_null()) {
		m_kdTreeQueryVector.instantiate();
	}
	if (m_queryVector.is_null()) {
		m_queryVector.instantiate();
	}
	m_kdTreeQueryVector->Resize(numValuesInKdTree);
	m_queryVector->Resize(m_data->GetFeatureMatrix().cols());

	if (m_cachedTrajectoryFeature.is_valid() && m_actorInstance && m_actorInstance->GetActor() && m_actorInstance->GetTransformData() && m_actorInstance->GetTransformData()->GetCurrentPose()) {
		size_t rootJointIndex = m_actorInstance->GetActor()->GetMotionExtractionNodeIndex();
		if (rootJointIndex == InvalidIndex) {
			rootJointIndex = 0;
		}
		m_trajectoryHistory.Init(*m_actorInstance->GetTransformData()->GetCurrentPose(),
			rootJointIndex,
			m_cachedTrajectoryFeature->GetFacingAxisDir(),
			m_trajectorySecsToTrack);
	}
}

void MotionMatchingInstance::DebugDraw(DebugDrawOutput &debugDisplay) {
	if (!GetMmDebugDraw() || !m_data) {
		return;
	}

	const size_t curFrameIndex = GetLowestCostFrameIndex();
	if (curFrameIndex == InvalidIndex) {
		return;
	}

	const FrameDatabase &frameDatabase = m_data->GetFrameDatabase();
	const FeatureSchema &featureSchema = m_data->GetFeatureSchema();
	const FeatureMatrix &featureMatrix = m_data->GetFeatureMatrix();

	if (!m_motionInstance || !m_motionInstance->GetMotion() || !m_actorInstance || !m_actorInstance->GetTransformData() || !m_actorInstance->GetTransformData()->GetCurrentPose()) {
		return;
	}
	const size_t currentFrame = frameDatabase.FindFrameIndex(m_motionInstance->GetMotion(), m_motionInstance->GetCurrentTime());

	if (currentFrame != InvalidIndex) {
		const Pose &currentPose = *m_actorInstance->GetTransformData()->GetCurrentPose();
		for (const Ref<Feature> &feature : featureSchema.GetFeatures()) {
			if (feature.is_valid() && feature->GetDebugDrawEnabled()) {
				feature->DebugDraw(debugDisplay, currentPose, featureMatrix, m_data->GetFeatureTransformer(), currentFrame);
			}
		}
	}

	const Color trajectoryQueryColor = Color(90.0f / 255.0f, 219.0f / 255.0f, 64.0f / 255.0f, 1.0f);
	m_trajectoryQuery.DebugDraw(debugDisplay, trajectoryQueryColor);

	m_trajectoryHistory.DebugDraw(debugDisplay, trajectoryQueryColor, m_cachedTrajectoryFeature.is_valid() ? m_cachedTrajectoryFeature->GetPastTimeRange() : 0.0f);

	DebugDrawQueryPose(debugDisplay, GetMmDebugDrawQueryPose(), GetMmDebugDrawQueryVelocities());
}

void MotionMatchingInstance::DebugDrawQueryPose(DebugDrawOutput &debugDisplay, bool drawPose, bool drawVelocities) const {
	if (!m_actorInstance || !m_data) {
		return;
	}
	const Color color = Color(1.0f, 1.0f, 1.0f);

	if (drawPose) {
		m_queryPose.DebugDraw(debugDisplay, color);
	}

	if (drawVelocities) {
		const PoseDataJointVelocities *velocityPoseData = m_queryPose.GetPoseData<PoseDataJointVelocities>();
		if (velocityPoseData && velocityPoseData->GetVelocities().size() > 0 && m_actorInstance->GetActor()) {
			const Skeleton *skeleton = m_actorInstance->GetActor()->GetSkeleton();
			if (!skeleton) {
				return;
			}
			for (const Ref<Feature> &feature : m_data->GetFeatureSchema().GetFeatures()) {
				if (feature.is_null()) {
					continue;
				}
				if (const FeatureVelocity *velocityFeature = Object::cast_to<FeatureVelocity>(feature.ptr())) {
					MmNode *joint = skeleton->FindNodeByName(velocityFeature->GetJointName());
					if (joint) {
						const size_t jointIndex = joint->GetNodeIndex();
						if (jointIndex >= velocityPoseData->GetVelocities().size()) {
							continue;
						}
						const size_t relativeToJointIndex = feature->GetRelativeToNodeIndex();
						const Vector3 &velocity = velocityPoseData->GetVelocities()[jointIndex];
						velocityFeature->DebugDraw(debugDisplay, m_queryPose, velocity, jointIndex, relativeToJointIndex, color);
					}
				}
			}
		}
	}
}

void MotionMatchingInstance::SamplePose(MotionInstance *motionInstance, Pose &outputPose) {
	if (!motionInstance || !motionInstance->GetMotion() || !m_actorInstance || !m_actorInstance->GetActor() || !m_actorInstance->GetTransformData() || !m_actorInstance->GetTransformData()->GetBindPose()) {
		return;
	}
	const Pose *bindPose = m_actorInstance->GetTransformData()->GetBindPose();
	motionInstance->GetMotion()->Update(bindPose, &outputPose, motionInstance);
	if (m_actorInstance->GetActor()->GetMotionExtractionNode() && m_actorInstance->GetActor()->GetMotionExtractionEnabled()) {
		outputPose.CompensateForMotionExtraction();
	}
}

void MotionMatchingInstance::SamplePose(Motion *motion, Pose &outputPose, float sampleTime) const {
	if (!motion) {
		return;
	}
	ActorInstance *poseActor = outputPose.GetActorInstance();
	if (!poseActor || !poseActor->GetTransformData() || !poseActor->GetTransformData()->GetBindPose()) {
		return;
	}
	MotionDataSampleSettings sampleSettings;
	sampleSettings.m_actorInstance = poseActor;
	sampleSettings.m_inPlace = false;
	sampleSettings.m_mirror = false;
	sampleSettings.m_retarget = false;
	sampleSettings.m_inputPose = sampleSettings.m_actorInstance->GetTransformData()->GetBindPose();

	sampleSettings.m_sampleTime = sampleTime;
	sampleSettings.m_sampleTime = std::clamp(sampleTime, 0.0f, motion->GetDuration());

	motion->SamplePose(&outputPose, sampleSettings);
}

void MotionMatchingInstance::PostUpdate([[maybe_unused]] float timeDelta) {
	if (!m_data) {
		m_motionExtractionDelta.Identity();
		return;
	}

	const size_t lowestCostFrame = GetLowestCostFrameIndex();
	if (m_data->GetFrameDatabase().GetNumFrames() == 0 || lowestCostFrame == InvalidIndex) {
		m_motionExtractionDelta.Identity();
		return;
	}

	if (!m_motionInstance || !m_prevMotionInstance) {
		m_motionExtractionDelta.Identity();
		return;
	}
	if (m_blendWeight >= 1.0f - std::numeric_limits<float>::epsilon()) {
		m_motionInstance->ExtractMotion(m_motionExtractionDelta);
	} else if (m_blendWeight > std::numeric_limits<float>::epsilon() && m_blendWeight < 1.0f - std::numeric_limits<float>::epsilon()) {
		Transform targetMotionExtractionDelta;
		m_motionInstance->ExtractMotion(m_motionExtractionDelta);
		m_prevMotionInstance->ExtractMotion(targetMotionExtractionDelta);
		m_motionExtractionDelta.Blend(targetMotionExtractionDelta, m_blendWeight);
	} else {
		m_prevMotionInstance->ExtractMotion(m_motionExtractionDelta);
	}
}

void MotionMatchingInstance::Output(Pose &outputPose) {
	if (!m_data) {
		if (m_actorInstance) {
			outputPose.InitFromBindPose(m_actorInstance);
		}
		return;
	}
	if (!m_actorInstance) {
		return;
	}

	const size_t lowestCostFrame = GetLowestCostFrameIndex();
	if (m_data->GetFrameDatabase().GetNumFrames() == 0 || lowestCostFrame == InvalidIndex) {
		outputPose.InitFromBindPose(m_actorInstance);
		return;
	}

	if (m_blendWeight >= 1.0f - std::numeric_limits<float>::epsilon()) {
		m_blendTargetPose.InitFromBindPose(m_actorInstance);
		if (m_motionInstance) {
			SamplePose(m_motionInstance, m_blendTargetPose);
		}
		outputPose = m_blendTargetPose;
	} else if (m_blendWeight > std::numeric_limits<float>::epsilon() && m_blendWeight < 1.0f - std::numeric_limits<float>::epsilon()) {
		m_blendSourcePose.InitFromBindPose(m_actorInstance);
		m_blendTargetPose.InitFromBindPose(m_actorInstance);
		if (m_motionInstance) {
			SamplePose(m_motionInstance, m_blendTargetPose);
		}
		if (m_prevMotionInstance) {
			SamplePose(m_prevMotionInstance, m_blendSourcePose);
		}

		outputPose = m_blendSourcePose;
		outputPose.Blend(&m_blendTargetPose, m_blendWeight);
	} else {
		m_blendSourcePose.InitFromBindPose(m_actorInstance);
		if (m_prevMotionInstance) {
			SamplePose(m_prevMotionInstance, m_blendSourcePose);
		}
		outputPose = m_blendSourcePose;
	}
}

void MotionMatchingInstance::Update(float timePassedInSeconds,
	const Vector3 &targetPos,
	const Vector3 &targetFacingDir,
	bool useTargetFacingDir,
	TrajectoryQuery::EMode mode,
	float pathRadius,
	float pathSpeed) {
	if (!m_data || !m_motionInstance || !m_actorInstance || !m_actorInstance->GetTransformData() || !m_actorInstance->GetTransformData()->GetCurrentPose()) {
		return;
	}

	size_t currentFrameIndex = GetLowestCostFrameIndex();
	if (currentFrameIndex == InvalidIndex) {
		currentFrameIndex = 0;
	}
	if (currentFrameIndex >= m_data->GetFrameDatabase().GetNumFrames()) {
		return;
	}

	m_trajectoryHistory.AddSample(*m_actorInstance->GetTransformData()->GetCurrentPose());

	m_trajectoryHistory.Update(timePassedInSeconds);

	if (m_cachedTrajectoryFeature.is_valid()) {
		m_trajectoryQuery.Update(*m_actorInstance,
			m_cachedTrajectoryFeature.ptr(),
			m_trajectoryHistory,
			mode,
			targetPos,
			targetFacingDir,
			useTargetFacingDir,
			timePassedInSeconds,
			pathRadius,
			pathSpeed);
	}

	m_motionInstance->SetFreezeAtLastFrame(true);
	m_motionInstance->SetMaxLoops(1);
	const float newMotionTime = m_motionInstance->CalcPlayStateAfterUpdate(timePassedInSeconds).m_currentTime;
	m_newMotionTime = newMotionTime;

	if (m_prevMotionInstance) {
		m_prevMotionInstance->Update(timePassedInSeconds);
	}

	m_timeSinceLastFrameSwitch += timePassedInSeconds;

	const float lowestCostSearchTimeInterval = 1.0f / m_lowestCostSearchFrequency;

	if (m_blending) {
		const float maxBlendTime = lowestCostSearchTimeInterval;
		m_blendProgressTime += timePassedInSeconds;
		if (m_blendProgressTime > maxBlendTime) {
			m_blendWeight = 1.0f;
			m_blendProgressTime = maxBlendTime;
			m_blending = false;
		} else {
			m_blendWeight = std::clamp(m_blendProgressTime / maxBlendTime, 0.0f, 1.0f);
		}
	}

	const bool searchLowestCostFrame = m_timeSinceLastFrameSwitch >= lowestCostSearchTimeInterval;
	if (searchLowestCostFrame) {

		{

			SamplePose(m_motionInstance->GetMotion(), m_queryPose, newMotionTime);

			if (m_actorInstance->GetActor() && m_actorInstance->GetActor()->GetMotionExtractionNode() && m_actorInstance->GetTransformData() && m_actorInstance->GetTransformData()->GetCurrentPose()) {
				const Pose *currentPose = m_actorInstance->GetTransformData()->GetCurrentPose();
				const size_t motionExtractionJointIndex = m_actorInstance->GetActor()->GetMotionExtractionNodeIndex();
				m_queryPose.SetWorldSpaceTransform(motionExtractionJointIndex,
					currentPose->GetWorldSpaceTransform(motionExtractionJointIndex));
			}

			PoseDataJointVelocities *velocityPoseData = m_queryPose.GetAndPreparePoseData<PoseDataJointVelocities>(m_actorInstance);
			if (!velocityPoseData || !m_motionInstance->GetMotion()) {
				return;
			}
			EMotionFXManager::ThreadData *threadData = GetEMotionFX().GetThreadData(m_actorInstance->GetThreadIndex());
			if (!threadData) {
				return;
			}
			AnimGraphPosePool &posePool = threadData->GetPosePool();
			const size_t relativeNodeIndex = m_cachedTrajectoryFeature.is_valid() ? m_cachedTrajectoryFeature->GetRelativeToNodeIndex() : InvalidIndex;
			velocityPoseData->CalculateVelocity(m_actorInstance, posePool, m_motionInstance->GetMotion(), newMotionTime, relativeNodeIndex);
		}

		const FeatureMatrix &featureMatrix = m_data->GetFeatureMatrix();
		const FrameDatabase &frameDatabase = m_data->GetFrameDatabase();
		if (m_queryVector.is_null() || frameDatabase.GetNumFrames() == 0) {
			m_timeSinceLastFrameSwitch = 0.0f;
			return;
		}

		Feature::QueryVectorContext queryVectorContext(m_queryPose, m_trajectoryQuery);
		queryVectorContext.m_featureTransformer = m_data->GetFeatureTransformer();

		Feature::FrameCostContext frameCostContext(*m_queryVector.ptr(), featureMatrix);
		const size_t lowestCostFrameIndex = FindLowestCostFrameIndex(queryVectorContext, frameCostContext);
		if (lowestCostFrameIndex >= frameDatabase.GetNumFrames() || currentFrameIndex >= frameDatabase.GetNumFrames()) {
			m_timeSinceLastFrameSwitch = 0.0f;
			return;
		}

		const Frame &currentFrame = frameDatabase.GetFrame(currentFrameIndex);
		const Frame &lowestCostFrame = frameDatabase.GetFrame(lowestCostFrameIndex);
		const bool sameMotion = (currentFrame.GetSourceMotion() == lowestCostFrame.GetSourceMotion());
		const float timeBetweenFrames = newMotionTime - lowestCostFrame.GetSampleTime();
		const bool sameLocation = sameMotion && (std::abs(timeBetweenFrames) < 0.1f);

		if (m_lowestCostFrameIndex == InvalidIndex) {
			m_lowestCostFrameIndex = lowestCostFrameIndex;
		}

		if (lowestCostFrameIndex != currentFrameIndex && !sameLocation) {
			if (!m_prevMotionInstance) {
				m_timeSinceLastFrameSwitch = 0.0f;
				return;
			}

			m_blending = true;
			m_blendWeight = 0.0f;
			m_blendProgressTime = 0.0f;

			m_prevMotionInstance->SetMotion(m_motionInstance->GetMotion());
			m_prevMotionInstance->SetMirrorMotion(m_motionInstance->GetMirrorMotion());
			m_prevMotionInstance->SetCurrentTime(newMotionTime);
			m_prevMotionInstance->SetLastCurrentTime(m_prevMotionInstance->GetCurrentTime() - timePassedInSeconds);

			m_lowestCostFrameIndex = lowestCostFrameIndex;

			m_motionInstance->SetMotion(lowestCostFrame.GetSourceMotion());
			m_motionInstance->SetMirrorMotion(lowestCostFrame.GetMirrored());

			m_newMotionTime = lowestCostFrame.GetSampleTime();
			m_motionInstance->SetCurrentTime(m_newMotionTime - timePassedInSeconds);
		}

		m_timeSinceLastFrameSwitch = 0.0f;
	}

	{
#ifdef GMM_MONITOR_ENABLED
		const FrameDatabase &frameDatabase = m_data->GetFrameDatabase();
		MotionMatchingMonitorRequests::FrameDatabaseInfo frameDatabaseInfo{frameDatabase.CalcMemoryUsageInBytes(), frameDatabase.GetNumFrames(), frameDatabase.GetNumUsedMotions(), frameDatabase.GetNumFrames() / (float)frameDatabase.GetSampleRate()};
		MotionMatchingMonitorRequestBus::Broadcast(&MotionMatchingMonitorRequests::SetFrameDatabaseInfo, frameDatabaseInfo);

		const KdTree &kdTree = m_data->GetKdTree();
		MotionMatchingMonitorRequests::KdTreeInfo kdTreeInfo{kdTree.CalcMemoryUsageInBytes(), kdTree.GetNumNodes(), kdTree.GetNumDimensions()};
		MotionMatchingMonitorRequestBus::Broadcast(&MotionMatchingMonitorRequests::SetKdTreeInfo, kdTreeInfo);

		const FeatureMatrix &featureMatrix = m_data->GetFeatureMatrix();
		MotionMatchingMonitorRequests::FeatureMatrixInfo featureMatrixInfo{featureMatrix.CalcMemoryUsageInBytes(), static_cast<size_t>(featureMatrix.rows()), static_cast<size_t>(featureMatrix.cols())};
		MotionMatchingMonitorRequestBus::Broadcast(&MotionMatchingMonitorRequests::SetFeatureMatrixInfo, featureMatrixInfo);
#endif
	}
}

size_t MotionMatchingInstance::FindLowestCostFrameIndex(const Feature::QueryVectorContext &queryVectorContext, const Feature::FrameCostContext &frameCostContext) {
	Time *timeSingleton = Time::get_singleton();
	const uint64_t timeBefore = timeSingleton ? timeSingleton->get_ticks_msec() : 0;

	if (!m_data || m_queryVector.is_null()) {
		return InvalidIndex;
	}
	const FrameDatabase &frameDatabase = m_data->GetFrameDatabase();
	if (frameDatabase.GetNumFrames() == 0) {
		return InvalidIndex;
	}
	const FeatureSchema &featureSchema = m_data->GetFeatureSchema();
	const FeatureTrajectory *trajectoryFeature = m_cachedTrajectoryFeature.ptr();

	{

		assert(m_queryVector->GetSize() == static_cast<size_t>(m_data->GetFeatureMatrix().cols()) &&
			"The query vector should have the same number of elements as the feature matrix has columns.");
		for (const Ref<Feature> &feature : featureSchema.GetFeatures()) {
			if (feature.is_valid()) {
				feature->FillQueryVector(*m_queryVector.ptr(), queryVectorContext);
			}
		}

		if (FeatureMatrixTransformer *transformer = queryVectorContext.m_featureTransformer) {
			transformer->Transform(m_queryVector->GetData());
		}
	}

	if (GetMmUseKdTree() && m_data->GetKdTree().IsInitialized() && m_kdTreeQueryVector.is_valid()) {
		std::vector<float> &kdTreeQueryVector = m_kdTreeQueryVector->GetData();
		const std::vector<float> &queryVectorData = m_queryVector->GetData();

		size_t startOffset = 0;
		for (const Ref<Feature> &feature : m_data->GetFeaturesInKdTree()) {
			if (feature.is_null()) {
				continue;
			}
			if (startOffset + feature->GetNumDimensions() > kdTreeQueryVector.size() ||
				feature->GetColumnOffset() + feature->GetNumDimensions() > queryVectorData.size()) {
				break;
			}
			memcpy(&kdTreeQueryVector[startOffset], &queryVectorData[feature->GetColumnOffset()], feature->GetNumDimensions() * sizeof(float));
			startOffset += feature->GetNumDimensions();
		}
		assert(startOffset == kdTreeQueryVector.size() && "Frame float vector is not the expected size.");

		m_data->GetKdTree().FindNearestNeighbors(kdTreeQueryVector, m_nearestFrames);
	}

	float minCost = std::numeric_limits<float>::max();
	size_t minCostFrameIndex = 0;
	m_tempCosts.resize(featureSchema.GetNumFeatures());
	m_minCosts.resize(featureSchema.GetNumFeatures());
	float minTrajectoryPastCost = 0.0f;
	float minTrajectoryFutureCost = 0.0f;

	const bool useKdTree = GetMmUseKdTree() && m_data->GetKdTree().IsInitialized();
	const size_t numFrames = useKdTree ? m_nearestFrames.size() : frameDatabase.GetNumFrames();
	for (size_t i = 0; i < numFrames; ++i) {
		const size_t frameIndex = useKdTree ? m_nearestFrames[i] : i;
		if (frameIndex >= frameDatabase.GetNumFrames()) {
			continue;
		}
		const Frame &frame = frameDatabase.GetFrame(frameIndex);
		Motion *frameMotion = frame.GetSourceMotion();
		if (!frameMotion) {
			continue;
		}

		if (frame.GetSampleTime() >= frameMotion->GetDuration() - 1.0f) {
			continue;
		}

		float frameCost = 0.0f;

		for (size_t featureIndex = 0; featureIndex < featureSchema.GetNumFeatures(); ++featureIndex) {
			Ref<Feature> feature = featureSchema.GetFeature(featureIndex);
			if (feature.is_valid() && Object::cast_to<FeatureTrajectory>(feature.ptr()) == nullptr) {
				const float featureCost = feature->CalculateFrameCost(frameIndex, frameCostContext);
				const float featureCostFactor = feature->GetCostFactor();
				const float featureFinalCost = featureCost * featureCostFactor;

				frameCost += featureFinalCost;
				m_tempCosts[featureIndex] = featureFinalCost;
			}
		}

		float trajectoryPastCost = 0.0f;
		float trajectoryFutureCost = 0.0f;
		if (trajectoryFeature) {
			trajectoryPastCost = trajectoryFeature->CalculatePastFrameCost(frameIndex, frameCostContext) * trajectoryFeature->GetPastCostFactor();
			trajectoryFutureCost = trajectoryFeature->CalculateFutureFrameCost(frameIndex, frameCostContext) * trajectoryFeature->GetFutureCostFactor();
			frameCost += trajectoryPastCost;
			frameCost += trajectoryFutureCost;
		}

		if (frameCost < minCost) {
			minCost = frameCost;
			minCostFrameIndex = frameIndex;

			for (size_t featureIndex = 0; featureIndex < featureSchema.GetNumFeatures(); ++featureIndex) {
				Ref<Feature> feature = featureSchema.GetFeature(featureIndex);
				if (feature.is_valid() && Object::cast_to<FeatureTrajectory>(feature.ptr()) == nullptr) {
					m_minCosts[featureIndex] = m_tempCosts[featureIndex];
				}
			}

			minTrajectoryPastCost = trajectoryPastCost;
			minTrajectoryFutureCost = trajectoryFutureCost;
		}
	}

	{
		Time *tSingleton = Time::get_singleton();
		const float time = (tSingleton && tSingleton->get_ticks_msec() >= timeBefore) ? (float)(tSingleton->get_ticks_msec() - timeBefore) / 1000.0f : 0.0f;
		MotionMatchingMonitorRequestBus::Broadcast(&MotionMatchingMonitorRequests::PushPerformanceHistogramValue, "FindLowestCostFrameIndex", time * 1000.0f);

		for (size_t featureIndex = 0; featureIndex < featureSchema.GetNumFeatures(); ++featureIndex) {
			Ref<Feature> feature = featureSchema.GetFeature(featureIndex);
			if (feature.is_valid() && Object::cast_to<FeatureTrajectory>(feature.ptr()) == nullptr) {
				MotionMatchingMonitorRequestBus::Broadcast(&MotionMatchingMonitorRequests::PushCostHistogramValue,
					feature->GetName().c_str(),
					m_minCosts[featureIndex],
					feature->GetDebugDrawColor());
			}
		}

		if (trajectoryFeature) {
			MotionMatchingMonitorRequestBus::Broadcast(&MotionMatchingMonitorRequests::PushCostHistogramValue, "Future Trajectory", minTrajectoryFutureCost, trajectoryFeature->GetDebugDrawColor());
			MotionMatchingMonitorRequestBus::Broadcast(&MotionMatchingMonitorRequests::PushCostHistogramValue, "Past Trajectory", minTrajectoryPastCost, trajectoryFeature->GetDebugDrawColor());
		}

		MotionMatchingMonitorRequestBus::Broadcast(&MotionMatchingMonitorRequests::PushCostHistogramValue, "Total Cost", minCost, Color(202.0f / 255.0f, 255.0f / 255.0f, 191.0f / 255.0f, 1.0f));
	}

	return minCostFrameIndex;
}

}
