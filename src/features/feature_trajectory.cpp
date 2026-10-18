#include "features/feature_trajectory.h"

#include <cassert>
#include <functional>

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/string.hpp>

#include "actor/actor_instance.h"
#include "actor/anim_graph_pose.h"
#include "actor/anim_graph_pose_pool.h"
#include "pose/pose.h"
#include "core/types/transform.h"
#include "pose/transform_data.h"
#include "database/frame.h"
#include "database/frame_database.h"
#include "core/math/feature_matrix_transformer.h"
#include "runtime/motion_matching_instance.h"
#include "trajectory/trajectory_query.h"

namespace godot::MotionMatching {

bool FeatureTrajectory::Init(const InitSettings &settings) {
	const bool result = Feature::Init(settings);
	UpdateFacingAxis();
	return result;
}

Vector2 FeatureTrajectory::CalculateFacingDirection(const Pose &pose, const Transform &invRootTransform) const {

	const Vector3 facingDirAnimationWorldSpace = pose.GetWorldSpaceTransform(m_jointIndex).TransformVector(m_facingAxisDir);

	const Vector3 facingDirection = invRootTransform.TransformVector(facingDirAnimationWorldSpace);

	return NormalizedSafe(Vector2(facingDirection.x, facingDirection.z));
}

FeatureTrajectory::Sample FeatureTrajectory::GetSampleFromPose(const Pose &pose, const Transform &invRootTransform) const {

	const Vector3 relativePos = invRootTransform.TransformPoint(pose.GetWorldSpaceTransform(m_jointIndex).m_position);
	const Vector2 position = Vector2(relativePos.x, relativePos.z);

	const Vector2 facingDirection = CalculateFacingDirection(pose, invRootTransform);

	return { position, facingDirection };
}

void FeatureTrajectory::ExtractFeatureValues(const ExtractFeatureContext &context) {
	const ActorInstance *actorInstance = context.m_actorInstance;
	if (context.m_frameDatabase == nullptr || context.m_frameIndex >= context.m_frameDatabase->GetNumFrames() || context.m_framePose == nullptr) {
		return;
	}
	AnimGraphPose *samplePose = context.m_posePool.RequestPose(actorInstance);
	AnimGraphPose *nextSamplePose = context.m_posePool.RequestPose(actorInstance);
	if (samplePose == nullptr || nextSamplePose == nullptr) {
		if (samplePose) {
			context.m_posePool.FreePose(samplePose);
		}
		if (nextSamplePose) {
			context.m_posePool.FreePose(nextSamplePose);
		}
		return;
	}

	const size_t frameIndex = context.m_frameIndex;
	const Frame &currentFrame = context.m_frameDatabase->GetFrame(context.m_frameIndex);

	const Transform invRootTransform = context.m_framePose->GetWorldSpaceTransform(m_relativeToNodeIndex).Inversed();

	const size_t midSampleIndex = CalcMidFrameIndex();
	const Sample midSample = GetSampleFromPose(*context.m_framePose, invRootTransform);
	SetFeatureData(context.m_featureMatrix, frameIndex, midSampleIndex, midSample);

	if (m_numPastSamples < 2 || m_numFutureSamples < 2) {
		context.m_posePool.FreePose(samplePose);
		context.m_posePool.FreePose(nextSamplePose);
		return;
	}
	const float pastFrameTimeDelta = m_pastTimeRange / static_cast<float>(m_numPastSamples - 1);
	currentFrame.SamplePose(&samplePose->GetPose());
	for (size_t i = 0; i < m_numPastSamples; ++i) {

		const float sampleTimeOffset = (i + 1) * pastFrameTimeDelta * (-1.0f);
		currentFrame.SamplePose(&nextSamplePose->GetPose(), sampleTimeOffset);

		const Sample sample = GetSampleFromPose(samplePose->GetPose(), invRootTransform);
		const size_t sampleIndex = CalcPastFrameIndex(i);
		SetFeatureData(context.m_featureMatrix, frameIndex, sampleIndex, sample);

		*samplePose = *nextSamplePose;
	}

	const float futureFrameTimeDelta = m_futureTimeRange / (float)(m_numFutureSamples - 1);
	currentFrame.SamplePose(&samplePose->GetPose());
	for (size_t i = 0; i < m_numFutureSamples; ++i) {

		const float sampleTimeOffset = (i + 1) * futureFrameTimeDelta;
		currentFrame.SamplePose(&nextSamplePose->GetPose(), sampleTimeOffset);

		const Sample sample = GetSampleFromPose(samplePose->GetPose(), invRootTransform);
		const size_t sampleIndex = CalcFutureFrameIndex(i);
		SetFeatureData(context.m_featureMatrix, frameIndex, sampleIndex, sample);

		*samplePose = *nextSamplePose;
	}

	context.m_posePool.FreePose(samplePose);
	context.m_posePool.FreePose(nextSamplePose);
}

void FeatureTrajectory::FillQueryVector(QueryVector &queryVector, const QueryVectorContext &context) {
	const Transform invRootTransform = context.m_currentPose.GetWorldSpaceTransform(m_relativeToNodeIndex).Inversed();

	auto FillControlPoints =
		[this, &queryVector, &invRootTransform](
			const std::vector<TrajectoryQuery::ControlPoint> &controlPoints, const std::function<size_t(size_t)> &CalcFrameIndex) {
			const size_t numControlPoints = controlPoints.size();
			for (size_t i = 0; i < numControlPoints; ++i) {
				TrajectoryQuery::ControlPoint controlPoint = controlPoints[i];
				controlPoint.m_position =
					invRootTransform.TransformPoint(controlPoint.m_position);
				controlPoint.m_facingDirection = invRootTransform.TransformVector(controlPoint.m_facingDirection);

				const size_t sampleIndex = CalcFrameIndex(i);
				const size_t sampleColumnStart = m_featureColumnOffset + sampleIndex * Sample::s_componentsPerSample;

				queryVector.SetVector2(Vector2(controlPoint.m_position.x, controlPoint.m_position.z), sampleColumnStart + 0);
				queryVector.SetVector2(Vector2(controlPoint.m_facingDirection.x, controlPoint.m_facingDirection.z), sampleColumnStart + 2);
			}
		};

	assert(
		context.m_trajectoryQuery.GetFutureControlPoints().size() == m_numFutureSamples &&
		"Number of future control points from the trajectory query does not match the one from the trajectory feature.");
	assert(
		context.m_trajectoryQuery.GetPastControlPoints().size() == m_numPastSamples &&
		"Number of past control points from the trajectory query does not match the one from the trajectory feature");

	FillControlPoints(
		context.m_trajectoryQuery.GetPastControlPoints(),
		[this](size_t frameIndex) {
			return CalcPastFrameIndex(frameIndex);
		});
	FillControlPoints(
		context.m_trajectoryQuery.GetFutureControlPoints(),
		[this](size_t frameIndex) {
			return CalcFutureFrameIndex(frameIndex);
		});
}

float FeatureTrajectory::CalculateCost(
	const FeatureMatrix &featureMatrix,
	size_t frameIndex,
	size_t numControlPoints,
	const SplineToFeatureMatrixIndex &splineToFeatureMatrixIndex,
	const FrameCostContext &context) const {
	float cost = 0.0f;
	Vector2 lastControlPoint, lastSamplePos;

	for (size_t i = 0; i < numControlPoints; ++i) {
		const Sample sample = GetFeatureData(featureMatrix, frameIndex, splineToFeatureMatrixIndex(i));
		const Vector2 &samplePos = sample.m_position;

		const size_t sampleColumnStart = m_featureColumnOffset + splineToFeatureMatrixIndex(i) * Sample::s_componentsPerSample;
		const Vector2 controlPointPos = context.m_queryVector.GetVector2(sampleColumnStart + 0);
		const Vector2 controlPointFacingDirRelativeSpace = context.m_queryVector.GetVector2(sampleColumnStart + 2);

		if (i != 0) {
			const Vector2 controlPointDelta = controlPointPos - lastControlPoint;
			const Vector2 sampleDelta = samplePos - lastSamplePos;

			const float posDistance = (samplePos - controlPointPos).length();
			const float posDeltaDistance = (controlPointDelta - sampleDelta).length();

			const float facingDirectionCost =
				GetNormalizedDirectionDifference(sample.m_facingDirection, controlPointFacingDirRelativeSpace);

			cost += CalcResidual(posDistance) + CalcResidual(posDeltaDistance) + CalcResidual(facingDirectionCost) * 2.0f;
		}

		lastControlPoint = controlPointPos;
		lastSamplePos = samplePos;
	}

	return cost;
}

float FeatureTrajectory::CalculateFutureFrameCost(size_t frameIndex, const FrameCostContext &context) const {
	return CalculateCost(
		context.m_featureMatrix, frameIndex, m_numFutureSamples,
		[this](size_t frameIndex) {
			return CalcFutureFrameIndex(frameIndex);
		},
		context);
}

float FeatureTrajectory::CalculatePastFrameCost(size_t frameIndex, const FrameCostContext &context) const {
	return CalculateCost(
		context.m_featureMatrix, frameIndex, m_numPastSamples,
		[this](size_t frameIndex) {
			return CalcPastFrameIndex(frameIndex);
		},
		context);
}

size_t FeatureTrajectory::CalcNumSamplesPerFrame() const {
	return m_numPastSamples + 1 + m_numFutureSamples;
}

size_t FeatureTrajectory::CalcMidFrameIndex() const {
	return m_numPastSamples;
}

size_t FeatureTrajectory::CalcPastFrameIndex(size_t historyFrameIndex) const {
	assert(historyFrameIndex < m_numPastSamples && "The history frame index is out of range");
	return m_numPastSamples - historyFrameIndex - 1;
}

size_t FeatureTrajectory::CalcFutureFrameIndex(size_t futureFrameIndex) const {
	assert(futureFrameIndex < m_numFutureSamples && "The future frame index is out of range");
	return CalcMidFrameIndex() + 1 + futureFrameIndex;
}

void FeatureTrajectory::SetFacingAxis(const Axis axis) {
	m_facingAxis = axis;
	UpdateFacingAxis();
}

void FeatureTrajectory::UpdateFacingAxis() {
	switch (m_facingAxis) {
		case Axis::X: {
			m_facingAxisDir = Vector3(1.0f, 0.0f, 0.0f);
			break;
		}
		case Axis::Y: {
			m_facingAxisDir = Vector3(0.0f, 1.0f, 0.0f);
			break;
		}
		case Axis::X_NEGATIVE: {
			m_facingAxisDir = -Vector3(1.0f, 0.0f, 0.0f);
			break;
		}
		case Axis::Y_NEGATIVE: {
			m_facingAxisDir = -Vector3(0.0f, 1.0f, 0.0f);
			break;
		}
		case Axis::Z: {
			m_facingAxisDir = Vector3(0.0f, 0.0f, 1.0f);
			break;
		}
		case Axis::Z_NEGATIVE: {
			m_facingAxisDir = Vector3(0.0f, 0.0f, -1.0f);
			break;
		}
		default: {
			assert(false && "Facing direction axis unknown.");
		}
	}
}

void FeatureTrajectory::SetPastTimeRange(float timeInSeconds) {
	m_pastTimeRange = timeInSeconds;
}

void FeatureTrajectory::SetFutureTimeRange(float timeInSeconds) {
	m_futureTimeRange = timeInSeconds;
}

void FeatureTrajectory::SetNumPastSamplesPerFrame(size_t numHistorySamples) {
	m_numPastSamples = numHistorySamples;
}

void FeatureTrajectory::SetNumFutureSamplesPerFrame(size_t numFutureSamples) {
	m_numFutureSamples = numFutureSamples;
}

void FeatureTrajectory::DebugDrawFacingDirection(
	DebugDrawOutput &debugDisplay,
	const Vector3 &positionWorldSpace,
	const Vector3 &facingDirectionWorldSpace,
	const Color &color) {
	const float length = 0.2f;
	const float radius = 0.01f;

	const Vector3 facingDirectionTarget = positionWorldSpace + facingDirectionWorldSpace * length;

	debugDisplay.cylinder_points.push_back(positionWorldSpace);
	debugDisplay.cylinder_points.push_back(facingDirectionTarget);
	debugDisplay.cylinder_radii.push_back(radius);
	debugDisplay.cylinder_colors.push_back(color);
}

void FeatureTrajectory::DebugDrawFacingDirection(
	DebugDrawOutput &debugDisplay,
	const Transform &worldSpaceTransform,
	const Sample &sample,
	const Vector3 &samplePosWorldSpace,
	const Color &color) const {
	const Vector3 facingDirectionWorldSpace =
		NormalizedSafe(worldSpaceTransform.TransformVector(Vector3(sample.m_facingDirection.x, 0.0f, sample.m_facingDirection.y)));
	DebugDrawFacingDirection(debugDisplay, samplePosWorldSpace, facingDirectionWorldSpace, color);
}

void FeatureTrajectory::DebugDrawTrajectory(
	DebugDrawOutput &debugDisplay,
	const FeatureMatrix &featureMatrix,
	const FeatureMatrixTransformer *featureTransformer,
	size_t frameIndex,
	const Transform &worldSpaceTransform,
	const Color &color,
	size_t numSamples,
	const SplineToFeatureMatrixIndex &splineToFeatureMatrixIndex) const {
	if (frameIndex == InvalidIndex) {
		return;
	}

	constexpr float markerSize = 0.02f;

	Sample nextSample;
	Vector3 nextSamplePos;
	for (size_t i = 0; i < numSamples - 1; ++i) {
		Sample currentSample =
			GetFeatureDataInverseTransformed(featureMatrix, featureTransformer, frameIndex, splineToFeatureMatrixIndex(i));
		nextSample = GetFeatureDataInverseTransformed(featureMatrix, featureTransformer, frameIndex, splineToFeatureMatrixIndex(i + 1));

		const Vector3 currentSamplePos = worldSpaceTransform.TransformPoint(Vector3(currentSample.m_position.x, 0.0f, currentSample.m_position.y));
		nextSamplePos = worldSpaceTransform.TransformPoint(Vector3(nextSample.m_position.x, 0.0f, nextSample.m_position.y));

		debugDisplay.cylinder_points.push_back(currentSamplePos);
		debugDisplay.cylinder_points.push_back(nextSamplePos);
		debugDisplay.cylinder_radii.push_back(0.0025f);
		debugDisplay.cylinder_colors.push_back(color);

		debugDisplay.ball_centers.push_back(currentSamplePos);
		debugDisplay.ball_sizes.push_back(markerSize);
		debugDisplay.ball_colors.push_back(color);
		DebugDrawFacingDirection(debugDisplay, worldSpaceTransform, currentSample, currentSamplePos, color);
	}

	debugDisplay.ball_centers.push_back(nextSamplePos);
	debugDisplay.ball_sizes.push_back(markerSize);
	debugDisplay.ball_colors.push_back(color);
	DebugDrawFacingDirection(debugDisplay, worldSpaceTransform, nextSample, nextSamplePos, color);
}

void FeatureTrajectory::DebugDraw(
	DebugDrawOutput &debugDisplay,
	const Pose &currentPose,
	const FeatureMatrix &featureMatrix,
	const FeatureMatrixTransformer *featureTransformer,
	size_t frameIndex) {
	const Transform transform = currentPose.GetWorldSpaceTransform(m_jointIndex);

	DebugDrawTrajectory(
		debugDisplay, featureMatrix, featureTransformer, frameIndex, transform, m_debugColor, m_numPastSamples,
		std::bind(&FeatureTrajectory::CalcPastFrameIndex, this, std::placeholders::_1));

	DebugDrawTrajectory(
		debugDisplay, featureMatrix, featureTransformer, frameIndex, transform, m_debugColor, m_numFutureSamples,
		std::bind(&FeatureTrajectory::CalcFutureFrameIndex, this, std::placeholders::_1));
}

bool FeatureTrajectory::GetCostFactorVisibility() const {
	return false;
}

int64_t FeatureTrajectory::get_num_past_samples() const {
	return static_cast<int64_t>(GetNumPastSamples());
}

int64_t FeatureTrajectory::get_num_future_samples() const {
	return static_cast<int64_t>(GetNumFutureSamples());
}

void FeatureTrajectory::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_past_time_range"), &FeatureTrajectory::GetPastTimeRange);
	ClassDB::bind_method(D_METHOD("get_future_time_range"), &FeatureTrajectory::GetFutureTimeRange);
	ClassDB::bind_method(D_METHOD("get_num_past_samples"), &FeatureTrajectory::get_num_past_samples);
	ClassDB::bind_method(D_METHOD("get_num_future_samples"), &FeatureTrajectory::get_num_future_samples);
	ClassDB::bind_method(D_METHOD("get_past_cost_factor"), &FeatureTrajectory::GetPastCostFactor);
	ClassDB::bind_method(D_METHOD("get_future_cost_factor"), &FeatureTrajectory::GetFutureCostFactor);
	ClassDB::bind_method(D_METHOD("get_facing_axis_dir"), &FeatureTrajectory::GetFacingAxisDir);
}

bool FeatureTrajectory::_set(const StringName &p_name, const Variant &p_value) {
	const String key = String(p_name);
	if (key == "past_time_range") {
		m_pastTimeRange = float(p_value);
		return true;
	}
	if (key == "num_past_samples") {
		m_numPastSamples = (size_t)(int64_t)p_value;
		return true;
	}
	if (key == "past_cost_factor") {
		m_pastCostFactor = float(p_value);
		return true;
	}
	if (key == "future_time_range") {
		m_futureTimeRange = float(p_value);
		return true;
	}
	if (key == "num_future_samples") {
		m_numFutureSamples = (size_t)(int64_t)p_value;
		return true;
	}
	if (key == "future_cost_factor") {
		m_futureCostFactor = float(p_value);
		return true;
	}
	if (key == "facing_axis") {
		SetFacingAxis((Axis)(int64_t)p_value);
		return true;
	}
	return Feature::_set(p_name, p_value);
}

bool FeatureTrajectory::_get(const StringName &p_name, Variant &r_ret) const {
	const String key = String(p_name);
	if (key == "past_time_range") {
		r_ret = m_pastTimeRange;
		return true;
	}
	if (key == "num_past_samples") {
		r_ret = (int64_t)m_numPastSamples;
		return true;
	}
	if (key == "past_cost_factor") {
		r_ret = m_pastCostFactor;
		return true;
	}
	if (key == "future_time_range") {
		r_ret = m_futureTimeRange;
		return true;
	}
	if (key == "num_future_samples") {
		r_ret = (int64_t)m_numFutureSamples;
		return true;
	}
	if (key == "future_cost_factor") {
		r_ret = m_futureCostFactor;
		return true;
	}
	if (key == "facing_axis") {
		r_ret = (int64_t)m_facingAxis;
		return true;
	}
	return Feature::_get(p_name, r_ret);
}

void FeatureTrajectory::AppendCustomProperties(List<PropertyInfo> *p_list) const {

	p_list->push_back(PropertyInfo(Variant::INT, "num_past_samples", PROPERTY_HINT_RANGE, "2,100,1"));
	p_list->push_back(PropertyInfo(Variant::FLOAT, "past_time_range", PROPERTY_HINT_RANGE, "0.01,10,0.1"));
	p_list->push_back(PropertyInfo(Variant::FLOAT, "past_cost_factor", PROPERTY_HINT_RANGE, "0,100,0.1"));
	p_list->push_back(PropertyInfo(Variant::INT, "num_future_samples", PROPERTY_HINT_RANGE, "2,100,1"));
	p_list->push_back(PropertyInfo(Variant::FLOAT, "future_time_range", PROPERTY_HINT_RANGE, "0.01,10,0.1"));
	p_list->push_back(PropertyInfo(Variant::FLOAT, "future_cost_factor", PROPERTY_HINT_RANGE, "0,100,0.1"));
	p_list->push_back(PropertyInfo(Variant::INT, "facing_axis", PROPERTY_HINT_ENUM, "X:0,Y:1,-X:2,-Y:3,Z:4,-Z:5"));
}

size_t FeatureTrajectory::GetNumDimensions() const {
	return CalcNumSamplesPerFrame() * Sample::s_componentsPerSample;
}

std::string FeatureTrajectory::GetDimensionName(size_t index) const {
	std::string result = "Trajectory";

	const int sampleIndex = static_cast<int>(index) / static_cast<int>(Sample::s_componentsPerSample);
	const int componentIndex = index % Sample::s_componentsPerSample;
	const int midSampleIndex = static_cast<int>(CalcMidFrameIndex());

	if (sampleIndex == midSampleIndex) {
		result += ".Current.";
	} else if (sampleIndex < midSampleIndex) {
		result += ".Past" + std::to_string(sampleIndex - static_cast<int>(m_numPastSamples)) + ".";
	} else {
		result += ".Future" + std::to_string(sampleIndex - static_cast<int>(m_numPastSamples)) + ".";
	}

	switch (componentIndex) {
		case 0: {
			result += "PosX";
			break;
		}
		case 1: {
			result += "PosZ";
			break;
		}
		case 2: {
			result += "FacingDirX";
			break;
		}
		case 3: {
			result += "FacingDirZ";
			break;
		}
		default: {
			result += Feature::GetDimensionName(index);
		}
	}

	return result;
}

FeatureTrajectory::Sample FeatureTrajectory::GetFeatureData(
	const FeatureMatrix &featureMatrix, size_t frameIndex, size_t sampleIndex) const {
	const size_t columnOffset = m_featureColumnOffset + sampleIndex * Sample::s_componentsPerSample;
	return {
		featureMatrix.GetVector2(frameIndex, columnOffset + 0),
		featureMatrix.GetVector2(frameIndex, columnOffset + 2),
	};
}

FeatureTrajectory::Sample FeatureTrajectory::GetFeatureDataInverseTransformed(
	const FeatureMatrix &featureMatrix, const FeatureMatrixTransformer *featureTransformer, size_t frameIndex, size_t sampleIndex) const {
	Sample sample = GetFeatureData(featureMatrix, frameIndex, sampleIndex);
	if (featureTransformer) {
		const size_t columnOffset = m_featureColumnOffset + sampleIndex * Sample::s_componentsPerSample;

		sample.m_position = featureTransformer->InverseTransform(sample.m_position, columnOffset + 0);
		sample.m_facingDirection = featureTransformer->InverseTransform(sample.m_facingDirection, columnOffset + 2);
	}
	return sample;
}

void FeatureTrajectory::SetFeatureData(FeatureMatrix &featureMatrix, size_t frameIndex, size_t sampleIndex, const Sample &sample) {
	const size_t columnOffset = m_featureColumnOffset + sampleIndex * Sample::s_componentsPerSample;
	featureMatrix.SetVector2(frameIndex, columnOffset + 0, sample.m_position);
	featureMatrix.SetVector2(frameIndex, columnOffset + 2, sample.m_facingDirection);
}

}