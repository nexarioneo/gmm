#include "MotionMatchingData.h"

#include <algorithm>
#include <atomic>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <thread>
#include <tuple>
#include <vector>

#include <godot_cpp/classes/object.hpp>
#include <godot_cpp/classes/project_settings.hpp>
#include <godot_cpp/classes/time.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#include "ActorInstance.h"
#include "AnimGraphPose.h"
#include "AnimGraphPosePool.h"
#include "../features/Feature.h"
#include "../math/FeatureMatrixMinMaxScaler.h"
#include "../math/FeatureMatrixStandardScaler.h"
#include "../features/FeatureTrajectory.h"
#include "Frame.h"
#include "FrameDatabase.h"
#include "KdTree.h"
#include "Motion.h"
#include "Pose.h"

namespace godot::MotionMatching {

static bool GetMultiThreadedInitialization() {
	ProjectSettings *settings = ProjectSettings::get_singleton();
	if (settings == nullptr) {
		return true;
	}
	return bool(settings->get_setting("motion_matching/multi_threaded_initialization", true));
}

MotionMatchingData::MotionMatchingData(const FeatureSchema& featureSchema)
	: m_featureSchema(featureSchema) {
	m_kdTree = std::make_unique<KdTree>();
}

MotionMatchingData::~MotionMatchingData() {
	Clear();
}

bool MotionMatchingData::ExtractFeatures(ActorInstance* actorInstance, FrameDatabase* frameDatabase) {
	if (frameDatabase == nullptr) {
		return false;
	}
	Time *timeSingleton = Time::get_singleton();
	const uint64_t timeBefore = timeSingleton ? timeSingleton->get_ticks_msec() : 0;

	const size_t numFrames = frameDatabase->GetNumFrames();
	if (numFrames == 0) {
		return true;
	}

	FeatureMatrix::Index featureComponentCount = 0;
	for (const Ref<Feature> &feature : m_featureSchema.GetFeatures()) {
		if (feature.is_null()) {
			continue;
		}
		Feature::InitSettings frameSettings;
		frameSettings.m_actorInstance = actorInstance;
		if (!feature->Init(frameSettings)) {
			return false;
		}

		feature->SetColumnOffset(featureComponentCount);
		featureComponentCount += feature->GetNumDimensions();
	}

	m_featureMatrix.resize(numFrames, featureComponentCount);

	ExtractFeatureRange(actorInstance, 0, numFrames);

	Time *timeSingletonAfter = Time::get_singleton();
	const uint64_t timeAfter = timeSingletonAfter ? timeSingletonAfter->get_ticks_msec() : timeBefore;
	const float extractFeaturesTimeMs = static_cast<float>(timeAfter >= timeBefore ? timeAfter - timeBefore : 0);
	UtilityFunctions::print_verbose(String("Motion Matching: Extracting features for ") + String::num_int64((int64_t)m_featureMatrix.rows()) + String(" frames took ") + String::num(extractFeaturesTimeMs, 2) + String(" ms."));
	return true;
}

void MotionMatchingData::ExtractFeatureRange(ActorInstance *actorInstance, size_t startFrame, size_t endFrame) {
	if (startFrame >= endFrame) {
		return;
	}
	const size_t numFrames = endFrame - startFrame;

	if (GetMultiThreadedInitialization()) {
		const size_t numBatches = static_cast<size_t>(std::ceil(static_cast<float>(numFrames) / static_cast<float>(s_numFramesPerBatch)));

		if (numBatches > 1) {
			size_t numThreads = static_cast<size_t>(std::thread::hardware_concurrency());
			if (numThreads == 0) {
				numThreads = 1;
			}
			numThreads = std::min(numBatches, numThreads);

			std::atomic<size_t> nextBatchIndex(0);
			std::vector<std::thread> threads;
			threads.reserve(numThreads);
			for (size_t threadIndex = 0; threadIndex < numThreads; ++threadIndex) {
				threads.emplace_back([this, actorInstance, startFrame, numBatches, numFrames, &nextBatchIndex]() {
					while (true) {
						const size_t batchIndex = nextBatchIndex.fetch_add(1, std::memory_order_relaxed);
						if (batchIndex >= numBatches) {
							break;
						}
						const size_t batchStart = startFrame + batchIndex * s_numFramesPerBatch;
						const size_t batchEnd = std::min(batchStart + s_numFramesPerBatch, startFrame + numFrames);
						ExtractFeatureValuesRange(actorInstance, m_frameDatabase, m_featureSchema, m_featureMatrix, batchStart, batchEnd);
					}
				});
			}
			for (std::thread& thread : threads) {
				thread.join();
			}
		} else {
			ExtractFeatureValuesRange(actorInstance, m_frameDatabase, m_featureSchema, m_featureMatrix, startFrame, endFrame);
		}
	} else {
		ExtractFeatureValuesRange(actorInstance, m_frameDatabase, m_featureSchema, m_featureMatrix, startFrame, endFrame);
	}
}

void MotionMatchingData::ExtractFeatureValuesRange(ActorInstance* actorInstance, FrameDatabase& frameDatabase, const FeatureSchema& featureSchema, FeatureMatrix& featureMatrix, size_t startFrame, size_t endFrame) {

	AnimGraphPosePool posePool;
	AnimGraphPose* pose = posePool.RequestPose(actorInstance);
	if (pose == nullptr) {
		return;
	}

	Feature::ExtractFeatureContext context(featureMatrix, posePool);
	context.m_frameDatabase = &frameDatabase;
	context.m_framePose = &pose->GetPose();
	context.m_actorInstance = actorInstance;
	const auto& frames = frameDatabase.GetFrames();

	for (size_t frameIndex = startFrame; frameIndex < endFrame; ++frameIndex) {
		const Frame& frame = frames[frameIndex];
		context.m_frameIndex = frame.GetFrameIndex();

		frame.SamplePose(const_cast<Pose*>(context.m_framePose));

		{
			for (const Ref<Feature> &feature : featureSchema.GetFeatures()) {
				if (feature.is_valid()) {
					feature->ExtractFeatureValues(context);
				}
			}
		}
	}

	posePool.FreePose(pose);
}

bool MotionMatchingData::Init(const InitSettings& settings) {
	Time *timeSingletonInit = Time::get_singleton();
	const uint64_t timeBefore = timeSingletonInit ? timeSingletonInit->get_ticks_msec() : 0;

	size_t totalNumFramesImported = 0;
	size_t totalNumFramesDiscarded = 0;
	for (Motion* motion : settings.m_motionList) {
		if (motion == nullptr) {
			continue;
		}
		size_t numFrames = 0;
		size_t numDiscarded = 0;
		std::tie(numFrames, numDiscarded) = m_frameDatabase.ImportFrames(motion, settings.m_frameImportSettings, false);
		totalNumFramesImported += numFrames;
		totalNumFramesDiscarded += numDiscarded;

		if (settings.m_importMirrored) {
			std::tie(numFrames, numDiscarded) = m_frameDatabase.ImportFrames(motion, settings.m_frameImportSettings, true);
			totalNumFramesImported += numFrames;
			totalNumFramesDiscarded += numDiscarded;
		}
	}

	if (totalNumFramesImported > 0 || totalNumFramesDiscarded > 0) {
		const float sampleRate = static_cast<float>(settings.m_frameImportSettings.m_sampleRate);
		UtilityFunctions::print_verbose(String("Motion Matching: Imported a total of ") + String::num_int64((int64_t)totalNumFramesImported) + String(" frames (") + String::num_int64((int64_t)totalNumFramesDiscarded) + String(" frames discarded) across ") + String::num_int64((int64_t)settings.m_motionList.size()) + String(" motions. This is ") + String::num(static_cast<float>(totalNumFramesImported) / sampleRate, 2) + String(" seconds (") + String::num(static_cast<float>(totalNumFramesImported) / sampleRate / 60.0f, 2) + String(" minutes) of motion data."));
	}

	if (!ExtractFeatures(settings.m_actorInstance, &m_frameDatabase)) {
		UtilityFunctions::push_error("Motion Matching: Failed to extract features from motion database.");
		return false;
	}

	if (!RebuildDerivedData(settings)) {
		return false;
	}

	Time *timeSingletonInitAfter = Time::get_singleton();
	const uint64_t timeInitAfter = timeSingletonInitAfter ? timeSingletonInitAfter->get_ticks_msec() : timeBefore;
	const float initTimeMs = static_cast<float>(timeInitAfter >= timeBefore ? timeInitAfter - timeBefore : 0);
	UtilityFunctions::print_verbose(String("Motion Matching: Feature matrix (") + String::num_int64((int64_t)m_featureMatrix.rows()) + String(", ") + String::num_int64((int64_t)m_featureMatrix.cols()) + String(") uses ") + String::num(static_cast<float>(m_featureMatrix.CalcMemoryUsageInBytes()) / 1024.0f / 1024.0f, 2) + String(" MB and took ") + String::num(initTimeMs, 2) + String(" ms to initialize (including initialization of acceleration structures)."));

	m_lastSettings = settings;
	m_initialized = true;
	return true;
}

bool MotionMatchingData::AppendMotion(Motion *motion) {
	if (!m_initialized) {
		UtilityFunctions::push_error("Motion Matching: AppendMotion needs a successful Init first.");
		return false;
	}
	if (motion == nullptr) {
		return false;
	}
	ActorInstance *actorInstance = m_lastSettings.m_actorInstance;
	const size_t oldRows = m_frameDatabase.GetNumFrames();
	size_t numFrames = 0;
	size_t numDiscarded = 0;
	std::tie(numFrames, numDiscarded) = m_frameDatabase.ImportFrames(motion, m_lastSettings.m_frameImportSettings, false);
	size_t newRows = m_frameDatabase.GetNumFrames();
	if (m_lastSettings.m_importMirrored) {
		std::tie(numFrames, numDiscarded) = m_frameDatabase.ImportFrames(motion, m_lastSettings.m_frameImportSettings, true);
		newRows = m_frameDatabase.GetNumFrames();
	}
	if (newRows <= oldRows) {
		return true;
	}

	m_featureMatrix.resize(newRows, m_featureMatrix.cols());
	ExtractFeatureRange(actorInstance, oldRows, newRows);
	if (!RebuildDerivedData(m_lastSettings)) {
		return false;
	}
	m_lastSettings.m_motionList.push_back(motion);
	return true;
}

bool MotionMatchingData::RebuildDerivedData(const InitSettings &settings) {

	if (settings.m_normalizeData) {
		Time *timeSingletonTransform = Time::get_singleton();
		const uint64_t transformTimeBefore = timeSingletonTransform ? timeSingletonTransform->get_ticks_msec() : 0;

		switch (settings.m_featureScalerType) {
			case FeatureScalerType::StandardScalerType: {
				m_featureTransformer.reset(new StandardScaler());
				break;
			}
			case FeatureScalerType::MinMaxScalerType: {
				m_featureTransformer.reset(new MinMaxScaler());
				break;
			}
			default: {
				m_featureTransformer.reset();
				UtilityFunctions::push_error("Motion Matching: Unknown feature scaler type.");
				break;
			}
		}

		if (m_featureTransformer) {
			m_featureTransformer->Fit(m_featureMatrix, settings.m_featureTansformerSettings);
			m_featureMatrix = m_featureTransformer->Transform(m_featureMatrix);
		}

		Time *timeSingletonTransformAfter = Time::get_singleton();
		const uint64_t transformTimeAfter = timeSingletonTransformAfter ? timeSingletonTransformAfter->get_ticks_msec() : transformTimeBefore;
		const float transformFeatureTimeMs = static_cast<float>(transformTimeAfter >= transformTimeBefore ? transformTimeAfter - transformTimeBefore : 0);
		UtilityFunctions::print_verbose(String("Motion Matching: Transforming/normalizing features took ") + String::num(transformFeatureTimeMs, 2) + String(" ms."));
	} else {
		m_featureTransformer.reset();
	}

	m_featuresInKdTree.clear();

	for (const Ref<Feature> &feature : m_featureSchema.GetFeatures()) {
		if (feature.is_valid() && Object::cast_to<FeatureTrajectory>(feature.ptr()) == nullptr) {
			m_featuresInKdTree.push_back(feature);
		}
	}

	if (!m_kdTree->Init(m_frameDatabase, m_featureMatrix, m_featuresInKdTree, settings.m_maxKdTreeDepth, settings.m_minFramesPerKdTreeNode)) {
		UtilityFunctions::push_error("Motion Matching: Failed to initialize KdTree acceleration structure.");
		return false;
	}
	return true;
}

void MotionMatchingData::Clear() {
	m_frameDatabase.Clear();
	m_featureMatrix.Clear();
	if (m_kdTree) {
		m_kdTree->Clear();
	}
	m_featuresInKdTree.clear();
	m_lastSettings = InitSettings();
	m_initialized = false;
}
}
