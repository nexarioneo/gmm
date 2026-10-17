#pragma once

#include <cstddef>
#include <memory>
#include <vector>

#include "../math/FeatureMatrix.h"
#include "../math/FeatureMatrixTransformer.h"
#include "../features/FeatureSchema.h"
#include "FrameDatabase.h"
#include "KdTree.h"

class ActorInstance;
class Motion;

namespace godot::MotionMatching {

class MotionMatchingData {
public:
	MotionMatchingData(const FeatureSchema& featureSchema);
	virtual ~MotionMatchingData();

	enum FeatureScalerType {
		StandardScalerType = 0,
		MinMaxScalerType = 1
	};

	struct InitSettings {
		ActorInstance* m_actorInstance = nullptr;
		std::vector<Motion*> m_motionList;
		FrameDatabase::FrameImportSettings m_frameImportSettings;
		size_t m_maxKdTreeDepth = 20;
		size_t m_minFramesPerKdTreeNode = 1000;
		bool m_importMirrored = false;

		bool m_normalizeData = false;
		FeatureScalerType m_featureScalerType = StandardScalerType;
		FeatureMatrixTransformer::Settings m_featureTansformerSettings = {};
	};
	bool Init(const InitSettings& settings);

	bool AppendMotion(Motion *motion);
	bool IsInitialized() const { return m_initialized; }

	void Clear();

	const FrameDatabase& GetFrameDatabase() const { return m_frameDatabase; }
	FrameDatabase& GetFrameDatabase() { return m_frameDatabase; }
	const FeatureSchema& GetFeatureSchema() const { return m_featureSchema; }
	const FeatureMatrix& GetFeatureMatrix() const { return m_featureMatrix; }
	FeatureMatrixTransformer* GetFeatureTransformer() { return m_featureTransformer.get(); }
	FeatureMatrixTransformer* GetFeatureTransformer() const { return m_featureTransformer.get(); }
	const KdTree& GetKdTree() const { return *m_kdTree.get(); }
	const std::vector<Ref<Feature>>& GetFeaturesInKdTree() const { return m_featuresInKdTree; }

protected:

	bool ExtractFeatures(ActorInstance* actorInstance, FrameDatabase* frameDatabase);

	static void ExtractFeatureValuesRange(ActorInstance* actorInstance, FrameDatabase& frameDatabase, const FeatureSchema& featureSchema, FeatureMatrix& featureMatrix, size_t startFrame, size_t endFrame);

	void ExtractFeatureRange(ActorInstance *actorInstance, size_t startFrame, size_t endFrame);

	bool RebuildDerivedData(const InitSettings &settings);

	const size_t s_numFramesPerBatch = 1000;

	FrameDatabase m_frameDatabase;

	const FeatureSchema& m_featureSchema;
	FeatureMatrix m_featureMatrix;
	std::unique_ptr<FeatureMatrixTransformer> m_featureTransformer;

	std::unique_ptr<KdTree> m_kdTree;
	std::vector<Ref<Feature>> m_featuresInKdTree;

	InitSettings m_lastSettings;
	bool m_initialized = false;
};
}
