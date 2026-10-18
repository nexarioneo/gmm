#pragma once

#include <cstddef>
#include <vector>

#include <godot_cpp/classes/ref.hpp>

#include "features/feature.h"
#include "core/math/feature_matrix.h"
#include "database/frame_database.h"

namespace godot::MotionMatching {
class KdTree {
public:
	KdTree() = default;
	virtual ~KdTree();

	bool Init(const FrameDatabase &frameDatabase,
		const FeatureMatrix &featureMatrix,
		const std::vector<Ref<Feature>> &features,
		size_t maxDepth = 10,
		size_t minFramesPerLeaf = 1000);

	static size_t CalcNumDimensions(const std::vector<Ref<Feature>> &features);

	void Clear();
	void PrintStats();

	size_t GetNumNodes() const;
	size_t GetNumDimensions() const;
	size_t CalcMemoryUsageInBytes() const;
	bool IsInitialized() const { return !m_nodes.empty(); }

	void FindNearestNeighbors(const std::vector<float> &frameFloats, std::vector<size_t> &resultFrameIndices) const;

private:
	struct Node {
		virtual ~Node() = default;

		Node *m_leftNode = nullptr;
		Node *m_rightNode = nullptr;
		Node *m_parent = nullptr;
		float m_median = 0.0f;
		size_t m_dimension = 0;
		std::vector<size_t> m_frames;
	};

	void BuildTreeNodes(const FrameDatabase &frameDatabase,
		const FeatureMatrix &featureMatrix,
		const std::vector<size_t> &localToSchemaFeatureColumns,
		Node *node,
		Node *parent,
		size_t dimension = 0,
		bool leftSide = true);
	void FillFramesForNode(Node *node,
		const FrameDatabase &frameDatabase,
		const FeatureMatrix &featureMatrix,
		const std::vector<size_t> &localToSchemaFeatureColumns,
		std::vector<float> &frameFeatureValues,
		Node *parent,
		bool leftSide);
	void RecursiveCalcNumFrames(Node *node, size_t &outNumFrames) const;
	void ClearFramesForNonEssentialNodes();
	void MergeSmallLeafNodesToParents();
	void RemoveZeroFrameLeafNodes();
	void RemoveLeafNode(Node *node);
	void FindNearestNeighbors(Node *node, const std::vector<float> &frameFloats, std::vector<size_t> &resultFrameIndices) const;
	std::vector<size_t> CalcLocalToSchemaFeatureColumns(const std::vector<Ref<Feature>> &features) const;

private:
	std::vector<Node *> m_nodes;
	size_t m_numDimensions = 0;
	size_t m_maxDepth = 20;
	size_t m_minFramesPerLeaf = 1000;
};
}
