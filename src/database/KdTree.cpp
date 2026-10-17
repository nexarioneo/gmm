#include "KdTree.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <string>

#include <godot_cpp/classes/time.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#include "../features/Feature.h"

namespace godot::MotionMatching {

KdTree::~KdTree() {
	Clear();
}

size_t KdTree::CalcNumDimensions(const std::vector<Ref<Feature>> &features) {
	size_t result = 0;
	for (const Ref<Feature> &feature : features) {
		if (feature.is_null() || feature->GetId().empty()) {
			continue;
		}

		result += feature->GetNumDimensions();
	}
	return result;
}

bool KdTree::Init(const FrameDatabase &frameDatabase,
	const FeatureMatrix &featureMatrix,
	const std::vector<Ref<Feature>> &features,
	size_t maxDepth,
	size_t minFramesPerLeaf) {
	Time *timeSingleton = Time::get_singleton();
	const uint64_t timeBefore = timeSingleton ? timeSingleton->get_ticks_msec() : 0;

	Clear();

	m_numDimensions = CalcNumDimensions(features);

	const size_t maxNumDimensions = 48;
	if (m_numDimensions == 0 || m_numDimensions > maxNumDimensions) {
		UtilityFunctions::push_error(String("Motion Matching: Cannot initialize KD-tree. KD-tree dimension (") + String::num_int64((int64_t)m_numDimensions) + String(") has to be between 1 and ") + String::num_int64((int64_t)maxNumDimensions) + String(". Please use Feature::SetIncludeInKdTree(false) on some features."));
		Clear();
		return false;
	}

	if (minFramesPerLeaf > 100000) {
		UtilityFunctions::push_error(String("Motion Matching: KdTree minFramesPerLeaf (") + String::num_int64((int64_t)minFramesPerLeaf) + String(") cannot be bigger than 100000."));
		Clear();
		return false;
	}

	if (maxDepth == 0) {
		UtilityFunctions::push_error(String("Motion Matching: KdTree max depth (") + String::num_int64((int64_t)maxDepth) + String(") cannot be zero"));
		Clear();
		return false;
	}

	if (frameDatabase.GetNumFrames() == 0) {
		UtilityFunctions::push_error("Motion Matching: Skipping to initialize KD-tree. No frames in the motion database.");
		Clear();
		return true;
	}

	if (featureMatrix.rows() < frameDatabase.GetNumFrames()) {
		UtilityFunctions::push_error("Motion Matching: Feature matrix has fewer rows than frames; cannot build KD-tree.");
		Clear();
		return false;
	}

	m_maxDepth = maxDepth;
	m_minFramesPerLeaf = minFramesPerLeaf;

	const std::vector<size_t> localToSchemaFeatureColumns = CalcLocalToSchemaFeatureColumns(features);

	BuildTreeNodes(frameDatabase, featureMatrix, localToSchemaFeatureColumns, new Node(), nullptr, 0);
	MergeSmallLeafNodesToParents();
	ClearFramesForNonEssentialNodes();
	RemoveZeroFrameLeafNodes();

	Time *timeSingletonAfter = Time::get_singleton();
	const uint64_t timeAfter = timeSingletonAfter ? timeSingletonAfter->get_ticks_msec() : timeBefore;
	const float initTimeMs = (float)(timeAfter >= timeBefore ? timeAfter - timeBefore : 0);
	UtilityFunctions::print_verbose(String("Motion Matching: KD-Tree initialized in ") + String::num(initTimeMs, 2) + String(" ms (numNodes = ") + String::num_int64((int64_t)m_nodes.size()) + String("  numDims = ") + String::num_int64((int64_t)m_numDimensions) + String("  Memory used = ") + String::num((float)CalcMemoryUsageInBytes() / 1024.0f / 1024.0f, 2) + String(" MB)."));

	PrintStats();
	return true;
}

std::vector<size_t> KdTree::CalcLocalToSchemaFeatureColumns(const std::vector<Ref<Feature>> &features) const {
	std::vector<size_t> localToSchemaFeatureColumns;
	localToSchemaFeatureColumns.resize(m_numDimensions);

	size_t currentColumn = 0;
	for (const Ref<Feature> &feature : features) {
		if (feature.is_null() || feature->GetId().empty()) {
			continue;
		}
		const size_t numDimensions = feature->GetNumDimensions();
		const size_t featureColumnOffset = feature->GetColumnOffset();
		for (size_t i = 0; i < numDimensions; ++i) {
			localToSchemaFeatureColumns[currentColumn] = featureColumnOffset + i;
			currentColumn++;
		}
	}

	assert(m_numDimensions == currentColumn && "There should be a column index mapping for each of the available dimensions.");
	return localToSchemaFeatureColumns;
}

void KdTree::Clear() {

	for (Node *node : m_nodes) {
		delete node;
	}

	m_nodes.clear();
	m_numDimensions = 0;
}

size_t KdTree::CalcMemoryUsageInBytes() const {
	size_t totalBytes = 0;

	for (const Node *node : m_nodes) {
		totalBytes += sizeof(Node);
		totalBytes += node->m_frames.capacity() * sizeof(size_t);
	}

	totalBytes += sizeof(KdTree);
	return totalBytes;
}

size_t KdTree::GetNumNodes() const {
	return m_nodes.size();
}

size_t KdTree::GetNumDimensions() const {
	return m_numDimensions;
}

void KdTree::BuildTreeNodes(const FrameDatabase &frameDatabase,
	const FeatureMatrix &featureMatrix,
	const std::vector<size_t> &localToSchemaFeatureColumns,
	Node *node,
	Node *parent,
	size_t dimension,
	bool leftSide) {
	node->m_parent = parent;
	node->m_dimension = dimension;
	m_nodes.emplace_back(node);

	std::vector<float> frameFeatureValues;
	FillFramesForNode(node, frameDatabase, featureMatrix, localToSchemaFeatureColumns, frameFeatureValues, parent, leftSide);

	const size_t maxDimensions = std::min(m_numDimensions, m_maxDepth);
	if (node->m_frames.size() < m_minFramesPerLeaf * 2 ||
		dimension >= maxDimensions) {
		return;
	}

	Node *leftNode = new Node();
	assert(!node->m_leftNode && "Expected the parent left node to be a nullptr");
	node->m_leftNode = leftNode;
	BuildTreeNodes(frameDatabase, featureMatrix, localToSchemaFeatureColumns, leftNode, node, dimension + 1, true);

	Node *rightNode = new Node();
	assert(!node->m_rightNode && "Expected the parent right node to be a nullptr");
	node->m_rightNode = rightNode;
	BuildTreeNodes(frameDatabase, featureMatrix, localToSchemaFeatureColumns, rightNode, node, dimension + 1, false);
}

void KdTree::ClearFramesForNonEssentialNodes() {
	for (Node *node : m_nodes) {
		if (node->m_leftNode && node->m_rightNode) {
			node->m_frames.clear();
			node->m_frames.shrink_to_fit();
		}
	}
}

void KdTree::RemoveLeafNode(Node *node) {
	if (node == nullptr) {
		return;
	}
	Node *parent = node->m_parent;

	if (parent != nullptr) {
		if (parent->m_leftNode == node) {
			parent->m_leftNode = nullptr;
		}

		if (parent->m_rightNode == node) {
			parent->m_rightNode = nullptr;
		}
	}

	const auto location = std::find(m_nodes.begin(), m_nodes.end(), node);
	assert(location != m_nodes.end() && "Expected to find the item to remove.");
	m_nodes.erase(location);

	delete node;
}

void KdTree::MergeSmallLeafNodesToParents() {

	if (m_nodes.size() < 2) {
		return;
	}

	std::vector<Node *> nodesToRemove;
	for (Node *node : m_nodes) {

		if ((!node->m_leftNode && !node->m_rightNode) &&
			node->m_frames.size() < m_minFramesPerLeaf) {
			nodesToRemove.emplace_back(node);
		}
	}

	for (Node *node : nodesToRemove) {
		RemoveLeafNode(node);
	}
}

void KdTree::RemoveZeroFrameLeafNodes() {
	std::vector<Node *> nodesToRemove;

	for (Node *node : m_nodes) {
		if ((!node->m_leftNode && !node->m_rightNode) &&
			node->m_frames.empty()) {
			nodesToRemove.emplace_back(node);
		}
	}

	for (Node *node : nodesToRemove) {
		RemoveLeafNode(node);
	}
}

void KdTree::FillFramesForNode(Node *node,
	const FrameDatabase &frameDatabase,
	const FeatureMatrix &featureMatrix,
	const std::vector<size_t> &localToSchemaFeatureColumns,
	std::vector<float> &frameFeatureValues,
	Node *parent,
	bool leftSide) {
	frameFeatureValues.clear();

	if (parent) {
		const size_t numExpectedFrames = (parent->m_frames.size() / 2) + 1;
		node->m_frames.reserve(numExpectedFrames);

		for (const size_t frameIndex : parent->m_frames) {
			if (parent->m_dimension >= localToSchemaFeatureColumns.size()) {
				break;
			}
			const float parentFeatureValue = featureMatrix(frameIndex, localToSchemaFeatureColumns[parent->m_dimension]);
			if (leftSide) {
				if (parentFeatureValue <= parent->m_median) {
					node->m_frames.emplace_back(frameIndex);
				}
			} else {
				if (parentFeatureValue > parent->m_median) {
					node->m_frames.emplace_back(frameIndex);
				}
			}
		}

		if (node->m_dimension < localToSchemaFeatureColumns.size()) {
			const size_t col = localToSchemaFeatureColumns[node->m_dimension];
			frameFeatureValues.reserve(node->m_frames.size());
			for (const size_t frameIndex : node->m_frames) {
				frameFeatureValues.push_back(featureMatrix(frameIndex, col));
			}
		}
	} else {
		node->m_frames.reserve(frameDatabase.GetNumFrames());
		frameFeatureValues.reserve(frameDatabase.GetNumFrames());
		assert(!localToSchemaFeatureColumns.empty() && node->m_dimension < localToSchemaFeatureColumns.size() && "Root dimension has no schema column mapping.");
		const size_t col = localToSchemaFeatureColumns[node->m_dimension];
		for (const Frame &frame : frameDatabase.GetFrames()) {
			const size_t frameIndex = frame.GetFrameIndex();
			node->m_frames.emplace_back(frameIndex);
			frameFeatureValues.push_back(featureMatrix(frameIndex, col));
		}
	}

	node->m_median = 0.0f;
	if (!frameFeatureValues.empty()) {
		auto medianIterator = frameFeatureValues.begin() + frameFeatureValues.size() / 2;
		std::nth_element(frameFeatureValues.begin(), medianIterator, frameFeatureValues.end());
		node->m_median = frameFeatureValues[frameFeatureValues.size() / 2];
	}
}

void KdTree::RecursiveCalcNumFrames(Node *node, size_t &outNumFrames) const {
	if (node->m_leftNode && node->m_rightNode) {
		RecursiveCalcNumFrames(node->m_leftNode, outNumFrames);
		RecursiveCalcNumFrames(node->m_rightNode, outNumFrames);
	} else {
		outNumFrames += node->m_frames.size();
	}
}

void KdTree::PrintStats() {
	if (m_nodes.empty()) {
		return;
	}
	size_t leftNumFrames = 0;
	size_t rightNumFrames = 0;
	if (m_nodes[0]->m_leftNode) {
		RecursiveCalcNumFrames(m_nodes[0]->m_leftNode, leftNumFrames);
	}

	if (m_nodes[0]->m_rightNode) {
		RecursiveCalcNumFrames(m_nodes[0]->m_rightNode, rightNumFrames);
	}

	const float numFrames = static_cast<float>(leftNumFrames + rightNumFrames);
	const float halfFrames = numFrames / 2.0f;

	const float balanceScore = (numFrames > 0.0f) ? 100.0f - (std::abs(halfFrames - static_cast<float>(leftNumFrames)) / numFrames) * 100.0f : 100.0f;

	size_t maxDepth = 0;
	for (const Node *node : m_nodes) {
		maxDepth = std::max(maxDepth, node->m_dimension);
	}

	UtilityFunctions::print_verbose(String("Motion Matching:     KdTree Balance Info: leftSide=") + String::num_int64((int64_t)leftNumFrames) + String(" rightSide=") + String::num_int64((int64_t)rightNumFrames) + String(" score=") + String::num(balanceScore, 2) + String(" totalFrames=") + String::num_int64((int64_t)(leftNumFrames + rightNumFrames)) + String(" maxDepth=") + String::num_int64((int64_t)maxDepth));

	size_t numLeafNodes = 0;
	size_t numZeroNodes = 0;
	size_t minFrames = 1000000000;
	size_t maxFrames = 0;
	std::string framesString;
	for (const Node *node : m_nodes) {
		if (node->m_leftNode || node->m_rightNode) {
			continue;
		}

		numLeafNodes++;

		if (node->m_frames.empty()) {
			numZeroNodes++;
		}

		if (!framesString.empty()) {
			framesString += ", ";
		}
		framesString += std::to_string(node->m_frames.size());

		minFrames = std::min(minFrames, node->m_frames.size());
		maxFrames = std::max(maxFrames, node->m_frames.size());
	}
	UtilityFunctions::print_verbose(String("Motion Matching:     Frames = (") + String(framesString.c_str()) + String(")"));

	const size_t avgFrames = (numLeafNodes > 0) ? (leftNumFrames + rightNumFrames) / numLeafNodes : 0;
	UtilityFunctions::print_verbose(String("Motion Matching:     KdTree Node Info: leafs=") + String::num_int64((int64_t)numLeafNodes) + String(" avgFrames=") + String::num_int64((int64_t)avgFrames) + String(" zeroFrames=") + String::num_int64((int64_t)numZeroNodes) + String(" minFrames=") + String::num_int64((int64_t)minFrames) + String(" maxFrames=") + String::num_int64((int64_t)maxFrames));
}

void KdTree::FindNearestNeighbors(const std::vector<float> &frameFloats, std::vector<size_t> &resultFrameIndices) const {
	assert(IsInitialized() && !m_nodes.empty() && "Expecting a valid and initialized kdTree. Did you forget to call KdTree::Init()?");
	if (m_nodes.empty()) {
		resultFrameIndices.clear();
		return;
	}
	Node *curNode = m_nodes[0];

	Node *nodeToSearch = nullptr;
	const size_t numDimensions = frameFloats.size();
	for (size_t d = 0; d < numDimensions; ++d) {
		assert(curNode->m_dimension == d && "Dimension mismatch");

		if (curNode->m_leftNode && curNode->m_rightNode) {
			curNode = (frameFloats[d] <= curNode->m_median) ? curNode->m_leftNode : curNode->m_rightNode;
		} else if (!curNode->m_leftNode && !curNode->m_rightNode) {
			nodeToSearch = curNode;
		} else {

			if (curNode->m_leftNode) {
				if (frameFloats[d] <= curNode->m_median) {
					curNode = curNode->m_leftNode;
				} else {
					nodeToSearch = curNode;
				}
			} else {
				if (frameFloats[d] > curNode->m_median) {
					curNode = curNode->m_rightNode;
				} else {
					nodeToSearch = curNode;
				}
			}
		}

		if (nodeToSearch) {
			FindNearestNeighbors(nodeToSearch, frameFloats, resultFrameIndices);
			return;
		}
	}

	FindNearestNeighbors(curNode, frameFloats, resultFrameIndices);
}

void KdTree::FindNearestNeighbors([[maybe_unused]] Node *node, [[maybe_unused]] const std::vector<float> &frameFloats, std::vector<size_t> &resultFrameIndices) const {
	if (node == nullptr) {
		resultFrameIndices.clear();
		return;
	}
	resultFrameIndices = node->m_frames;
}

}
