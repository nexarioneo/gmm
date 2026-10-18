#pragma once

#include <limits>
#include <string>
#include <vector>

#include <godot_cpp/variant/vector2.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include "core/math/feature_matrix.h"
#include "core/math/feature_matrix_transformer.h"

namespace godot::MotionMatching {

class MinMaxScaler : public FeatureMatrixTransformer {
public:
	MinMaxScaler() = default;

	bool Fit(const FeatureMatrix &featureMatrix, const Settings &settings = {}) override;

	float Transform(float value, FeatureMatrix::Index column) const override;
	Vector2 Transform(const Vector2 &value, FeatureMatrix::Index column) const override;
	Vector3 Transform(const Vector3 &value, FeatureMatrix::Index column) const override;
	void Transform(std::vector<float> &data) const override;
	FeatureMatrix Transform(const FeatureMatrix &featureMatrix) const override;

	FeatureMatrix InverseTransform(const FeatureMatrix &featureMatrix) const override;
	Vector2 InverseTransform(const Vector2 &value, FeatureMatrix::Index column) const override;
	Vector3 InverseTransform(const Vector3 &value, FeatureMatrix::Index column) const override;
	float InverseTransform(float value, FeatureMatrix::Index column) const override;

	const std::vector<float> &GetMin() const { return m_dataMin; }
	const std::vector<float> &GetMax() const { return m_dataMax; }

	static constexpr float s_epsilon = std::numeric_limits<float>::epsilon();

	void SaveMinMaxAsCsv(const char *filename, const std::vector<std::string> &columnNames = {});

private:
	std::vector<float> m_dataMin;
	std::vector<float> m_dataMax;
	std::vector<float> m_dataRange;

	bool m_clip = false;

	float m_featureMin = 0.0f;
	float m_featureMax = 1.0f;
	float m_featureRange = 1.0f;
};
}