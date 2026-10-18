#pragma once

#include <limits>
#include <string>
#include <vector>

#include <godot_cpp/variant/vector2.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include "core/math/feature_matrix.h"
#include "core/math/feature_matrix_transformer.h"

namespace godot::MotionMatching {

class StandardScaler : public FeatureMatrixTransformer {
public:
	StandardScaler() = default;

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

	const std::vector<float> &GetMeans() const {
		return m_means;
	}
	const std::vector<float> &GetStandardDeviations() const {
		return m_standardDeviations;
	}

	static constexpr float s_epsilon = std::numeric_limits<float>::epsilon();

	void SaveAsCsv(const char *filename, const std::vector<std::string> &columnNames = {});

private:
	std::vector<float> m_means;
	std::vector<float> m_standardDeviations;
};
}