#pragma once

#include <vector>

#include <godot_cpp/variant/vector2.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include "FeatureMatrix.h"

namespace godot::MotionMatching {

class FeatureMatrixTransformer {
public:
	virtual ~FeatureMatrixTransformer() = default;

	struct Settings {
		float m_featureMin = -1.0f;
		float m_featureMax = 1.0f;

		bool m_clip = false;
	};

	virtual bool Fit(const FeatureMatrix &featureMatrix, const Settings &settings) = 0;

	virtual float Transform(float value, FeatureMatrix::Index column) const = 0;
	virtual Vector2 Transform(const Vector2 &value, FeatureMatrix::Index column) const = 0;
	virtual Vector3 Transform(const Vector3 &value, FeatureMatrix::Index column) const = 0;
	virtual void Transform(std::vector<float> &data) const = 0;
	virtual FeatureMatrix Transform(const FeatureMatrix &in) const = 0;

	virtual float InverseTransform(float value, FeatureMatrix::Index column) const = 0;
	virtual Vector2 InverseTransform(const Vector2 &value, FeatureMatrix::Index column) const = 0;
	virtual Vector3 InverseTransform(const Vector3 &value, FeatureMatrix::Index column) const = 0;
	virtual FeatureMatrix InverseTransform(const FeatureMatrix &in) const = 0;
};
}
