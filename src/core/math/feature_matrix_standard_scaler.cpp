#include "core/math/feature_matrix_standard_scaler.h"

#include <cassert>
#include <cmath>
#include <string>

#include <godot_cpp/classes/dir_access.hpp>
#include <godot_cpp/classes/file_access.hpp>
#include <godot_cpp/classes/ref.hpp>
#include <godot_cpp/variant/string.hpp>

namespace godot::MotionMatching {

bool StandardScaler::Fit(const FeatureMatrix &featureMatrix, [[maybe_unused]] const Settings &settings) {
	const FeatureMatrix::Index numRows = featureMatrix.rows();
	const FeatureMatrix::Index numColumns = featureMatrix.cols();

	m_means.clear();
	m_means.resize(numColumns);
	m_standardDeviations.clear();
	m_standardDeviations.resize(numColumns);

	if (numRows == 0) {
		return true;
	}

	for (FeatureMatrix::Index column = 0; column < numColumns; ++column) {

		float accumulated = 0.0f;
		for (FeatureMatrix::Index row = 0; row < numRows; ++row) {
			accumulated += featureMatrix(row, column);
		}
		const float mean = accumulated / numRows;
		m_means[column] = mean;

		float rss = 0.0f;
		for (FeatureMatrix::Index row = 0; row < numRows; ++row) {
			const float value = featureMatrix(row, column);
			rss += (value - mean) * (value - mean);
		}
		const float variance = rss / numRows;
		const float standardDeviation = sqrtf(variance);
		m_standardDeviations[column] = standardDeviation;
	}

	return true;
}

float StandardScaler::Transform(float value, FeatureMatrix::Index column) const {
	if (column >= m_means.size() || column >= m_standardDeviations.size()) {
		return value;
	}
	const float mean = m_means[column];
	float standardDeviation = m_standardDeviations[column];

	if (standardDeviation < s_epsilon) {
		standardDeviation = 1.0f;
	}

	return (value - mean) / standardDeviation;
}

Vector2 StandardScaler::Transform(const Vector2 &value, FeatureMatrix::Index column) const {
	return Vector2(Transform(value.x, column + 0), Transform(value.y, column + 1));
}

Vector3 StandardScaler::Transform(const Vector3 &value, FeatureMatrix::Index column) const {
	return Vector3(Transform(value.x, column + 0), Transform(value.y, column + 1), Transform(value.z, column + 2));
}

void StandardScaler::Transform(std::vector<float> &data) const {
	const size_t numValues = data.size();
	assert(numValues == m_means.size() && "Input data needs to have the same number of elements.");
	for (size_t i = 0; i < numValues; ++i) {
		data[i] = Transform(data[i], i);
	}
}

FeatureMatrix StandardScaler::Transform(const FeatureMatrix &featureMatrix) const {
	const FeatureMatrix::Index numRows = featureMatrix.rows();
	const FeatureMatrix::Index numColumns = featureMatrix.cols();
	FeatureMatrix result;
	result.resize(numRows, numColumns);

	for (FeatureMatrix::Index row = 0; row < numRows; ++row) {
		for (FeatureMatrix::Index column = 0; column < numColumns; ++column) {
			result(row, column) = Transform(featureMatrix(row, column), column);
		}
	}

	return result;
}

FeatureMatrix StandardScaler::InverseTransform(const FeatureMatrix &featureMatrix) const {
	const FeatureMatrix::Index numRows = featureMatrix.rows();
	const FeatureMatrix::Index numColumns = featureMatrix.cols();
	FeatureMatrix result;
	result.resize(numRows, numColumns);

	for (FeatureMatrix::Index row = 0; row < numRows; ++row) {
		for (FeatureMatrix::Index column = 0; column < numColumns; ++column) {
			result(row, column) = InverseTransform(featureMatrix(row, column), column);
		}
	}

	return result;
}

Vector2 StandardScaler::InverseTransform(const Vector2 &value, FeatureMatrix::Index column) const {
	return Vector2(InverseTransform(value.x, column + 0), InverseTransform(value.y, column + 1));
}

Vector3 StandardScaler::InverseTransform(const Vector3 &value, FeatureMatrix::Index column) const {
	return Vector3(
		InverseTransform(value.x, column + 0),
		InverseTransform(value.y, column + 1),
		InverseTransform(value.z, column + 2));
}

float StandardScaler::InverseTransform(float value, FeatureMatrix::Index column) const {
	if (column >= m_means.size() || column >= m_standardDeviations.size()) {
		return value;
	}
	float standardDeviation = m_standardDeviations[column];
	if (standardDeviation < s_epsilon) {
		standardDeviation = 1.0f;
	}

	return value * standardDeviation + m_means[column];
}

void StandardScaler::SaveAsCsv(const char *filename, const std::vector<std::string> &columnNames) {
	std::string data;

	if (!columnNames.empty()) {
		for (size_t i = 0; i < columnNames.size(); ++i) {
			if (i != 0) {
				data += ",";
			}

			data += columnNames[i].c_str();
		}
		data += "\n";
	}

	for (size_t i = 0; i < m_means.size(); ++i) {
		if (i != 0) {
			data += ",";
		}

		data += std::to_string(m_means[i]);
	}

	data += "\n";

	for (size_t i = 0; i < m_standardDeviations.size(); ++i) {
		if (i != 0) {
			data += ",";
		}

		data += std::to_string(m_standardDeviations[i]);
	}

	data += "\n";

	const String path(filename);
	DirAccess::make_dir_recursive_absolute(path.get_base_dir());
	Ref<FileAccess> file = FileAccess::open(path, FileAccess::WRITE);
	if (file.is_valid()) {
		file->store_string(String(data.c_str()));
		file->close();
	}
}

}
