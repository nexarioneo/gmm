#include "core/math/feature_matrix_minmax_scaler.h"

#include <algorithm>
#include <cassert>
#include <limits>
#include <string>

#include <godot_cpp/classes/dir_access.hpp>
#include <godot_cpp/classes/file_access.hpp>
#include <godot_cpp/classes/ref.hpp>
#include <godot_cpp/variant/string.hpp>

namespace godot::MotionMatching {

bool MinMaxScaler::Fit(const FeatureMatrix &featureMatrix, const Settings &settings) {
	const FeatureMatrix::Index numRows = featureMatrix.rows();
	const FeatureMatrix::Index numColumns = featureMatrix.cols();

	m_clip = settings.m_clip;
	m_featureMin = settings.m_featureMin;
	m_featureMax = settings.m_featureMax;
	m_featureRange = settings.m_featureMax - settings.m_featureMin;
	assert(m_featureRange > s_epsilon && "Feature range too small. This will lead to divisions by zero.");
	m_dataMin.clear();
	m_dataMin.resize(numColumns, std::numeric_limits<float>::max());
	m_dataMax.clear();
	m_dataMax.resize(numColumns, std::numeric_limits<float>::lowest());

	for (FeatureMatrix::Index row = 0; row < numRows; ++row) {
		for (FeatureMatrix::Index column = 0; column < numColumns; ++column) {
			m_dataMin[column] = std::min(m_dataMin[column], featureMatrix(row, column));
			m_dataMax[column] = std::max(m_dataMax[column], featureMatrix(row, column));
		}
	}

	m_dataRange.clear();
	m_dataRange.resize(numColumns);
	for (FeatureMatrix::Index column = 0; column < numColumns; ++column) {
		const float columnMin = m_dataMin[column];
		const float columnMax = m_dataMax[column];
		const float range = columnMax - columnMin;
		m_dataRange[column] = range;
	}

	return true;
}

float MinMaxScaler::Transform(float value, FeatureMatrix::Index column) const {
	if (column >= m_dataMin.size() || column >= m_dataRange.size()) {
		return value;
	}
	const float &min = m_dataMin[column];
	const float &range = m_dataRange[column];

	float result = value;
	if (range > s_epsilon) {
		const float normalizedValue = (value - min) / (range);
		const float scaled = normalizedValue * m_featureRange + m_featureMin;

		result = scaled;
	}

	if (m_clip) {
		result = std::clamp(result, m_featureMin, m_featureMax);
	}

	return result;
}

Vector2 MinMaxScaler::Transform(const Vector2 &value, FeatureMatrix::Index column) const {
	return Vector2(
		Transform(value.x, column + 0),
		Transform(value.y, column + 1));
}

Vector3 MinMaxScaler::Transform(const Vector3 &value, FeatureMatrix::Index column) const {
	return Vector3(
		Transform(value.x, column + 0),
		Transform(value.y, column + 1),
		Transform(value.z, column + 2));
}

void MinMaxScaler::Transform(std::vector<float> &data) const {
	const size_t numValues = data.size();
	assert(numValues == m_dataMin.size() && "Input data needs to have the same number of elements.");
	for (size_t i = 0; i < numValues; ++i) {
		data[i] = Transform(data[i], i);
	}
}

FeatureMatrix MinMaxScaler::Transform(const FeatureMatrix &featureMatrix) const {
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

FeatureMatrix MinMaxScaler::InverseTransform(const FeatureMatrix &featureMatrix) const {
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

Vector2 MinMaxScaler::InverseTransform(const Vector2 &value, FeatureMatrix::Index column) const {
	return Vector2(
		InverseTransform(value.x, column + 0),
		InverseTransform(value.y, column + 1));
}

Vector3 MinMaxScaler::InverseTransform(const Vector3 &value, FeatureMatrix::Index column) const {
	return Vector3(
		InverseTransform(value.x, column + 0),
		InverseTransform(value.y, column + 1),
		InverseTransform(value.z, column + 2));
}

float MinMaxScaler::InverseTransform(float value, FeatureMatrix::Index column) const {
	if (m_featureRange <= s_epsilon || column >= m_dataRange.size() || column >= m_dataMin.size()) {
		return value;
	}
	const float normalizedValue = (value - m_featureMin) / m_featureRange;
	return normalizedValue * m_dataRange[column] + m_dataMin[column];
}

void MinMaxScaler::SaveMinMaxAsCsv(const char *filename, const std::vector<std::string> &columnNames) {
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

	for (size_t i = 0; i < m_dataMin.size(); ++i) {
		if (i != 0) {
			data += ",";
		}

		data += std::to_string(m_dataMin[i]);
	}

	data += "\n";

	for (size_t i = 0; i < m_dataMax.size(); ++i) {
		if (i != 0) {
			data += ",";
		}

		data += std::to_string(m_dataMax[i]);
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
