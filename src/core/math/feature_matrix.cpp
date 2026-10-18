#include "core/math/feature_matrix.h"
#include "features/feature_schema.h"

#include <fstream>
#include <limits>

namespace godot::MotionMatching {

void FeatureMatrix::Clear() {
	resize(0, 0);
}

void FeatureMatrix::SaveAsCsv(const std::string &filename, const std::vector<std::string> &columnNames) const {
	std::ofstream file(filename.c_str());

	if (!columnNames.empty()) {
		for (size_t i = 0; i < columnNames.size(); ++i) {
			if (i != 0) {
				file << ",";
			}

			file << columnNames[i].c_str();
		}
		file << "\n";
	}

#ifdef GMM_USE_EIGEN

	const static Eigen::IOFormat csvFormat( 8, Eigen::DontAlignCols, ", ", "\n");
	file << format(csvFormat);
#else

	file.precision(std::numeric_limits<float>::max_digits10);
	for (size_t row = 0; row < rows(); ++row) {
		for (size_t column = 0; column < cols(); ++column) {
			if (column != 0) {
				file << ",";
			}
			file << coeff(row, column);
		}
		file << "\n";
	}
#endif
}

void FeatureMatrix::SaveAsCsv(const std::string &filename, const FeatureSchema *featureSchema) const {
	if (featureSchema == nullptr) {
		SaveAsCsv(filename, std::vector<std::string>{});
		return;
	}
	const std::vector<std::string> columnNames = featureSchema->CollectColumnNames();
	SaveAsCsv(filename, columnNames);
}

Vector2 FeatureMatrix::GetVector2(Index row, Index startColumn) const {
	return Vector2(
		coeff(row, startColumn + 0),
		coeff(row, startColumn + 1));
}

void FeatureMatrix::SetVector2(Index row, Index startColumn, const Vector2 &value) {
	operator()(row, startColumn + 0) = value.x;
	operator()(row, startColumn + 1) = value.y;
}

Vector3 FeatureMatrix::GetVector3(Index row, Index startColumn) const {
	return Vector3(
		coeff(row, startColumn + 0),
		coeff(row, startColumn + 1),
		coeff(row, startColumn + 2));
}

void FeatureMatrix::SetVector3(Index row, Index startColumn, const Vector3 &value) {
	operator()(row, startColumn + 0) = value.x;
	operator()(row, startColumn + 1) = value.y;
	operator()(row, startColumn + 2) = value.z;
}

size_t FeatureMatrix::CalcMemoryUsageInBytes() const {
	const size_t bytesPerValue = sizeof(GMM_MM_FLOATTYPE);
	const size_t numValues = size();
	return numValues * bytesPerValue;
}

}
