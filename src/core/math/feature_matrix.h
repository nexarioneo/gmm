#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <godot_cpp/variant/vector2.hpp>
#include <godot_cpp/variant/vector3.hpp>

#define GMM_MM_FLOATTYPE float

#ifdef GMM_USE_EIGEN
#include <Eigen/Dense>
#endif

namespace godot::MotionMatching {
class FeatureSchema;

#ifdef GMM_USE_EIGEN

	using FeatureMatrixType = Eigen::Matrix<GMM_MM_FLOATTYPE, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
#else

	class FeatureMatrixType {
	public:
		size_t size() const {
			return m_data.size();
		}

		size_t rows() const {
			return m_rowCount;
		}

		size_t cols() const {
			return m_columnCount;
		}

		void resize(size_t rowCount, size_t columnCount) {
			m_rowCount = rowCount;
			m_columnCount = columnCount;
			m_data.resize(m_rowCount * m_columnCount);
		}

		float &operator()(size_t row, size_t column) {
			return m_data[row * m_columnCount + column];
		}

		const float &operator()(size_t row, size_t column) const {
			return m_data[row * m_columnCount + column];
		}

		float coeff(size_t row, size_t column) const {
			return m_data[row * m_columnCount + column];
		}

	private:
		std::vector<float> m_data;
		size_t m_rowCount = 0;
		size_t m_columnCount = 0;
	};
#endif

	class FeatureMatrix : public FeatureMatrixType {
public:
#ifdef GMM_USE_EIGEN
		using Index = Eigen::Index;
#else
		using Index = size_t;
#endif

		virtual ~FeatureMatrix() = default;

		void Clear();

		void SaveAsCsv(const std::string &filename, const std::vector<std::string> &columnNames = {}) const;
		void SaveAsCsv(const std::string &filename, const FeatureSchema *featureSchema) const;

		size_t CalcMemoryUsageInBytes() const;

		Vector2 GetVector2(Index row, Index startColumn) const;
		void SetVector2(Index row, Index startColumn, const Vector2 &value);

		Vector3 GetVector3(Index row, Index startColumn) const;
		void SetVector3(Index row, Index startColumn, const Vector3 &value);
	};
}