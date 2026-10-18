#include "runtime/csv_serializers.h"

#include <cstdint>
#include <cstdlib>
#include <string>

#include <godot_cpp/classes/dir_access.hpp>
#include <godot_cpp/classes/file_access.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/basis.hpp>
#include <godot_cpp/variant/packed_string_array.hpp>
#include <godot_cpp/variant/quaternion.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/utility_functions.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include "actor/actor.h"
#include "actor/actor_instance.h"
#include "core/types/skeleton_node.h"
#include "pose/pose.h"
#include "core/types/skeleton.h"
#include "core/types/transform.h"
#include "pose/transform_data.h"
#include "features/feature_schema.h"
#include "core/math/query_vector.h"

namespace godot::MotionMatching {

static std::vector<std::string> TokenizeToStdStrings(const std::string &text, const String &delimiter) {
	const PackedStringArray parts = String(text.c_str()).split(delimiter, true);
	std::vector<std::string> tokens;
	tokens.reserve(parts.size());
	for (int i = 0; i < parts.size(); ++i) {
		String token = parts[i];
		if (token.ends_with("\r")) {
			token = token.substr(0, token.length() - 1);
		}
		tokens.emplace_back(token.utf8().get_data());
	}
	return tokens;
}

static std::string ToStdString(const String &text) {
	return std::string(text.utf8().get_data());
}

CsvWriterBase::~CsvWriterBase() {
	End();
}

bool CsvWriterBase::OpenFile(const char *filename, FileAccess::ModeFlags openMode) {
	if (m_file.is_valid()) {
		m_file->close();
		m_file.unref();
	}
	if (filename == nullptr || filename[0] == '\0') {
		return false;
	}

	const String path(filename);
	DirAccess::make_dir_recursive_absolute(path.get_base_dir());
	m_file = FileAccess::open(path, openMode);
	if (m_file.is_null()) {
		End();
		return false;
	}

	return true;
}

void CsvWriterBase::End() {
	if (m_file.is_valid()) {
		m_file->close();
		m_file.unref();
	}
	m_tempBuffer.clear();
}

void CsvWriterBase::WriteLine(std::string &line) {

	while (!line.empty() && line.back() == ',') {
		line.pop_back();
	}
	line += "\n";
	if (m_file.is_valid()) {
		m_file->store_string(String(line.c_str()));
	}
}

void CsvWriterBase::WriteVector3ToString(const Vector3 &vec, std::string &text) {

	text += ToStdString(String::num(vec.x, 8) + String(",") + String::num(vec.y, 8) + String(",") + String::num(vec.z, 8) + String(","));
}

void CsvWriterBase::WriteFloatArrayToString(const std::vector<float> &values, std::string &text) {

	text.reserve(text.size() + values.size() * 10);
	for (float value : values) {
		text += ToStdString(String::num(value, 8) + String(","));
	}
}

bool PoseWriterCsv::Begin(const char *filename, ActorInstance *actorInstance, const WriteSettings &writeSettings) {
	m_settings = writeSettings;

	if (!actorInstance) {
		return false;
	}

	if (!CsvWriterBase::OpenFile(filename, FileAccess::WRITE)) {
		return false;
	}

	m_actorInstance = actorInstance;

	SaveColumnNamesToString(m_tempBuffer);
	return true;
}

void PoseWriterCsv::SaveColumnNamesToString(std::string &outText) {
	if (!m_actorInstance || !m_actorInstance->GetActor()) {
		outText.clear();
		WriteLine(m_tempBuffer);
		return;
	}
	const Skeleton *skeleton = m_actorInstance->GetActor()->GetSkeleton();
	if (!skeleton) {
		outText.clear();
		WriteLine(m_tempBuffer);
		return;
	}
	const size_t numEnabledJoints = m_actorInstance->GetNumEnabledNodes();
	outText.clear();
	outText.reserve(50 * numEnabledJoints);

	auto SaveVector3ColumnNames = [](std::string &text, const char *jointName, const char *vecName) {
		text += ToStdString(String(jointName) + String(".") + String(vecName) + String(".X,") + String(jointName) + String(".") + String(vecName) + String(".Y,") + String(jointName) + String(".") + String(vecName) + String(".Z,"));
	};

	for (size_t i = 0; i < numEnabledJoints; ++i) {
		const size_t jointIndex = m_actorInstance->GetEnabledNode(i);
		const MmNode *joint = skeleton->GetNode(jointIndex);
		if (!joint) {
			continue;
		}

		if (m_settings.m_writePositions) {
			SaveVector3ColumnNames(outText, joint->GetName(), "Pos");
		}

		if (m_settings.m_writeRotations) {
			SaveVector3ColumnNames(outText, joint->GetName(), "RotBasisX");
			SaveVector3ColumnNames(outText, joint->GetName(), "RotBasisY");
		}
	}

	WriteLine(m_tempBuffer);
}

void PoseWriterCsv::WritePose(Pose &pose, const ETransformSpace transformSpace) {
	if (!IsReady() || !m_actorInstance) {
		return;
	}

	SavePoseToString(pose, transformSpace, m_tempBuffer);
	WriteLine(m_tempBuffer);
}

void PoseWriterCsv::SavePoseToString(Pose &pose, const ETransformSpace transformSpace, std::string &outText) {
	const size_t numEnabledJoints = m_actorInstance->GetNumEnabledNodes();
	outText.clear();
	outText.reserve(10 * 3 * 3 * numEnabledJoints);

	for (size_t i = 0; i < numEnabledJoints; ++i) {
		const size_t jointIndex = m_actorInstance->GetEnabledNode(i);

		Transform transform = Transform::CreateIdentity();
		switch (transformSpace) {
			case TRANSFORM_SPACE_LOCAL: {
				transform = pose.GetLocalSpaceTransform(jointIndex);
				break;
			}
			case TRANSFORM_SPACE_MODEL: {
				transform = pose.GetModelSpaceTransform(jointIndex);
				break;
			}
			default: {
				UtilityFunctions::push_error("Motion Matching: Unsupported transform space");
				break;
			}
		}

		if (m_settings.m_writePositions) {
			const Vector3 position = transform.m_position;
			WriteVector3ToString(position, outText);
		}

		if (m_settings.m_writeRotations) {
			const Quaternion rotation = transform.m_rotation;

			const Basis rotationMatrix = Basis(rotation);
			WriteVector3ToString(rotationMatrix.get_column(Vector3::AXIS_X).normalized(), outText);
			WriteVector3ToString(rotationMatrix.get_column(Vector3::AXIS_Y).normalized(), outText);
		}
	}
}

void PoseWriterCsv::End() {
	m_actorInstance = nullptr;
	CsvWriterBase::End();
}

bool QueryVectorWriterCsv::Begin(const char *filename, const FeatureSchema *featureSchema) {
	if (!featureSchema) {
		return false;
	}

	if (!CsvWriterBase::OpenFile(filename, FileAccess::WRITE)) {
		return false;
	}

	const std::vector<std::string> columnNames = featureSchema->CollectColumnNames();
	m_tempBuffer.clear();
	if (!columnNames.empty()) {
		for (size_t i = 0; i < columnNames.size(); ++i) {
			if (i != 0) {
				m_tempBuffer += ",";
			}

			m_tempBuffer += columnNames[i];
		}
	}
	WriteLine(m_tempBuffer);

	return true;
}

void QueryVectorWriterCsv::Write(const QueryVector *queryVector) {
	if (!IsReady() || !queryVector) {
		return;
	}

	m_tempBuffer.clear();
	WriteFloatArrayToString(queryVector->GetData(), m_tempBuffer);
	WriteLine(m_tempBuffer);
}

bool BestMatchingFrameWriterCsv::Begin(const char *filename) {
	if (!CsvWriterBase::OpenFile(filename, FileAccess::WRITE)) {
		return false;
	}

	m_tempBuffer = "Best Matching Frames";
	WriteLine(m_tempBuffer);
	return true;
}

void BestMatchingFrameWriterCsv::Write(size_t frame) {
	m_tempBuffer = std::to_string(frame);
	WriteLine(m_tempBuffer);
}

PoseReaderCsv::~PoseReaderCsv() {
	End();
}

bool PoseReaderCsv::Begin(const char *filename, const ReadSettings &readSettings) {
	m_settings = readSettings;

	const String path(filename);
	Ref<FileAccess> file = FileAccess::open(path, FileAccess::READ);
	if (file.is_null()) {
		return false;
	}

	const uint64_t fileSize = file->get_length();
	std::string textBuffer;
	textBuffer.resize(static_cast<size_t>(fileSize));
	if (fileSize > 0) {
		file->get_buffer(reinterpret_cast<uint8_t *>(textBuffer.data()), static_cast<uint64_t>(fileSize));
	}
	file->close();

	std::vector<std::string> lines = TokenizeToStdStrings(textBuffer, "\n");
	if (lines.empty()) {
		return false;
	}

	m_columnNamesLine = lines[0];
	lines.erase(lines.begin());

	while (!lines.empty() && lines.back().empty()) {
		lines.pop_back();
	}
	m_poseValueLines = lines;

	return true;
}

void PoseReaderCsv::ApplyPose(ActorInstance *actorInstance, Pose &pose, const ETransformSpace transformSpace, size_t index) {
	if (!actorInstance || index >= m_poseValueLines.size()) {
		UtilityFunctions::push_error("Motion Matching: ApplyPose called with null actor or out-of-range pose index.");
		return;
	}
	if (!actorInstance->GetTransformData() || !actorInstance->GetTransformData()->GetBindPose()) {
		UtilityFunctions::push_error("Motion Matching: ApplyPose called without bind pose.");
		return;
	}
	const std::vector<std::string> valueTokens = TokenizeToStdStrings(m_poseValueLines[index], ",");

	Pose *bindPose = actorInstance->GetTransformData()->GetBindPose();

	size_t valueIndex = 0;
	const size_t numEnabledJoints = actorInstance->GetNumEnabledNodes();
	for (size_t i = 0; i < numEnabledJoints; ++i) {
		const size_t jointIndex = actorInstance->GetEnabledNode(i);

		Transform transform = bindPose->GetLocalSpaceTransform(jointIndex);
		switch (transformSpace) {
			case TRANSFORM_SPACE_LOCAL: {
				transform = pose.GetLocalSpaceTransform(jointIndex);
				break;
			}
			case TRANSFORM_SPACE_MODEL: {
				transform = pose.GetModelSpaceTransform(jointIndex);
				break;
			}
			default: {
				UtilityFunctions::push_error("Motion Matching: Unsupported transform space");
				break;
			}
		}

		auto LoadVector3FromString = [&valueTokens](size_t &valueIndex, Vector3 &outVec) {

			auto SafeStof = [](const std::string &s) -> float {
				if (s.empty()) {
					return 0.0f;
				}
				char *end = nullptr;
				const float v = std::strtof(s.c_str(), &end);
				return (end != nullptr && *end == '\0') ? v : 0.0f;
			};
			if (valueIndex + 2 >= valueTokens.size()) {
				outVec = Vector3();
				valueIndex += 3;
				return;
			}
			outVec.x = SafeStof(valueTokens[valueIndex + 0]);
			outVec.y = SafeStof(valueTokens[valueIndex + 1]);
			outVec.z = SafeStof(valueTokens[valueIndex + 2]);
			valueIndex += 3;
		};

		if (m_settings.m_readPositions) {
			LoadVector3FromString(valueIndex, transform.m_position);
		}

		if (m_settings.m_readRotations) {

			Vector3 basisX(0.0f, 0.0f, 0.0f);
			Vector3 basisY(0.0f, 0.0f, 0.0f);
			LoadVector3FromString(valueIndex, basisX);
			LoadVector3FromString(valueIndex, basisY);
			basisX = basisX.normalized();
			basisY = basisY.normalized();

			Basis rotationMatrix;
			rotationMatrix.set_column(Vector3::AXIS_X, basisX);
			rotationMatrix.set_column(Vector3::AXIS_Y, basisY);
			rotationMatrix.set_column(Vector3::AXIS_Z, basisX.cross(basisY));

			transform.m_rotation = rotationMatrix.get_quaternion();
		}

		switch (transformSpace) {
			case TRANSFORM_SPACE_LOCAL: {
				pose.SetLocalSpaceTransform(jointIndex, transform);
				break;
			}
			case TRANSFORM_SPACE_MODEL: {
				pose.SetModelSpaceTransform(jointIndex, transform);
				break;
			}
			default: {
				UtilityFunctions::push_error("Motion Matching: Unsupported transform space");
				break;
			}
		}
	}
}

void PoseReaderCsv::End() {
	m_columnNamesLine.clear();
	m_poseValueLines.clear();
}
}
