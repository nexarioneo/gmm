#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <godot_cpp/classes/file_access.hpp>
#include <godot_cpp/classes/ref.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include "TransformSpace.h"

class ActorInstance;
class Pose;

namespace godot::MotionMatching {
class FeatureSchema;
class QueryVector;

class CsvWriterBase {
public:
	virtual ~CsvWriterBase();

protected:
	bool OpenFile(const char *filename, FileAccess::ModeFlags openMode);
	virtual void End();
	bool IsReady() const { return m_file.is_valid(); }

	void WriteLine(std::string &line);
	void WriteVector3ToString(const Vector3 &vec, std::string &text);
	void WriteFloatArrayToString(const std::vector<float> &values, std::string &text);

protected:
	Ref<FileAccess> m_file;
	std::string m_tempBuffer;
};

class PoseWriterCsv : public CsvWriterBase {
public:
	virtual ~PoseWriterCsv() = default;

	struct WriteSettings {
		bool m_writePositions = true;
		bool m_writeRotations = true;
	};

	bool Begin(const char *filename, ActorInstance *actorInstance, const WriteSettings &writeSettings);
	void WritePose(Pose &pose, const ETransformSpace transformSpace);
	void End() override;

private:
	void SaveColumnNamesToString(std::string &outText);
	void SavePoseToString(Pose &pose, const ETransformSpace transformSpace, std::string &outText);

	ActorInstance *m_actorInstance = nullptr;
	WriteSettings m_settings;
};

class QueryVectorWriterCsv : public CsvWriterBase {
public:
	virtual ~QueryVectorWriterCsv() = default;

	bool Begin(const char *filename, const FeatureSchema *featureSchema);
	void Write(const QueryVector *queryVector);
};

class BestMatchingFrameWriterCsv : public CsvWriterBase {
public:
	virtual ~BestMatchingFrameWriterCsv() = default;

	bool Begin(const char *filename);
	void Write(size_t frame);
};

class PoseReaderCsv {
public:
	~PoseReaderCsv();

	struct ReadSettings {
		bool m_readPositions = true;
		bool m_readRotations = true;
	};

	bool Begin(const char *filename, const ReadSettings &readSettings);
	void ApplyPose(ActorInstance *actorInstance, Pose &pose, const ETransformSpace transformSpace, size_t index);
	size_t GetNumPoses() const { return m_poseValueLines.size(); }
	void End();

private:
	std::string m_columnNamesLine;
	std::vector<std::string> m_poseValueLines;
	ReadSettings m_settings;
};
}
