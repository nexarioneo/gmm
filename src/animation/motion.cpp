#include "animation/motion.h"

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>

#include <godot_cpp/classes/animation.hpp>
#include <godot_cpp/classes/ref.hpp>
#include <godot_cpp/classes/resource.hpp>
#include <godot_cpp/core/memory.hpp>
#include <godot_cpp/variant/node_path.hpp>
#include <godot_cpp/variant/string.hpp>

#include "runtime/event_data.h"
#include "actor/actor.h"
#include "actor/actor_instance.h"
#include "animation/motion_instance.h"
#include "pose/pose.h"
#include "core/types/skeleton.h"

namespace {

int32_t FindBoneInMap(const std::unordered_map<std::string, int32_t> &boneIndexByName, const godot::String &name) {
	const auto utf8Chars = name.utf8();
	const char *cstr = utf8Chars.get_data();
	if (!cstr || !*cstr) {
		return -1;
	}
	const auto it = boneIndexByName.find(cstr);
	return it != boneIndexByName.end() ? it->second : -1;
}

int32_t ResolveTrackBone(const godot::Animation *animation, int32_t trackIndex,
		const std::unordered_map<std::string, int32_t> &boneIndexByName) {
	const godot::NodePath path = animation->track_get_path(trackIndex);
	const int64_t subNameCount = path.get_subname_count();
	for (int64_t s = 0; s < subNameCount; ++s) {
		const int32_t boneIndex = FindBoneInMap(boneIndexByName, godot::String(path.get_subname(s)));
		if (boneIndex >= 0) {
			return boneIndex;
		}
	}
	const int64_t nameCount = path.get_name_count();
	for (int64_t n = 0; n < nameCount; ++n) {
		const int32_t boneIndex = FindBoneInMap(boneIndexByName, godot::String(path.get_name(n)));
		if (boneIndex >= 0) {
			return boneIndex;
		}
	}
	return -1;
}

}

float Motion::GetDuration() const {
	if (m_animation == nullptr) {
		return 0.0f;
	}

	const double length = m_animation->get_length();
	return length > 0.0 ? static_cast<float>(length) : 0.0f;
}

void Motion::SamplePose(Pose *outputPose, MotionDataSampleSettings &settings) const {
	SamplePoseInternal(outputPose, settings);
}

void Motion::SamplePose(Pose *outputPose, const MotionDataSampleSettings &settings) const {
	SamplePoseInternal(outputPose, settings);
}

void Motion::Update(const Pose *inputPose, Pose *outputPose, MotionInstance *motionInstance) const {
	if (outputPose == nullptr || motionInstance == nullptr) {
		return;
	}
	MotionDataSampleSettings settings;
	const ActorInstance *instanceActor = motionInstance->GetActorInstance();
	settings.m_actorInstance = instanceActor != nullptr ? instanceActor : outputPose->GetActorInstance();
	settings.m_inputPose = inputPose;
	settings.m_inPlace = false;

	settings.m_mirror = motionInstance->GetMirrorMotion();
	settings.m_retarget = false;
	float sampleTime = motionInstance->GetCurrentTime();
	const float duration = GetDuration();
	if (sampleTime < 0.0f) {
		sampleTime = 0.0f;
	} else if (sampleTime > duration) {
		sampleTime = duration;
	}
	settings.m_sampleTime = sampleTime;
	SamplePoseInternal(outputPose, settings);
}

void Motion::SamplePoseInternal(Pose *outputPose, const MotionDataSampleSettings &settings) const {
	if (outputPose == nullptr || m_animation == nullptr) {
		return;
	}

	const ActorInstance *actorInstance = settings.m_actorInstance;
	if (actorInstance == nullptr) {
		actorInstance = outputPose->GetActorInstance();
	}
	if (actorInstance == nullptr) {
		return;
	}
	const Actor *actor = actorInstance->GetActor();
	if (actor == nullptr) {
		return;
	}
	const Skeleton *skeleton = actor->GetSkeleton();
	godot::Skeleton3D *linkedSkeleton = skeleton != nullptr ? skeleton->GetLinkedSkeleton() : nullptr;
	if (linkedSkeleton == nullptr) {
		return;
	}
	const size_t numJoints = actorInstance->GetNumNodes();
	if (numJoints == 0) {
		return;
	}
	if (outputPose->GetNumTransforms() < numJoints) {
		outputPose->ResizeTransforms(numJoints);
	}

	const Pose *bindPose = settings.m_inputPose;
	if (bindPose != nullptr) {
		size_t copyCount = bindPose->GetNumTransforms();
		if (copyCount > numJoints) {
			copyCount = numJoints;
		}
		for (size_t i = 0; i < copyCount; ++i) {
			outputPose->SetLocalSpaceTransform(i, bindPose->GetLocalSpaceTransform(i));
		}
	}

	std::unordered_map<std::string, int32_t> boneIndexByName;
	const int32_t boneCount = linkedSkeleton->get_bone_count();
	for (int32_t b = 0; b < boneCount; ++b) {
		const auto utf8Chars = linkedSkeleton->get_bone_name(b).utf8();
		const char *cstr = utf8Chars.get_data();
		if (cstr && *cstr) {
			boneIndexByName[cstr] = b;
		}
	}

	double sampleTime = static_cast<double>(settings.m_sampleTime);
	if (sampleTime < 0.0) {
		sampleTime = 0.0;
	}

	std::unordered_map<size_t, Transform> pending;
	const int32_t trackCount = m_animation->get_track_count();
	pending.reserve(trackCount > 0 ? static_cast<size_t>(trackCount) : 0);
	for (int32_t t = 0; t < trackCount; ++t) {
		const godot::Animation::TrackType trackType = m_animation->track_get_type(t);
		if (trackType != godot::Animation::TYPE_POSITION_3D &&
				trackType != godot::Animation::TYPE_ROTATION_3D) {

			continue;
		}
		if (m_animation->track_get_key_count(t) <= 0) {
			continue;
		}
		const int32_t boneIndex = ResolveTrackBone(m_animation, t, boneIndexByName);
		if (boneIndex < 0 || static_cast<size_t>(boneIndex) >= numJoints) {
			continue;
		}
		const size_t jointIndex = static_cast<size_t>(boneIndex);
		auto it = pending.find(jointIndex);
		if (it == pending.end()) {
			it = pending.emplace(jointIndex, outputPose->GetLocalSpaceTransform(jointIndex)).first;
		}
		if (trackType == godot::Animation::TYPE_POSITION_3D) {
			it->second.m_position = m_animation->position_track_interpolate(t, sampleTime);
		} else {
			it->second.m_rotation = m_animation->rotation_track_interpolate(t, sampleTime);
		}
	}

	if (settings.m_inPlace) {
		const size_t rootIndex = actor->GetMotionExtractionNodeIndex();
		if (rootIndex < numJoints) {
			godot::Vector3 bindPosition;
			if (bindPose != nullptr && rootIndex < bindPose->GetNumTransforms()) {
				bindPosition = bindPose->GetLocalSpaceTransform(rootIndex).m_position;
			}
			auto it = pending.find(rootIndex);
			if (it != pending.end()) {
				it->second.m_position = bindPosition;
			} else {
				Transform rooted = outputPose->GetLocalSpaceTransform(rootIndex);
				rooted.m_position = bindPosition;
				pending.emplace(rootIndex, rooted);
			}
		}
	}

	for (const auto &entry : pending) {
		Transform jointTransform = entry.second;
		if (settings.m_mirror) {
			jointTransform.m_position.x = -jointTransform.m_position.x;
			const godot::Quaternion mirroredRot(
				jointTransform.m_rotation.x,
				-jointTransform.m_rotation.y,
				-jointTransform.m_rotation.z,
				jointTransform.m_rotation.w);
			jointTransform.m_rotation = mirroredRot.normalized();
		}
		outputPose->SetLocalSpaceTransform(entry.first, jointTransform);
	}
}

void Motion::LinkAnimation(godot::Animation *animation) {
	m_animation = animation;
	m_eventTableDirty = true;
}

void Motion::RebuildEventTable() const {
	if (!m_eventTableDirty) {
		return;
	}
	m_eventTableDirty = false;
	m_eventTable.Clear();
	if (m_animation == nullptr) {
		return;
	}

	const int32_t trackCount = m_animation->get_track_count();
	for (int32_t t = 0; t < trackCount; ++t) {
		const godot::Animation::TrackType trackType = m_animation->track_get_type(t);
		if (trackType != godot::Animation::TYPE_METHOD && trackType != godot::Animation::TYPE_AUDIO) {
			continue;
		}
		const bool isAudio = (trackType == godot::Animation::TYPE_AUDIO);
		MotionEventTrack *track = m_eventTable.AddTrack();
		if (track == nullptr) {
			continue;
		}
		const int32_t keyCount = m_animation->track_get_key_count(t);
		for (int32_t k = 0; k < keyCount; ++k) {
			MotionEvent event;
			event.SetIsTickEvent(true);
			const float keyTime = static_cast<float>(m_animation->track_get_key_time(t, k));
			event.SetStartTime(keyTime);
			event.SetEndTime(keyTime);
			auto tag = memnew(godot::MotionMatching::TagEventData);
			if (isAudio) {

				godot::Ref<godot::Resource> stream = m_animation->audio_track_get_key_stream(t, k);
				godot::String streamName = stream.is_valid() ? stream->get_path().get_file() : godot::String("audio");
				if (streamName.is_empty()) {
					streamName = stream.is_valid() ? stream->get_name() : godot::String("audio");
				}
				tag->set_tag(godot::String("audio:") + streamName);
			} else {
				tag->set_tag(m_animation->method_track_get_name(t, k));
			}

			event.AddEventData(std::shared_ptr<godot::MotionMatching::EventData>(
				tag.ptr(), [tag](godot::MotionMatching::EventData *) mutable {}));
			track->AddEvent(event);
		}
	}
}
