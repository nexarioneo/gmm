#include "MotionInstance.h"

#include "Actor.h"
#include "ActorInstance.h"
#include "Pose.h"

void MotionInstance::ExtractMotion(Transform &outDelta) {
	outDelta.Identity();
	if (m_motion == nullptr || m_actorInstance == nullptr) {
		return;
	}
	Actor *actor = m_actorInstance->GetActor();
	if (actor == nullptr) {
		return;
	}
	const size_t rootIndex = actor->GetMotionExtractionNodeIndex();
	Pose prevPose;
	Pose curPose;
	prevPose.LinkToActorInstance(m_actorInstance);
	curPose.LinkToActorInstance(m_actorInstance);
	MotionDataSampleSettings prevSettings;
	prevSettings.m_actorInstance = m_actorInstance;
	prevSettings.m_sampleTime = m_lastCurrentTime;
	m_motion->SamplePose(&prevPose, prevSettings);
	MotionDataSampleSettings curSettings;
	curSettings.m_actorInstance = m_actorInstance;
	curSettings.m_sampleTime = m_currentTime;
	m_motion->SamplePose(&curPose, curSettings);
	if (rootIndex >= prevPose.GetNumTransforms() || rootIndex >= curPose.GetNumTransforms()) {
		return;
	}
	const Transform prevRoot = prevPose.GetWorldSpaceTransform(rootIndex);
	const Transform curRoot = curPose.GetWorldSpaceTransform(rootIndex);
	const godot::Quaternion invPrevRot = prevRoot.m_rotation.inverse();
	outDelta.m_rotation = curRoot.m_rotation * invPrevRot;
	outDelta.m_position = curRoot.m_position - prevRoot.m_position;
}
