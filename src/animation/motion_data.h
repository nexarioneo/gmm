#pragma once

class ActorInstance;
class Pose;

struct MotionDataSampleSettings {
	const ActorInstance *m_actorInstance = nullptr;
	const Pose *m_inputPose = nullptr;
	float m_sampleTime = 0.0f;

	bool m_inPlace = false;
	bool m_mirror = false;
	bool m_retarget = false;
};