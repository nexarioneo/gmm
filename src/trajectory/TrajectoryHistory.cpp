#include "TrajectoryHistory.h"

#include <cassert>
#include <limits>

#include "Transform.h"

namespace godot::MotionMatching {

Sample operator*(Sample sample, float weight) {
	return { sample.m_position * weight, sample.m_facingDirection * weight };
}

Sample operator*(float weight, Sample sample) {
	return { weight * sample.m_position, weight * sample.m_facingDirection };
}

Sample operator-(Sample lhs, const Sample &rhs) {
	return { lhs.m_position - rhs.m_position, lhs.m_facingDirection - rhs.m_facingDirection };
}

Sample operator+(Sample lhs, const Sample &rhs) {
	return { lhs.m_position + rhs.m_position, lhs.m_facingDirection + rhs.m_facingDirection };
}

void TrajectoryHistory::Init(const Pose &pose, size_t jointIndex, const Vector3 &facingAxisDir, float numSecondsToTrack) {
	assert(numSecondsToTrack > 0.0f && "Number of seconds to track has to be greater than zero.");
	Clear();
	m_jointIndex = jointIndex;
	m_facingAxisDir = facingAxisDir;
	m_numSecondsToTrack = numSecondsToTrack;

	PrefillSamples(pose, 1.0f / 60.0f);
}

void TrajectoryHistory::AddSample(const Pose &pose) {
	Sample sample;
	const Transform worldSpaceTransform = pose.GetWorldSpaceTransform(m_jointIndex);
	sample.m_position = worldSpaceTransform.m_position;

	sample.m_facingDirection = NormalizedSafe(worldSpaceTransform.TransformVector(m_facingAxisDir));

	AddKey(m_currentTime, sample);

	while (GetNumKeys() > 2 &&
		((GetKey(GetNumKeys() - 2)->m_time - GetFirstTime()) > m_numSecondsToTrack)) {
		RemoveFirstKey();
	}
}

void TrajectoryHistory::PrefillSamples(const Pose &pose, float timeDelta) {
	const size_t numKeyframes = static_cast<size_t>(m_numSecondsToTrack / timeDelta);
	for (size_t i = 0; i < numKeyframes; ++i) {
		AddSample(pose);
		Update(timeDelta);
	}
}

void TrajectoryHistory::Clear() {
	m_jointIndex = 0;
	m_currentTime = 0.0f;
	ClearKeys();
}

void TrajectoryHistory::Update(float timeDelta) {
	m_currentTime += timeDelta;
}

Sample TrajectoryHistory::Evaluate(float time) const {
	if (GetNumKeys() == 0) {
		return {};
	}

	return GetValueAtTime(GetLastTime() - time);
}

Sample TrajectoryHistory::EvaluateNormalized(float normalizedTime) const {

	if (m_keys.empty()) {
		return {};
	}
	const float firstTime = GetFirstTime();
	const float lastTime = GetLastTime();
	const float range = lastTime - firstTime;

	const float time = (1.0f - normalizedTime) * range + firstTime;
	return GetValueAtTime(time);
}

void TrajectoryHistory::DebugDraw(DebugDrawOutput &output, const Color &color, float timeStart) const {
	const size_t numKeyframes = GetNumKeys();
	if (numKeyframes == 0) {
		return;
	}

	const float adjustedLastTime = GetLastTime() - timeStart;
	size_t adjustedLastKey = FindKeyNumber(adjustedLastTime);
	if (adjustedLastKey == InvalidIndex) {
		adjustedLastKey = GetNumKeys() - 1;
	}
	const float firstTime = GetFirstTime();
	const float range = adjustedLastTime - firstTime;
	if (range <= std::numeric_limits<float>::epsilon()) {
		return;
	}

	for (size_t i = 0; i < adjustedLastKey; ++i) {
		const float time = GetKey(i)->m_time;
		const float normalized = (time - firstTime) / range;
		if (normalized < 0.3f) {
			continue;
		}

		Color finalColor = color;
		finalColor.a = finalColor.a * 0.6f * normalized;
		const float markerSize = m_debugMarkerSize * 0.7f * normalized;

		const Sample currentSample = GetKey(i)->m_value;
		output.ball_centers.push_back(currentSample.m_position);
		output.ball_sizes.push_back(markerSize);
		output.ball_colors.push_back(finalColor);

		const float facingDirectionLength = m_debugMarkerSize * 10.0f * normalized;
		output.line_points.push_back(currentSample.m_position);
		output.line_points.push_back(currentSample.m_position + currentSample.m_facingDirection * facingDirectionLength);
		output.line_colors.push_back(finalColor);
		output.line_colors.push_back(finalColor);
	}
}

void TrajectoryHistory::DebugDrawSampled(DebugDrawOutput &output, size_t numSamples, const Color &color) const {
	if (m_keys.empty() || numSamples == 0) {
		return;
	}
	Sample lastSample = EvaluateNormalized(0.0f);
	for (size_t i = 0; i < numSamples; ++i) {
		const float sampleTime = (numSamples > 1) ? i / static_cast<float>(numSamples - 1) : 0.0f;
		const Sample currentSample = EvaluateNormalized(sampleTime);
		if (i > 0) {
			output.line_points.push_back(lastSample.m_position);
			output.line_points.push_back(currentSample.m_position);
			output.line_colors.push_back(color);
			output.line_colors.push_back(color);
		}

		output.ball_centers.push_back(currentSample.m_position);
		output.ball_sizes.push_back(m_debugMarkerSize);
		output.ball_colors.push_back(color);

		lastSample = currentSample;
	}
}

void TrajectoryHistory::AddKey(float time, const Sample &value) {
	m_keys.push_back(Key{ time, value });
}

void TrajectoryHistory::RemoveFirstKey() {
	assert(!m_keys.empty() && "Cannot remove the first key from an empty history!");
	m_keys.erase(m_keys.begin());
}

void TrajectoryHistory::ClearKeys() {
	m_keys.clear();
}

size_t TrajectoryHistory::GetNumKeys() const {
	return m_keys.size();
}

const TrajectoryHistory::Key *TrajectoryHistory::GetKey(size_t index) const {
	assert(index < m_keys.size() && "Key index is out of range!");
	return &m_keys[index];
}

float TrajectoryHistory::GetFirstTime() const {

	if (m_keys.empty()) {
		return 0.0f;
	}
	return m_keys.front().m_time;
}

float TrajectoryHistory::GetLastTime() const {

	if (m_keys.empty()) {
		return 0.0f;
	}
	return m_keys.back().m_time;
}

size_t TrajectoryHistory::FindKeyNumber(float time) const {
	for (size_t i = 0; i < m_keys.size(); ++i) {
		if (m_keys[i].m_time >= time) {
			return i;
		}
	}
	return InvalidIndex;
}

Sample TrajectoryHistory::GetValueAtTime(float time) const {
	if (m_keys.empty()) {
		return {};
	}
	if (time <= m_keys.front().m_time) {
		return m_keys.front().m_value;
	}
	if (time >= m_keys.back().m_time) {
		return m_keys.back().m_value;
	}
	for (size_t i = 1; i < m_keys.size(); ++i) {
		if (time <= m_keys[i].m_time) {
			const float t0 = m_keys[i - 1].m_time;
			const float t1 = m_keys[i].m_time;
			if (t1 <= t0) {
				return m_keys[i].m_value;
			}
			const float alpha = (time - t0) / (t1 - t0);

			return m_keys[i - 1].m_value * (1.0f - alpha) + m_keys[i].m_value * alpha;
		}
	}
	return m_keys.back().m_value;
}

}
