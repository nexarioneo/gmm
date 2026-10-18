#pragma once

#include <cstddef>
#include <cstdint>

#include <godot_cpp/classes/animation_node.hpp>
#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/skeleton3d.hpp>
#include <godot_cpp/core/property_info.hpp>
#include <godot_cpp/templates/list.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/string_name.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include "features/feature_schema.h"
#include "database/motion_matching_data.h"
#include "runtime/motion_matching_instance.h"
#include "trajectory/trajectory_query.h"
#include "actor/actor_instance.h"
#include "actor/anim_graph_pose_pool.h"
#include "animation/motion_event_buffer.h"
#include "animation/motion_set.h"
#include "pose/pose.h"

#ifdef GMM_MONITOR_ENABLED

#include "debug/monitor.h"
#include "debug/monitor_bus.h"
#endif

namespace godot::MotionMatching {

class MotionMatchingNode : public AnimationNode {
	GDCLASS(MotionMatchingNode, AnimationNode)

public:

	static constexpr const char *INPUT_TARGET_POS = "target_pos";
	static constexpr const char *INPUT_TARGET_FACING_DIR = "target_facing_dir";
	static constexpr const char *INPUT_USE_FACING_DIR = "use_facing_dir";

	static constexpr int SERIALIZE_VERSION = 11;

	enum TrajectoryMode {
		TARGET_DRIVEN = 0,
		AUTOMATIC = 1,
	};

	enum ScalerType {
		STANDARD = 0,
		MIN_MAX = 1,
	};

	struct UniqueData {
		~UniqueData() {
			Clear();
		}
		void Clear();

		void Update(MotionMatchingNode *node);

		MotionMatchingInstance *m_instance = nullptr;
		MotionMatchingData *m_data = nullptr;
		bool m_hasError = false;

		MotionSet m_motionSet;

		ActorInstance m_actorInstance;
	};

	MotionMatchingNode();
	~MotionMatchingNode() override;

	double _process(double p_time, bool p_seek, bool p_is_external_seeking, bool p_test_only) override;

	bool _set(const StringName &p_name, const Variant &p_value);
	bool _get(const StringName &p_name, Variant &r_ret) const;
	void _get_property_list(List<PropertyInfo> *p_list) const;
	void _validate_property(PropertyInfo &p_property) const;

	Ref<FeatureSchema> GetFeatureSchema() const { return m_featureSchema; }

	float GetLowestCostSearchFrequency() const { return m_lowestCostSearchFrequency; }
	size_t GetSampleRate() const { return m_sampleRate; }
	TrajectoryQuery::EMode GetTrajectoryQueryMode() const { return m_trajectoryQueryMode; }
	float GetPathRadius() const { return m_pathRadius; }
	float GetPathSpeed() const { return m_pathSpeed; }
	bool GetNormalizeData() const { return m_normalizeData; }
	MotionMatchingData::FeatureScalerType GetFeatureScalerType() const { return m_featureScalerType; }
	float GetFeatureMin() const { return m_featureMin; }
	float GetFeatureMax() const { return m_featureMax; }
	bool GetClipFeatures() const { return m_clipFeatures; }
	size_t GetMaxKdTreeDepth() const { return m_maxKdTreeDepth; }
	size_t GetMinFramesPerKdTreeNode() const { return m_minFramesPerKdTreeNode; }
	bool GetMirror() const { return m_mirror; }
	const Array &GetMotions() const { return m_motions; }

	void set_search_frequency(float p_value);
	float get_search_frequency() const;
	void set_sample_rate(int64_t p_value);
	int64_t get_sample_rate() const;
	void set_trajectory_query_mode(int64_t p_value);
	int64_t get_trajectory_query_mode() const;
	void set_path_radius(float p_value);
	float get_path_radius() const;
	void set_path_speed(float p_value);
	float get_path_speed() const;
	void set_normalize_data(bool p_value);
	bool get_normalize_data() const;
	void set_feature_scaler_type(int64_t p_value);
	int64_t get_feature_scaler_type() const;
	void set_feature_min(float p_value);
	float get_feature_min() const;
	void set_feature_max(float p_value);
	float get_feature_max() const;
	void set_clip_features(bool p_value);
	bool get_clip_features() const;
	void set_max_kd_tree_depth(int64_t p_value);
	int64_t get_max_kd_tree_depth() const;
	void set_min_frames_per_kd_tree_node(int64_t p_value);
	int64_t get_min_frames_per_kd_tree_node() const;
	void set_mirror(bool p_value);
	bool get_mirror() const;
	void set_feature_schema(const Ref<FeatureSchema> &p_schema);
	Ref<FeatureSchema> get_feature_schema() const;
	void set_motions(const Array &p_motions);
	Array get_motions() const;
	void set_visualize_feature_schema(bool p_value);
	bool is_visualize_feature_schema() const;

	void set_target_position(const Vector3 &p_value);
	Vector3 get_target_position() const;
	void set_target_facing_dir(const Vector3 &p_value);
	Vector3 get_target_facing_dir() const;
	void set_use_facing_dir(bool p_value);
	bool get_use_facing_dir() const;

	float get_update_time_ms() const { return m_updateTimeInMs; }
	float get_post_update_time_ms() const { return m_postUpdateTimeInMs; }
	float get_output_time_ms() const { return m_outputTimeInMs; }

	void SetBuildSkeleton(Skeleton3D *skeleton, Node3D *root);
	Skeleton3D *m_buildSkeleton = nullptr;
	Node3D *m_buildRoot = nullptr;

	void reinit();

	void toggle_debug_draw_schema();

	bool is_debug_draw_schema_enabled() const { return m_visualizeFeatureSchema; }

protected:
	static void _bind_methods();

	friend class MotionMatching;

private:

	bool IsTrajectoryPathVisible() const;
	bool IsFeatureScalerTypeVisible() const;
	bool IsMinMaxVisible() const;

	void Update(float timePassedInSeconds, const Vector3 &targetPos, const Vector3 &targetFacingDir, bool useFacingDir);
	void PostUpdate(float timePassedInSeconds);
	void Output();

	void StampTick();
	float TickDeltaSeconds() const;

	float m_lowestCostSearchFrequency = 5.0f;
	size_t m_sampleRate = 30;
	TrajectoryQuery::EMode m_trajectoryQueryMode = TrajectoryQuery::MODE_TARGETDRIVEN;
	float m_pathRadius = 1.0f;
	float m_pathSpeed = 1.0f;
	bool m_normalizeData = false;
	MotionMatchingData::FeatureScalerType m_featureScalerType = MotionMatchingData::StandardScalerType;
	float m_featureMin = 0.0f;
	float m_featureMax = 1.0f;
	bool m_clipFeatures = false;
	size_t m_maxKdTreeDepth = 15;
	size_t m_minFramesPerKdTreeNode = 1000;
	bool m_mirror = false;
	Ref<FeatureSchema> m_featureSchema;
	Array m_motions;

	Vector3 m_targetPos;
	Vector3 m_targetFacingDir = Vector3(0.0f, 0.0f, -1.0f);
	bool m_useFacingDir = false;

	float m_preSyncTime = 0.0f;
	float m_currentPlayTime = 0.0f;

	bool m_visualizeFeatureSchema = false;

	float m_updateTimeInMs = 0.0f;
	float m_postUpdateTimeInMs = 0.0f;
	float m_outputTimeInMs = 0.0f;
	uint64_t m_tickStartMsec = 0;

	UniqueData m_uniqueData;

	bool m_dataDirty = true;

	AnimGraphPosePool m_posePool;
	Pose m_outputPose;
	MotionEventBuffer m_eventBuffer;
	Transform m_trajectoryDelta;

	const Pose &GetOutputPose() const { return m_outputPose; }

#ifdef GMM_MONITOR_ENABLED
	MotionMatchingMonitor m_monitor;
#endif
};

}
