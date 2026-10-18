#pragma once

#include <cstdint>
#include <memory>

#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/core/property_info.hpp>
#include <godot_cpp/templates/list.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/node_path.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/string_name.hpp>
#include <godot_cpp/variant/variant.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include "database/motion_matching_data.h"
#include "debug/compat.h"
#include "debug/debug_renderer.h"
#include "features/feature_schema.h"
#include "nodes/motion_matching_node.h"

class ActorInstance;
class Motion;
class MotionInstance;

namespace godot {
class Skeleton3D;
class Viewport;
class World3D;
}

namespace godot::MotionMatching {

class MotionMatching : public Node {
	GDCLASS(MotionMatching, Node)

public:

	static constexpr int SERIALIZE_VERSION = 11;

	enum TrajectoryMode {
		TARGET_DRIVEN = 0,
		AUTOMATIC = 1,
	};

	enum ScalerType {
		STANDARD = 0,
		MIN_MAX = 1,
	};

	MotionMatching();
	~MotionMatching() override;

	void _ready() override;

	void _process(double p_delta) override;

	void _exit_tree() override;

	bool _set(const StringName &p_name, const Variant &p_value);
	bool _get(const StringName &p_name, Variant &r_ret) const;
	void _get_property_list(List<PropertyInfo> *p_list) const;
	void _validate_property(PropertyInfo &p_property) const;

	Ref<FeatureSchema> GetFeatureSchema() const;
	float GetLowestCostSearchFrequency() const;
	size_t GetSampleRate() const;
	TrajectoryQuery::EMode GetTrajectoryQueryMode() const;
	float GetPathRadius() const;
	float GetPathSpeed() const;
	bool GetNormalizeData() const;
	MotionMatchingData::FeatureScalerType GetFeatureScalerType() const;
	float GetFeatureMin() const;
	float GetFeatureMax() const;
	bool GetClipFeatures() const;
	size_t GetMaxKdTreeDepth() const;
	size_t GetMinFramesPerKdTreeNode() const;
	bool GetMirror() const;
	const Array &GetMotions() const;

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

	void set_skeleton_path(const NodePath &p_path);
	NodePath get_skeleton_path() const;
	::godot::Skeleton3D *resolve_skeleton() const;

	void refresh_joint_hints();

	void SetDebugDrawFeatureSchema(FeatureSchema *p_featureSchema);
	FeatureSchema *GetDebugDrawFeatureSchema() const;
	void debug_draw_frame_features(DebugDrawOutput &p_output);
	void clear_debug_draw();

	float get_update_time_ms() const;
	float get_post_update_time_ms() const;
	float get_output_time_ms() const;

	void reinit();
	void toggle_debug_draw_schema();
	bool is_debug_draw_schema_enabled() const;

protected:
	static void _bind_methods();

	void _on_core_motion_event(const String &tag);

private:

	Ref<MotionMatchingNode> m_core;

	NodePath m_skeletonPath;

	bool m_dataDirty = true;

	MotionMatchingDebugRenderer m_renderer;
	DebugDrawOutput m_debugOutput;
	bool m_drawActive = false;

	MotionInstance *m_lastMotionInstance = nullptr;
	std::unique_ptr<MotionMatchingData> m_debugData;
	Ref<FeatureSchema> m_debugVisFeatureSchema;
};

}
