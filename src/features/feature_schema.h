#pragma once

#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

#include <godot_cpp/classes/ref.hpp>
#include <godot_cpp/classes/resource.hpp>
#include <godot_cpp/core/property_info.hpp>
#include <godot_cpp/templates/list.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/packed_string_array.hpp>
#include <godot_cpp/variant/string.hpp>

#include "features/feature.h"
#include "debug/compat.h"

namespace godot::MotionMatching {

class FeatureSchema : public Resource {
	GDCLASS(FeatureSchema, Resource)

public:
	virtual ~FeatureSchema();

	void AddFeature(const Ref<Feature> &feature);
	void AddFeature(Feature *feature);
	void Clear();

	size_t GetNumFeatures() const;
	Ref<Feature> GetFeature(size_t index) const;
	const std::vector<Ref<Feature>> &GetFeatures() const;

	Ref<Feature> FindFeatureById(const FeatureId &featureId) const;

	std::vector<std::string> CollectColumnNames() const;

	int64_t get_num_features() const;
	void add_feature(const Ref<Feature> &feature);
	Ref<Feature> get_feature(int64_t index) const;
	Ref<Feature> find_feature_by_id(const String &feature_id) const;
	PackedStringArray get_column_names() const;

	void populate_default_features();

	static Ref<Feature> CreateFeatureByType(const FeatureId &typeId);

	bool _set(const StringName &p_name, const Variant &p_value);
	bool _get(const StringName &p_name, Variant &r_ret) const;
	void _get_property_list(List<PropertyInfo> *p_list) const;

protected:
	static void _bind_methods();

	std::vector<Ref<Feature>> m_features;
	std::unordered_map<FeatureId, Ref<Feature>> m_featuresById;
};

}
