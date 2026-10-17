#include "FeatureSchema.h"

#include <algorithm>
#include <cassert>

#include <godot_cpp/classes/class_db_singleton.hpp>
#include <godot_cpp/classes/object.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/string_name.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#include <godot_cpp/core/memory.hpp>

#include "FeaturePosition.h"
#include "FeatureSchemaDefault.h"
#include "FeatureTrajectory.h"
#include "FeatureVelocity.h"

namespace godot::MotionMatching {

void FeatureSchema::_bind_methods() {
	ClassDB::bind_method(D_METHOD("add_feature", "feature"), &FeatureSchema::add_feature);
	ClassDB::bind_method(D_METHOD("clear"), &FeatureSchema::Clear);
	ClassDB::bind_method(D_METHOD("get_num_features"), &FeatureSchema::get_num_features);
	ClassDB::bind_method(D_METHOD("get_feature", "index"), &FeatureSchema::get_feature);
	ClassDB::bind_method(D_METHOD("find_feature_by_id", "feature_id"), &FeatureSchema::find_feature_by_id);
	ClassDB::bind_method(D_METHOD("get_column_names"), &FeatureSchema::get_column_names);
	ClassDB::bind_method(D_METHOD("populate_default_features"), &FeatureSchema::populate_default_features);
}

FeatureSchema::~FeatureSchema() {
	Clear();
}

Ref<Feature> FeatureSchema::GetFeature(size_t index) const {
	if (index < m_features.size()) {
		return m_features[index];
	}
	return Ref<Feature>();
}

const std::vector<Ref<Feature>> &FeatureSchema::GetFeatures() const {
	return m_features;
}

void FeatureSchema::AddFeature(const Ref<Feature> &feature) {
	if (feature.is_null()) {
		return;
	}
	auto iterator = std::find_if(m_featuresById.begin(), m_featuresById.end(), [&feature](const auto &curEntry) -> bool {
		return (feature->GetId() == curEntry.second->GetId());
	});

	if (iterator != m_featuresById.end()) {
		assert(false && "Cannot add feature. Feature with the same id has already been registered.");
		return;
	}

	m_featuresById.emplace(feature->GetId(), feature);
	m_features.emplace_back(feature);
}

void FeatureSchema::AddFeature(Feature *feature) {
	AddFeature(Ref<Feature>(feature));
}

void FeatureSchema::Clear() {
	m_featuresById.clear();
	m_features.clear();
}

size_t FeatureSchema::GetNumFeatures() const {
	return m_features.size();
}

Ref<Feature> FeatureSchema::FindFeatureById(const FeatureId &featureId) const {
	const auto result = m_featuresById.find(featureId);
	if (result == m_featuresById.end()) {
		return Ref<Feature>();
	}

	return result->second;
}

Ref<Feature> FeatureSchema::CreateFeatureByType(const FeatureId &typeId) {
	ClassDBSingleton *class_db = ClassDBSingleton::get_singleton();
	if (class_db == nullptr) {
		UtilityFunctions::push_error("Motion Matching: Cannot get ClassDB singleton.");
		return Ref<Feature>();
	}

	const StringName class_name(String(typeId.c_str()));
	if (!class_db->class_exists(class_name) || !class_db->can_instantiate(class_name)) {
		UtilityFunctions::push_warning("Motion Matching: Cannot find instantiable class for this type.");
		return Ref<Feature>();
	}

	Variant v = class_db->instantiate(class_name);
	Ref<Feature> f = v;
	return f;
}

std::vector<std::string> FeatureSchema::CollectColumnNames() const {
	std::vector<std::string> columnNames;

	for (const Ref<Feature> &feature : m_features) {
		if (feature.is_valid()) {
			const size_t numDimensions = feature->GetNumDimensions();
			for (size_t dimension = 0; dimension < numDimensions; ++dimension) {
				columnNames.push_back(feature->GetDimensionName(dimension));
			}
		}
	}

	return columnNames;
}

int64_t FeatureSchema::get_num_features() const {
	return (int64_t)GetNumFeatures();
}

void FeatureSchema::add_feature(const Ref<Feature> &feature) {
	AddFeature(feature);
}

Ref<Feature> FeatureSchema::get_feature(int64_t index) const {
	return GetFeature((size_t)index);
}

Ref<Feature> FeatureSchema::find_feature_by_id(const String &feature_id) const {
	return FindFeatureById(FeatureId(feature_id.utf8().get_data()));
}

PackedStringArray FeatureSchema::get_column_names() const {
	PackedStringArray out;	for (const std::string &name : CollectColumnNames()) {
		out.push_back(String(name.c_str()));
	}
	return out;
}

bool FeatureSchema::_set(const StringName &p_name, const Variant &p_value) {
	const String key = String(p_name);
	if (key == "features" && p_value.get_type() == Variant::ARRAY) {
		Array arr = p_value;
		Clear();
		for (int64_t i = 0; i < arr.size(); ++i) {
			Object *o = arr[i];
			Feature *f = Object::cast_to<Feature>(o);
			if (f) {
				AddFeature(Ref<Feature>(f));
			} else {

				Ref<FeaturePosition> created = memnew(FeaturePosition);
				AddFeature(created);
			}
		}
		return true;
	}
	return false;
}

bool FeatureSchema::_get(const StringName &p_name, Variant &r_ret) const {
	const String key = String(p_name);
	if (key == "features") {

		Array arr;
		arr.set_typed(Variant::OBJECT, StringName("Feature"), Variant());
		for (const Ref<Feature> &f : m_features) {
			arr.push_back(Variant(f.ptr()));
		}
		r_ret = arr;
		return true;
	}
	return false;
}

void FeatureSchema::_get_property_list(List<PropertyInfo> *p_list) const {

	const String hint = String::num_int64(Variant::OBJECT) + "/" + String::num_int64(PROPERTY_HINT_RESOURCE_TYPE) + ":Feature";
	p_list->push_back(PropertyInfo(Variant::ARRAY, "features", PROPERTY_HINT_ARRAY_TYPE, hint, PROPERTY_USAGE_DEFAULT));
}

void FeatureSchema::populate_default_features() {

	if (GetNumFeatures() != 0) {
		return;
	}
	Ref<FeatureTrajectory> trajectory = memnew(FeatureTrajectory);
	AddFeature(trajectory);
	Ref<FeaturePosition> leftPos = memnew(FeaturePosition);
	leftPos->SetName("Left Foot Position");
	AddFeature(leftPos);
	Ref<FeaturePosition> rightPos = memnew(FeaturePosition);
	rightPos->SetName("Right Foot Position");
	AddFeature(rightPos);
	Ref<FeatureVelocity> leftVel = memnew(FeatureVelocity);
	leftVel->SetName("Left Foot Velocity");
	AddFeature(leftVel);
	Ref<FeatureVelocity> rightVel = memnew(FeatureVelocity);
	rightVel->SetName("Right Foot Velocity");
	AddFeature(rightVel);
	Ref<FeatureVelocity> pelvisVel = memnew(FeatureVelocity);
	pelvisVel->SetName("Pelvis Velocity");
	AddFeature(pelvisVel);
}

}
