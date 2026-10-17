#include "EventData.h"

#include <godot_cpp/core/class_db.hpp>

namespace godot::MotionMatching {

void EventData::_bind_methods() {
	ClassDB::bind_method(D_METHOD("equals", "other", "ignore_empty_fields"), &EventData::equals, DEFVAL(false));
}

bool EventData::Equal([[maybe_unused]] const EventData &rhs, [[maybe_unused]] bool ignoreEmptyFields) const {

	return false;
}

bool EventData::equals(const EventData *other, bool ignore_empty_fields) const {
	if (other == nullptr) {
		return false;
	}
	return Equal(*other, ignore_empty_fields);
}

bool DiscardFrameEventData::Equal([[maybe_unused]] const EventData &rhs, [[maybe_unused]] bool ignoreEmptyFields) const {
	return true;
}

void DiscardFrameEventData::_bind_methods() {

}

bool TagEventData::Equal(const EventData &rhs, [[maybe_unused]] bool ignoreEmptyFields) const {
	const TagEventData *other = dynamic_cast<const TagEventData *>(&rhs);
	if (other) {

		return String(m_tag.c_str()).nocasecmp_to(String(other->m_tag.c_str())) == 0;
	}
	return false;
}

void TagEventData::_bind_methods() {

	ClassDB::bind_method(D_METHOD("get_tag"), &TagEventData::get_tag);
	ClassDB::bind_method(D_METHOD("set_tag", "tag"), &TagEventData::set_tag);
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "tag"), "set_tag", "get_tag");
}

String TagEventData::get_tag() const {
	return String(m_tag.c_str());
}

void TagEventData::set_tag(const String &tag) {
	m_tag = std::string(tag.utf8().get_data());
}

}
