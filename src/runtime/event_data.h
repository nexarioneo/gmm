#pragma once

#include <string>

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/string.hpp>

namespace godot::MotionMatching {

class EventData : public RefCounted {
	GDCLASS(EventData, RefCounted)

public:
	virtual ~EventData() = default;

	virtual bool Equal(const EventData &rhs, bool ignoreEmptyFields = false) const;

	bool equals(const EventData *other, bool ignore_empty_fields = false) const;

protected:
	static void _bind_methods();
};

class DiscardFrameEventData : public EventData {
	GDCLASS(DiscardFrameEventData, EventData)

public:
	DiscardFrameEventData() = default;
	~DiscardFrameEventData() override = default;

	bool Equal(const EventData &rhs, bool ignoreEmptyFields = false) const override;

protected:
	static void _bind_methods();

private:
	std::string m_tag;
};

class TagEventData : public EventData {
	GDCLASS(TagEventData, EventData)

public:
	TagEventData() = default;
	~TagEventData() override = default;

	bool Equal(const EventData &rhs, bool ignoreEmptyFields = false) const override;

	String get_tag() const;
	void set_tag(const String &tag);

protected:
	static void _bind_methods();

private:
	std::string m_tag;
};
}
