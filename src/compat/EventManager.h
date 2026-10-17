#pragma once

class EventManager {
public:
	EventManager() = default;
	~EventManager() = default;

	EventManager(const EventManager &) = delete;
	EventManager &operator=(const EventManager &) = delete;
};
