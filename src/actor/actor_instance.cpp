#include "actor/actor_instance.h"

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/skeleton3d.hpp>

void ActorInstance::LinkActor(godot::Skeleton3D *skeleton, godot::Node3D *root) {
	m_actor.LinkActor(skeleton);
	m_transformData.LinkActorInstance(this);
	m_root = root;
}

Transform ActorInstance::GetWorldSpaceTransform() const {
	if (!m_root) {
		return Transform();
	}
	const godot::Transform3D globalTransform = m_root->get_global_transform();
	Transform result;
	result.m_position = globalTransform.get_origin();
	result.m_rotation = globalTransform.get_basis().get_rotation_quaternion();
	return result;
}
