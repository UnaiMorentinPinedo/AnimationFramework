/*-------------------------------------------------------
File Name: inverseKinematics.cpp
Author: Unai Morentin
---------------------------------------------------------*/

#include "inverseKinematics.h"
#include "node.hpp"
#include "scene.hpp"
#include "renderer.hpp"
#include "camera.hpp"
#include "editor.hpp"

namespace cs460 {
	Two_Bone_IK::Two_Bone_IK() :
		 j_0(new Node("joint 0", -1, Scene::get_instance().m_root)), 
		 j_1(new Node("joint 1", -1, j_0)),
		 j_2(new Node("joint 2", -1, j_1)) {
		
	}

	Two_Bone_IK::~Two_Bone_IK() {
		End();
		delete j_2; delete j_1;  delete j_0;
	}

	void Two_Bone_IK::Initialize() {
		d_1 = 3.f;
		d_2 = 3.f;
		//set initial position
		j_0->set_position({ 0.f, 0.f, 0.f });
		j_1->set_position({ d_1, 0.f, 0.f });
		j_2->set_position({ d_2, 0.f, 0.f });

		m_renderer = Scene::get_instance().m_renderer;
	}

	void Two_Bone_IK::Update() {
		//set joints position respect to d_1 and d_2
		j_1->set_position({ d_1, 0.f, 0.f });
		j_2->set_position({ d_2, 0.f, 0.f });
		 
		// do the IK
		solve_IK();

		Render();
	}

	void Two_Bone_IK::Render() {
		float right = j_0->get_world().m_position.x + d_1 + d_2;
		float left = j_0->get_world().m_position.x - d_1 - d_2;
		float top = j_0->get_world().m_position.y + d_1 + d_2;
		float down = j_0->get_world().m_position.y - d_1 - d_2;

		//debug draw axixes: d1+d2 radius
		m_renderer->debug_draw_line(j_0->get_world().m_position, glm::vec3(0, top, 0), { 1.f,1.f,1.f,1.f });
		m_renderer->debug_draw_line(j_0->get_world().m_position, glm::vec3(right,0 , 0), { 1.f,1.f,1.f,1.f });
		m_renderer->debug_draw_line(j_0->get_world().m_position, glm::vec3(0, down, 0), { 1.f,1.f,1.f,1.f });
		m_renderer->debug_draw_line(j_0->get_world().m_position, glm::vec3(left, 0, 0), { 1.f,1.f,1.f,1.f });

		//debug draw joints and bones
		m_renderer->debug_draw_point(j_0->get_world().m_position, { 0.2f,0.2f,1.f,0.5f });
		m_renderer->debug_draw_point(j_1->get_world().m_position, { 0.2f,0.2f,1.f,0.5f });
		m_renderer->debug_draw_point(j_2->get_world().m_position, { 0.2f,0.2f,1.f,0.5f });

		m_renderer->debug_draw_pyramid(j_0->get_world().m_position, j_1->get_world().m_position, { 0.6f,0.6f,0.6f,1.f });
		m_renderer->debug_draw_pyramid(j_1->get_world().m_position, j_2->get_world().m_position, { 0.6f,0.6f,0.6f,1.f });


		//debug draw end effector
		m_renderer->debug_draw_point(m_target, { 1.f,0.f,0.f,0.5f });
	}


	void Two_Bone_IK::solve_IK() {
		float angle_2 = solve_for_angle2();
		if (angle_2 == -1.f) //outside
			return;
		float angle_1 = solve_for_angle1(angle_2);

		//rotate joints
		j_0->get_transform()->rotate_in_Z_by_angle(angle_1);
		j_1->get_transform()->rotate_in_Z_by_angle(angle_2);
	}

	float Two_Bone_IK::solve_for_angle1(float angle_2) {
		float top_part = m_target.y * (d_1 + d_2 * cos(angle_2)) - m_target.x * (d_2 * sin(angle_2));
		float bottom_part = m_target.x * (d_1 + d_2 * cos(angle_2)) + m_target.y * (d_2 * sin(angle_2));

		return std::atan2(top_part, bottom_part);
	}

	float Two_Bone_IK::solve_for_angle2() {
		//compute top part of the division
		float top_part = (m_target.x * m_target.x) + (m_target.y * m_target.y) - ((d_1 * d_1) + (d_2 * d_2));
		float fin = top_part / (2 * d_1 * d_2);
		if (fin < -1) return -1.f;
		if (fin > 1) return -1.f;
		return glm::acos(fin);
	}

	void Two_Bone_IK::End() {

	}

	IK::IK() {
		m_joints.push_back(new Node("joint 0", -1, Scene::get_instance().m_root));
		m_joints.push_back(new Node("joint 1", -1, m_joints.back()));
		m_joints.push_back(new Node("joint 2", -1, m_joints.back()));
		m_joints.push_back(new Node("joint 3", -1, m_joints.back()));
		m_joints.push_back(new Node("joint 4", -1, m_joints.back()));
		m_joints.push_back(new Node("joint 5", -1, m_joints.back()));

		m_target = new Node("Target", -1, Scene::get_instance().m_root);
	}

	IK::~IK() {
		for (auto& it : m_joints)
			delete it;

		delete m_target;
	}

	void IK::Initialize() {
		for (auto& it : m_joints) {
			it->set_position({ 1.f, 0.f,0.f });
		}

		m_renderer = Scene::get_instance().m_renderer;
	}

	void IK::AddJoint() {
		std::string number = std::to_string(m_joints.size());
		Node* new_joint = new Node("joint "+ number, -1, m_joints.back());
		new_joint->set_position({ 1.f, 0.f,0.f });
		m_joints.push_back(new_joint);

		m_state = "PROCESSING";
	}

	void IK::RemoveJoint() {
		Node* to_remove = m_joints.back();
		m_joints.resize(m_joints.size() - 1);
		to_remove->get_parent()->m_children.clear();
		delete to_remove;

		m_state = "PROCESSING";
	}

	void IK::UpdateJointDistance(size_t idx, float dist) {
		Node* joint = m_joints[idx];
		Node* parent = m_joints[idx - 1];

		glm::vec3 v = glm::normalize(parent->get_world().m_position - joint->get_world().m_position);

		joint->set_position(v * dist);
	}

	void IK::Render() {
		float totaldist = 0.f;
		for (unsigned int i = 0; i < m_joints.size() - 1; i++) {
			Node* n_0 = m_joints[i];
			m_renderer->debug_draw_point(n_0->get_world().m_position, { 0.2f,0.2f,1.f,0.5f });
			Node* n_1 = m_joints[i+1];
			m_renderer->debug_draw_pyramid(n_0->get_world().m_position, n_1->get_world().m_position, { 0.6f,0.6f,0.6f,1.f });

			totaldist += glm::length(n_1->get_world().m_position - n_0->get_world().m_position);
		}
		//draw last element
		m_renderer->debug_draw_point(m_joints.back()->get_world().m_position, { 0.2f,0.2f,1.f,0.5f });

		//render target
		m_renderer->debug_draw_point(m_target->get_world().m_position, { 1.f,0.f,0.f,0.5f }, 16.f);


		//render axises
		glm::vec3 root_pos = m_joints.front()->get_world().m_position;
		float right = root_pos.x + totaldist;
		float left = root_pos.x - totaldist;
		float top = root_pos.y + totaldist;
		float down = root_pos.y - totaldist;
		float far_ = root_pos.z - totaldist;
		float close = root_pos.z + totaldist;


		m_renderer->debug_draw_line(root_pos, glm::vec3(root_pos.x, top, root_pos.z), { 1.f,1.f,1.f,1.f });
		m_renderer->debug_draw_line(root_pos, glm::vec3(root_pos.x, down, root_pos.z), { 1.f,1.f,1.f,1.f });
		m_renderer->debug_draw_line(root_pos, glm::vec3(right, root_pos.y, root_pos.z), { 1.f,1.f,1.f,1.f });
		m_renderer->debug_draw_line(root_pos, glm::vec3(left, root_pos.y, root_pos.z), { 1.f,1.f,1.f,1.f });
		m_renderer->debug_draw_line(root_pos, glm::vec3(root_pos.x, root_pos.y, far_), { 1.f,1.f,1.f,1.f });
		m_renderer->debug_draw_line(root_pos, glm::vec3(root_pos.x, root_pos.y, close), { 1.f,1.f,1.f,1.f });
	}

	void CCD::Update() {
		//logic
		if(m_state.compare("PROCESSING") == 0)
			Logic();

		IK::Render();
	}

	void CCD::Logic() {
		// target position
		glm::vec3 target_pos = m_target->get_world().m_position;

		//starting at joint previous to end effector and work inward towards the root joint
		for (int i = static_cast<int>(m_joints.size()) - 2; i >= 0; i--) {
			//end effector position
			glm::vec3 end_effector_pos = m_joints.back()->get_world().m_position;

			Node* joint = m_joints[i];
			//get joints pos 
			glm::vec3 joint_pos = joint->get_world().m_position;

			glm::vec3 v1 = glm::normalize(end_effector_pos - joint_pos);
			glm::vec3 v2 = glm::normalize(target_pos - joint_pos);

			float angle = glm::acos(glm::clamp(glm::dot(v1, v2), -1.f, 1.f));
			glm::vec3 r;
			if (std::abs(glm::dot(v1, v2)) < 1)
				r = glm::normalize(glm::cross(v1, v2));
			else
				r = glm::vec3(0.f, 0.f, 1.f); //arbitrary vec3

			glm::quat q = glm::angleAxis(angle, r);

			joint->get_local().m_rotation = q * joint->get_local().m_rotation;

			//Update the world transform of the manipulator, starting at this joint going all the way to the end effector
			for (unsigned int k = i; k < m_joints.size(); k++) {
				m_joints[k]->get_transform()->update();
			}
			//check endeffector and target distance < distance threshold
			if (glm::length(m_joints.back()->get_world().m_position - m_target->get_world().m_position) < distance_threshold) {
				m_number_of_iteration = m_total_number_of_iteration;
				m_state = "SUCCESS";
				return;
			}
		}
		if (m_number_of_iteration > 0) {
			m_number_of_iteration--;
			m_state = "PROCESSING";
			return;
		}
		m_number_of_iteration = m_total_number_of_iteration;
		m_state = "FAILURE";
	}

	void Fabrik::Update() {
		update_bones();
		//logic
		if (m_state.compare("PROCESSING") == 0){
			Logic();
			update_joints_orientations();
		}

		IK::Render();
	}

	void Fabrik::Logic() {
		glm::vec3 root_pos = m_joints.front()->get_world().m_position;

		//FORWARD STEP
		//set the end effector to the target position
		m_joints_world_pos.back() = m_target->get_world().m_position;

		for (size_t i = m_joints_world_pos.size() - 1; i > 0; i--) {
			//current joint pos
			glm::vec3& curr_joint_pos = m_joints_world_pos[i];
			// parent joint pos
			glm::vec3& parent_joint_pos = m_joints_world_pos[i-1];
			//distance betwwwen both
			float dist = m_bones[i - 1];

			glm::vec3 v = glm::normalize(parent_joint_pos - curr_joint_pos);
			//update parent position
			parent_joint_pos = curr_joint_pos + v * dist;
		}

		//BACKWARD STEP
		m_joints_world_pos.front() = root_pos;

		for (size_t i = 0; i < m_joints_world_pos.size() - 1; i++) {
			glm::vec3 v = glm::normalize(m_joints_world_pos[i + 1] - m_joints_world_pos[i]);
			m_joints_world_pos[i + 1] = m_joints_world_pos[i] + v * m_bones[i];
		}
	}

	void Fabrik::update_joints_orientations() {
		//end effector position
		glm::vec3 end_effector_pos = m_joints.back()->get_world().m_position;
		// target position
		glm::vec3 target_pos = m_target->get_world().m_position;

		for (size_t i = 0; i < m_joints.size() - 1; i++) {
			Node* joint = m_joints[i];
			//get joints pos 
			glm::vec3 joint_pos = joint->get_world().m_position;

			glm::vec3 v1 = glm::normalize(m_joints[i + 1]->get_world().m_position - joint_pos);
			glm::vec3 v2 = glm::normalize(m_joints_world_pos[i + 1] - joint_pos);

			float angle = glm::acos(glm::clamp(glm::dot(v1, v2), -1.f, 1.f));
			glm::vec3 r;
			if (std::abs(glm::dot(v1, v2)) < 1)
				r = glm::normalize(glm::cross(v1, v2));
			else
				r = glm::vec3(0.f, 0.f, 1.f); //arbitrary vec3

			glm::quat q = glm::angleAxis(angle, r);

			joint->get_world().m_rotation = q * joint->get_world().m_rotation;

			//update joint's local
			if (joint->get_parent())
				joint->get_local().m_rotation = glm::inverse(joint->get_parent()->get_world().m_rotation) * joint->get_world().m_rotation;
			else
				joint->get_local() = joint->get_world();

			//Update the world transform of the manipulator, starting at this joint going all the way to the end effector
			for (unsigned int k = i; k < m_joints.size(); k++) {
				m_joints[i]->get_transform()->update();
			}

			//check endeffector and target distance < distance threshold
			if (glm::length(m_joints.back()->get_world().m_position - m_target->get_world().m_position) < 0.5f) {
				m_state = "SUCCESS";
				return;
			}

			if (m_number_of_iteration > 0) {
				m_number_of_iteration--;
				m_state = "PROCESSING";
				return;
			}

			m_state = "FAILURE";

		}
	}

	void Fabrik::update_bones() {
		m_bones.clear();
		m_joints_world_pos.clear();

		for (size_t i = 0; i < m_joints.size() - 1; i++) {
			Node* node0 = m_joints[i];
			Node* node1 = m_joints[i+1];

			m_joints_world_pos.push_back(node0->get_world().m_position);
			m_bones.push_back(glm::length(node1->get_world().m_position - node0->get_world().m_position));
		}
		//store last item world position
		m_joints_world_pos.push_back(m_joints.back()->get_world().m_position);
	}
}