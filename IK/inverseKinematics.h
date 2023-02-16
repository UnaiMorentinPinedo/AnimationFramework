/*-------------------------------------------------------
File Name: inverseKinematics.h
Author: Unai Morentin
---------------------------------------------------------*/

#pragma once
#include "fwd.hpp"
#include <vector>

namespace cs460 {
	struct Node;
	struct Renderer;

	struct Two_Bone_IK {
		Node* j_0, *j_1, *j_2; // 3 joints, 2 bones
		float d_1, d_2; // distance between joints, can be changed bya gui
		glm::vec3 m_target;

		Two_Bone_IK();
		~Two_Bone_IK();

		void Initialize();
		void Update();
		void Render();
		void End();

	private:
		void solve_IK();
		float solve_for_angle1(float angle_0);
		float solve_for_angle2();
		
		Renderer* m_renderer;
	};

	struct IK {
		std::vector<Node*> m_joints; //manipulator

		Node* m_target = nullptr;


		IK();
		~IK();
		void Initialize();
		void UpdateJointDistance(size_t idx, float dist);
		void AddJoint();
		void RemoveJoint();
		virtual void Update() {};
		void Render();
		void End() {};

		Renderer* m_renderer;

		//Distance threshold for which we consider the end effector to have successfully 
		//reached the target
		float distance_threshold = 0.4f;

		int m_number_of_iteration = 5;
		int m_total_number_of_iteration = 5;
		std::string m_state;
	};

	struct CCD : public IK { //cyclic coordinate descent
		virtual void Update();
		void Logic();
	};

	struct Fabrik : public IK { //cyclic coordinate descent
		std::vector<float> m_bones;
		std::vector<glm::vec3> m_joints_world_pos;
		virtual void Update();
		void Logic();
		void update_joints_orientations();

		void update_bones();
	};



}