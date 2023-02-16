/*-------------------------------------------------------
Copyright (C) 20xx DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior written consent of
DigiPen Institute of Technology is prohibited.
File Name: model.hpp
Author: Unai Morentin, u.morentin, 540001418
---------------------------------------------------------*/

#pragma once
#include <unordered_map>
#include <string>
#include "../include/tinygltf/tiny_gltf.h"
#include "math.hpp"
#include "animation.hpp"

namespace cs460 {
	class Node;
	struct Model;

	struct Skin {
		Skin(std::vector<int>& gltf_idxs, int gltf_skel_idx, Model* model);
		std::vector<Node*> m_joints;
		Node* m_skeleton_root;
		bool m_show_skeleton = true;

		std::vector<glm::mat4> m_mtxs;

		void update();
		void render();
		void draw_bones(Node* child);
	};

	struct Model {
		Node* m_model_root = nullptr;
		std::vector<Node*> m_roots;
		Skin* m_skin = nullptr; 
		AnimationHandler* m_anim_handler = nullptr;
		std::string m_name;
		std::unordered_map<int, Node*> m_nodes;

		Model(std::string name) : m_name(name) {}
		~Model() {
			delete m_anim_handler;
			delete m_skin;
			for (auto& it : m_nodes)
				delete it.second;
			delete m_model_root;
		}
	};


	struct Model_Data {
		std::vector<Node*> m_roots;
		std::unordered_map<int, Node*> m_all_nodes;
		std::vector<int> m_joints;
		int m_skeleton_root;
		std::string m_name;
		tinygltf::Model m_model;
		bool m_loaded = false;

		std::vector<Datas*> m_anim_data;

		~Model_Data();
		Model_Data(tinygltf::Model m) : m_model(m) {}
		void deleteNode(Node* node);
		Model* spawn_model(float _spawnPos);
		Node* create_node(const Node* _node, Node* _parent, Model* spawning_model);
		//skinning
		void get_bind_inv_mtx(Node* joint, unsigned int idx, int gltf_idx);
		//animation
		void get_anim_data();
	};
}