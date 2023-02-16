/*-------------------------------------------------------
Copyright (C) 20xx DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior written consent of
DigiPen Institute of Technology is prohibited.
File Name: model.cpp
Author: Unai Morentin, u.morentin, 540001418
---------------------------------------------------------*/

#include "model.hpp"
#include "node.hpp"
#include "mesh.hpp"
#include "animation.hpp"
#include "renderer.hpp"

namespace cs460 {
	Skin::Skin(std::vector<int>& gltf_idxs, int gltf_skel_idx, Model* model) {
		for (auto& it : gltf_idxs) {
			m_joints.push_back(model->m_nodes.find(it)->second);
		}

		m_skeleton_root = model->m_nodes.find(gltf_skel_idx)->second;
	}

	void Skin::update() {
		m_mtxs.clear();
		for (auto& it : m_joints) {							//inv bind mtx
			glm::mat4 mtx = it->get_world().get_matrix() * it->m_skin_mtx;
			mtx = glm::inverse(m_skeleton_root->get_world().get_matrix()) * mtx;
			mtx = m_skeleton_root->get_local().get_matrix() * mtx;

			m_mtxs.push_back(mtx);
		}
	}

	void Skin::render() {
		if (!m_show_skeleton)
			return;
		draw_bones(m_skeleton_root);

		//render joints
		for (auto& it : m_joints)
			Scene::get_instance().m_renderer->debug_draw_point(it->get_world().m_position, { 1.f,0.f,0.f,1.f }, 3.f);
	}

	void Skin::draw_bones(Node* parent) {
		for (auto& it : parent->m_children) {
			Scene::get_instance().m_renderer->debug_draw_line(it->get_world().m_position, parent->get_world().m_position, { 0.f,0.2f,0.85f,1.f });
			draw_bones(it);
		}
	}

	Model_Data::~Model_Data() {
		for (auto& it : m_roots)
			deleteNode(it);
	}

	void Model_Data::deleteNode(Node* node) {
		for (auto& it : node->m_children)
			deleteNode(it);

		//delete meshes
		delete node->get_mesh();
	}

	Model* Model_Data::spawn_model(float _spawnPos) {
		Model* model_to_spawn = new Model(m_name);

		model_to_spawn->m_model_root = new Node(m_name, -1,  Scene::get_instance().m_root);
		model_to_spawn->m_model_root->m_model = model_to_spawn;

		// create roots hierarchicaly
		for (auto& it : m_roots) {
			model_to_spawn->m_roots.push_back(create_node(it, model_to_spawn->m_model_root, model_to_spawn));
		}

		//create model skin if  any
		if (m_joints.size() > 0)
			model_to_spawn->m_skin = new Skin(m_joints, m_skeleton_root, model_to_spawn);

		//create animation
		if (m_anim_data.size() > 0)
			model_to_spawn->m_anim_handler = new AnimationHandler(m_anim_data, model_to_spawn);

		//set postion not to overlapeed with other models
		model_to_spawn->m_model_root->set_position(glm::vec3(_spawnPos, 0.f, 0.f));
		//insert model root node in the scene graph
		Scene::get_instance().insert_node(model_to_spawn->m_model_root);

		return model_to_spawn;
	}

	Node* Model_Data::create_node(const Node* _model_node, Node* _parent, Model* spawning_model) {
		Node* new_node = new Node(_model_node, _parent);
		new_node->set_parent(_parent);
		new_node->m_model = spawning_model;

		for (auto& it : _model_node->m_children) {
			create_node(it, new_node, spawning_model);
		}

		//insert the node in the scene graph
		Scene::get_instance().insert_node(new_node);
		//insert the node in the model ligated to its gltf index
		spawning_model->m_nodes.insert({ new_node->m_gltf_idx, new_node });

		return new_node;
	}


	void Model_Data::get_bind_inv_mtx(Node* joint, unsigned int idx, int gltf_idx) {
		auto& skin = m_model.skins[0];

		//get all tiny gltf components
		auto& accessor = m_model.accessors[skin.inverseBindMatrices];
		auto& buffer_view = m_model.bufferViews[accessor.bufferView];
		auto& buffer = m_model.buffers[buffer_view.buffer];
		//  [0]		+	b_view.offset + acc.offset	+ idx in skins.joints vector * sizeof(mat4)
		glm::mat4* mtx = reinterpret_cast<glm::mat4*>(&buffer.data.at(0) + buffer_view.byteOffset + accessor.byteOffset + idx * accessor.ByteStride(buffer_view));
		joint->m_skin_mtx = *mtx;

		//add the joints idx
		m_joints.push_back(gltf_idx);
	}

	void Model_Data::get_anim_data() {
		for (auto& anim_it : m_model.animations) {
			auto new_anim_data = new Datas;
			new_anim_data->m_name = anim_it.name;
			auto& channels = anim_it.channels; 

			for (auto& it : channels) {
				//get channel's sampler
				auto& sampler = anim_it.samplers[it.sampler];

				//get input(key frames)
				auto& input_acc = m_model.accessors[sampler.input];
				auto& input_buffer_view = m_model.bufferViews[input_acc.bufferView];
				auto& input_buffer = m_model.buffers[input_buffer_view.buffer];

				Datas::Data data;
				//channel
				data.target = it.target_node;
				data.path = it.target_path;

				unsigned char* input_it = input_buffer.data.data() + input_acc.byteOffset + input_buffer_view.byteOffset;
				//get input data
				for (size_t i = 0; i < input_acc.count; i++) {
					float* pointer = reinterpret_cast<float*>(input_it);
					if (*pointer > new_anim_data->bigges_t)
						new_anim_data->bigges_t = *pointer;
					data.input.push_back(*pointer);

					input_it += input_acc.ByteStride(input_buffer_view);
				}

				//output
				auto& output_acc = m_model.accessors[sampler.output];
				auto& output_buffer_view = m_model.bufferViews[output_acc.bufferView];
				auto& output_buffer = m_model.buffers[output_buffer_view.buffer];

				//get output data
				unsigned char* output_it = output_buffer.data.data() + output_acc.byteOffset + output_buffer_view.byteOffset;

				for (size_t i = 0; i < output_acc.count; i++) {
					if (data.path.compare("translation") == 0) {
						glm::vec3* pointer = reinterpret_cast<glm::vec3*>(output_it);
						data.output_vec3.push_back(*pointer);
					}
					else if (data.path.compare("rotation") == 0) {
						glm::quat* pointer = reinterpret_cast<glm::quat*>(output_it);
						data.output_vec4.push_back(*pointer);
					}

					output_it += output_acc.ByteStride(output_buffer_view);
				}
				new_anim_data->m_datas.push_back(data);
			}
			m_anim_data.push_back(new_anim_data);
		}
	}
}