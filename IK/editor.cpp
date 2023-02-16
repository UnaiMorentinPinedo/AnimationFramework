/*-------------------------------------------------------
File Name: editor.cpp
Author: Unai Morentin
---------------------------------------------------------*/

#include "editor.hpp"
#include "renderer.hpp"
#include "resources.hpp"
#include "loader.hpp"
#include "camera.hpp"
#include "geometry.h"
#include "model.hpp"
#include "inverseKinematics.h"

namespace cs460 {
	using namespace ImGui;

	Editor::Editor() : m_selected(nullptr), m_path("resources/models/"){

	}

	Editor::~Editor() {
	}

	void Editor::create(GLFWwindow* _window) {
		IMGUI_CHECKVERSION();
		// Create context
		ImGui::CreateContext();

		// initialize IO
		ImGuiIO& io = ImGui::GetIO();


		ImGui_ImplGlfw_InitForOpenGL(_window, true);
		const char* glsl_version = "#version 130";
		ImGui_ImplOpenGL3_Init(glsl_version);

		ImGui::StyleColorsClassic();

		m_scene = &Scene::get_instance();
		m_window = m_scene->m_renderer->get_window();

		create_model_instances();
	}

	
	void Editor::create_model_instances() {
		/*m_scene->m_renderer->m_resources->m_models.insert({ "MIXAMO/xbot.gltf", new Model_Data(Mixamo) });*/
	}

	void Editor::update() {
		new_frame();

		//show_node_data();
		//show_nodes_list();
		if (m_selected)
			updateGizmos();

		if (m_scene->m_using_twobone)
			compute_target();
		else
			m_selected = m_scene->m_renderer->m_IK->m_target;

		//GUI
		if (m_scene->m_using_twobone)
			two_bone_IK_GUI();
		else
			IK_GUI();

		render();
	}

	void Editor::show_node_data() {
		if (!m_selected)
			return;
		if(ImGui::Begin("Node info")) {
			//NAME
			std::string msg = "Name: " + m_selected->get_name();
			ImGui::MenuItem(msg.c_str());
			ImGui::Separator();
			//TRANSFORM
			Transform& tr = m_selected->get_local();
			if (ImGui::InputFloat3("Position", &tr.m_position[0])) {
			}
			if (ImGui::InputFloat3("Scale", &tr.m_scale[0])) {
			}
			if (ImGui::InputFloat4("Rotation", &tr.m_rotation[0])) {
			}
			ImGui::Separator();
			//MESH
			Mesh* m = m_selected->get_mesh();
			if (m) {
				std::vector<Primitive*> pri = m->m_primitives;
				for (auto& it : pri) {
					auto material = "Material: " + it->m_material.m_gltf_material.name;
					ImGui::MenuItem(material.c_str());
					auto msg = "Number of vertex: " + std::to_string(it->m_vert_number);
					ImGui::MenuItem(msg.c_str());
				}
			}
			ImGui::Separator();
			//SKIN
			if (m_selected->m_model && m_selected->m_model->m_skin) {
				ImGui::MenuItem("Animation:");
				ImGui::Checkbox("Show skeleton", &m_selected->m_model->m_skin->m_show_skeleton);
			}
			ImGui::Separator();
		}
		ImGui::End();
	}

	void Editor::show_nodes_list() {
		int counter = 0;
		if (ImGui::Begin("Node list:")) 
			show_node_tree(m_scene->m_root, counter);

		ImGui::End();
	}

	void Editor::show_node_tree(Node* _node, int& counter) {
		counter++;
		std::string msg = "#" + std::to_string(counter) + ": " + _node->get_name();
		if (ImGui::TreeNodeEx(msg.c_str())) {
			if (ImGui::Selectable(("Select " + msg).c_str())) {
				m_selected = _node;
			}
			for (unsigned int i = 0u; i < _node->m_children.size(); i++) {
				show_node_tree(_node->m_children[i], counter);
			}
			ImGui::TreePop();
		}
	}

	void Editor::compute_target() {
		//debug
		ImVec2 mouse_pos = ImGui::GetIO().MousePos;
		//top left is origin
		mouse_pos.y =/* window->get_size().y*/720 - mouse_pos.y;

		//unproject
		const auto& view_mtx = m_scene->m_renderer->m_camera->view_mtx;
		const auto& proj_mtx = m_scene->m_renderer->m_camera->proj_mtx;
		glm::vec4 viewport = { 0.f, 0.f, 1280, 720 };
		glm::vec3 near_pos = glm::unProject({ mouse_pos.x, mouse_pos.y, 0.f }, view_mtx, proj_mtx, viewport);
		glm::vec3 far_pos = glm::unProject({ mouse_pos.x, mouse_pos.y, 1.f }, view_mtx, proj_mtx, viewport);

		glm::vec3 r = glm::normalize(far_pos - near_pos);
		glm::vec3 cam_pos = m_scene->m_renderer->m_camera->Position; // origin for the ray

		//find t
		if (r.z == 0.f) return;
		float t = -cam_pos.z / r.z;
		m_scene->m_renderer->m_twoBone_IK->m_target = cam_pos + t * r;
	}

	void Editor::updateGizmos() {
		ImGuiIO& io = ImGui::GetIO();
		ImGuizmo::SetRect(0.f,0.f, io.DisplaySize.x, io.DisplaySize.y);

		glm::mat4 manipulated_mtx = m_selected->get_world().get_matrix();
		ImGuizmo::Manipulate(&m_scene->m_renderer->m_camera->view_mtx[0][0], &m_scene->m_renderer->m_camera->proj_mtx[0][0],
			m_guizmo_op, ImGuizmo::LOCAL, &manipulated_mtx[0][0]);

		if (ImGuizmo::IsUsing()) {
			if (m_selected->get_parent()) {
				Transform tr;
				tr.descompose_transform(manipulated_mtx);
				m_selected->get_local() = m_selected->get_parent()->get_transform()->get_world().inverse_concatenate_transforms(tr);
			}
			else
				m_selected->get_transform()->get_local().descompose_transform(manipulated_mtx);

			if (!m_scene->m_using_twobone)
				m_scene->m_renderer->m_IK->m_state = "PROCESSING";
		}
	}

	void Editor::two_bone_IK_GUI() {
		if (Begin("Two Bone IK")) {
			Text("Using Analytical Two-bone IK in 2D");
			Separator();
			SliderFloat("d1", &m_scene->m_renderer->m_twoBone_IK->d_1, 0.1f, 10.f);
			SliderFloat("d2", &m_scene->m_renderer->m_twoBone_IK->d_2, 0.1f, 10.f);
		}
		End();
	}

	void Editor::IK_GUI() {
		if (Begin("IK")) {
			if (dynamic_cast<CCD*>(m_scene->m_renderer->m_IK))
				Text("Using CCD");
			else
				Text("Using FABRIK");
			Separator();
			Text(m_scene->m_renderer->m_IK->m_state.c_str());
			Separator();
			SliderFloat("Distance threshold", &m_scene->m_renderer->m_IK->distance_threshold, 0.05f, 2.f);
			SliderInt("Number of iterations", &m_scene->m_renderer->m_IK->m_total_number_of_iteration, 1, 1000);
			Separator();
			for (size_t i = 0; i < m_scene->m_renderer->m_IK->m_joints.size() ;i++)
			{
				Text(m_scene->m_renderer->m_IK->m_joints[i]->get_name().c_str());
				if (i != 0) {
					float dist = glm::length(m_scene->m_renderer->m_IK->m_joints[i-1]->get_world().m_position - m_scene->m_renderer->m_IK->m_joints[i]->get_world().m_position);
					std::string text = "joint " + std::to_string(i) + " distance";
					if (SliderFloat(text.c_str(), &dist, 0.5f, 10.f)) {
						m_scene->m_renderer->m_IK->UpdateJointDistance(i, dist);
					}
				}
			}

			Separator();
			if (Button("Add joint"))
				m_scene->m_renderer->m_IK->AddJoint();
			if (Button("Delete joint"))
				m_scene->m_renderer->m_IK->RemoveJoint();
		}
		End();
	}

	void Editor::render() {
		// render your GUI
		// Render dear imgui into screen
		ImGui::Render();
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
	}

	void Editor::new_frame() {
		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();
		ImGuizmo::BeginFrame();
	}

	void Editor::destroy() {
		ImGui_ImplOpenGL3_Shutdown();
		ImGui_ImplGlfw_Shutdown();
		ImGui::DestroyContext();
	}

}