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
#include "blending.hpp"

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

		//create world
		m_selected = m_scene->create_world();
	}

	
	void Editor::create_model_instances() {
		m_scene->m_renderer->m_resources->m_models.insert({ "MIXAMO/xbot.gltf", new Model_Data(Mixamo) });
		m_scene->m_renderer->m_resources->m_models.insert({ "MIXAMO2/xbot.gltf", new Model_Data(Mixamo2) });
	}

	void Editor::update() {
		new_frame();

		if (!m_scene->demo_1D && !m_scene->demo_2D) {
			show_node_data();
			show_nodes_list();

			if (m_selected && m_selected->m_model)
				show_animation_blending();
			//if (glfwGetMouseButton(m_window->get_window(), GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS)
			//	picking();
			if (m_selected)
				updateGizmos();
		}
		if (m_scene->demo_1D) {
			show_demo_1D();
		}
		else if (m_scene->demo_2D) {
			show_demo_2D();

		}

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

	void Editor::show_animation_blending() {
		bool open = true;
		if (Begin("Animation Blending", &open, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize)) {
			AnimationHandler* m_anim_handler = m_selected->m_model->m_anim_handler;
			if (BeginCombo("Blending type", get_blending_type_string().c_str())) {
				if (Selectable("1D")) {
					m_anim_handler->e_blending_type = BLENDINGTYPE::_1D;
					m_anim_handler->ResetBlendTree();
				}
				else if (Selectable("2D")) {
					m_anim_handler->e_blending_type = BLENDINGTYPE::_2D;
					m_anim_handler->ResetBlendTree();
				}
				EndCombo();
			}

			switch (m_anim_handler->e_blending_type) {
			case BLENDINGTYPE::_1D:
				blending_1D();
				break;
			case BLENDINGTYPE::_2D:
				blending_2D();
				break;
			}
		}

		End();
	}

	void Editor::blending_1D() {
		BeginChild("a", ImVec2(0, 0), true);

		AnimationHandler* m_anim_handler = m_selected->m_model->m_anim_handler;

		if (BeginCombo("Add Node", "")) {
			for (auto& it : m_anim_handler->m_animations) {
				if (it->added)
					continue;

				if (Selectable(it->m_name.c_str())) {
					it->added = true;
					m_anim_handler->AddNode(it);
				}
			}
			EndCombo();
		}

		SliderFloat("Blend factor", &m_anim_handler->m_blend_tree->m_blend_param->x, 0.f, m_anim_handler->m_blend_tree->m_children.size() > 1 ? m_anim_handler->m_blend_tree->m_children.back()->m_blend_pos.x : 1.f);
		for (auto& it : m_anim_handler->m_blend_tree->m_children) {
			std::string name = "Blend node " + it->m_animation->m_name;
			if (TreeNodeEx(name.c_str())) {
				if (SliderFloat("pos", &it->m_blend_pos.x, 0.f, 10.f))
					m_anim_handler->SortNodes();
				if (MenuItem("Erase node"))
					m_anim_handler->EraseNode(it);
				TreePop();
			}
		}
		BeginChild("g", ImVec2(0, 0), true);

		EndChild();
		blending_1D_blendspace();
		EndChild();

	}

	void Editor::blending_1D_blendspace() {
		ImVec2 w_min = ImGui::GetItemRectMin();
		ImVec2 w_max = ImGui::GetItemRectMax();
		ImDrawList* draw_list = ImGui::GetWindowDrawList();
		draw_list->PushClipRect(w_min, w_max);

		draw_list->AddRectFilled(w_min, w_max, IM_COL32(220, 220, 220, 160));

		ImVec2 size = ImVec2(w_max.x - w_min.x, w_max.y - w_min.y);

		float w_mid_x = w_min.x + size.x / 2;
		float w_mid_y = w_min.y + size.y / 2;

		ImVec2 line_minPos = ImVec2(w_mid_x, w_min.y + 13);
		ImVec2 line_maxPos = ImVec2(w_mid_x, w_max.y - 13);

		AnimationHandler* m_anim_handler = m_selected->m_model->m_anim_handler;

		//create max and min points and the line
		draw_list->AddLine(ImVec2(line_minPos.x, line_minPos.y - 5.f), ImVec2(line_maxPos.x, line_maxPos.y + 5.f), IM_COL32(0, 0, 0, 255), 3);

		// map min position to smallest node's position
		if (m_anim_handler->m_blend_tree->m_children.size() >= 1)
			draw_list->AddCircle(line_minPos, 2.5f, IM_COL32(0, 0, 0, 160), 10, 10);
		// map max position to biggest node's position
		if (m_anim_handler->m_blend_tree->m_children.size() >= 2) {
			draw_list->AddCircle(line_maxPos, 2.5f, IM_COL32(0, 0, 0, 160), 10, 10);
			// map blend factor to the line
			float mapped_pos = line_minPos.y + (m_anim_handler->m_blend_tree->m_blend_param->x * (line_maxPos.y - line_minPos.y)) / m_anim_handler->m_blend_tree->m_children.back()->m_blend_pos.x;
			draw_list->AddCircle(ImVec2(w_mid_x, mapped_pos), 3.5f, IM_COL32(130, 90, 44, 220), 10, 10);
		}

		//map other blend nodes to that line
		if (m_anim_handler->m_blend_tree->m_children.size() > 2) {
			//ignore first and last nodes
			for (int i = 1; i < m_anim_handler->m_blend_tree->m_children.size() - 1; i++) {
				float mapped_pos = line_minPos.y + (m_anim_handler->m_blend_tree->m_children[i]->m_blend_pos.x * (line_maxPos.y - line_minPos.y)) / m_anim_handler->m_blend_tree->m_children.back()->m_blend_pos.x;
				draw_list->AddCircle(ImVec2(w_mid_x, mapped_pos), 2.5f, IM_COL32(0, 0, 0, 160), 10, 10);
			}
		}

		draw_list->PopClipRect();
	}

	void Editor::blending_2D() {
		BeginChild("a", ImVec2(0, 0), true);

		AnimationHandler* m_anim_handler = m_selected->m_model->m_anim_handler;

		if (BeginCombo("Add Node", "")) {
			for (auto& it : m_anim_handler->m_animations) {
				if (it->added)
					continue;

				if (Selectable(it->m_name.c_str())) {
					it->added = true;
					m_anim_handler->AddNode(it);
				}
			}
			EndCombo();
		}

		SliderFloat("Blend factor x", &m_anim_handler->m_blend_tree->m_blend_param->x, 0.f, 10.f);
		SliderFloat("Blend factor y", &m_anim_handler->m_blend_tree->m_blend_param->y, 0.f, 10.f);
		for (auto& it : m_anim_handler->m_blend_tree->m_children) {
			std::string name = "Blend node " + it->m_animation->m_name;
			if (TreeNodeEx(name.c_str())) {
				if (SliderFloat2("pos", &it->m_blend_pos.x, 0.f, 10.f))
					;
				if (MenuItem("Erase node"))
					m_anim_handler->EraseNode(it);
				TreePop();
			}
		}
		BeginChild("g", ImVec2(0, 0), true);

		EndChild();
		blending_2D_blendspace();
		EndChild();
	}


	void Editor::blending_2D_blendspace() {
		glm::vec2 w_min = {ImGui::GetItemRectMin().x, ImGui::GetItemRectMin().y};
		glm::vec2 w_max = {ImGui::GetItemRectMax().x, ImGui::GetItemRectMax().y};

		ImDrawList* draw_list = ImGui::GetWindowDrawList();
		draw_list->PushClipRect(ImVec2(w_min.x, w_min.y), ImVec2(w_max.x, w_max.y));

		draw_list->AddRectFilled(ImVec2(w_min.x, w_min.y), ImVec2(w_max.x, w_max.y), IM_COL32(220, 220, 220, 160));

		glm::vec2 size = { w_max.x - w_min.x, w_max.y - w_min.y };

		float w_mid_x = w_min.x + size.x / 2;
		float w_mid_y = w_min.y + size.y / 2;

		AnimationHandler* m_anim_handler = m_selected->m_model->m_anim_handler;
		BlendNode2D* root = dynamic_cast<BlendNode2D*>(m_anim_handler->m_blend_tree);
		glm::vec2 max_pos = { 10,10 };

		for (auto& it : root->m_triangles) {
			//get position mapped to the window
			glm::vec2 p_0 = w_min + root->m_children[it[0]]->m_blend_pos * size / max_pos;
			glm::vec2 p_1 = w_min + root->m_children[it[1]]->m_blend_pos * size / max_pos;
			glm::vec2 p_2 = w_min + root->m_children[it[2]]->m_blend_pos * size / max_pos;

			draw_list->AddTriangleFilled(ImVec2(p_0.x, p_0.y), ImVec2(p_1.x, p_1.y), ImVec2(p_2.x, p_2.y), IM_COL32(220, 0, 220, 10));
			draw_list->AddTriangle(ImVec2(p_0.x, p_0.y), ImVec2(p_1.x, p_1.y), ImVec2(p_2.x, p_2.y), IM_COL32(0, 0, 0, 255), 2.f);
		}
		//draw points for position
		for (auto& it : root->m_children) {
			glm::vec2 pos_mapped = w_min + (it->m_blend_pos * size) / max_pos;
			draw_list->AddCircle(ImVec2(pos_mapped.x, pos_mapped.y), 3.5f, IM_COL32(0, 0, 0, 160), 10, 10);
		}

		//map and draw blend param
		glm::vec2 blend_mapped = w_min + (*root->m_blend_param * size) / max_pos;
		draw_list->AddCircle(ImVec2(blend_mapped.x, blend_mapped.y), 3.5f, IM_COL32(130, 90, 44, 220), 10, 10);

		draw_list->PopClipRect();
	}

	std::string Editor::get_blending_type_string() {
		AnimationHandler* m_anim_handler = m_selected->m_model->m_anim_handler;
		
		switch (m_anim_handler->e_blending_type) {
		case BLENDINGTYPE::_1D:
			return "1D";
			break;
		case BLENDINGTYPE::_2D:
			return "2D";
			break;
		}
	}

	void Editor::show_demo_1D() {
		bool open = true;
		Begin("Graduation night", &open, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

		Text("After 4 years of suffering in Digipen to get the degree finally you have achieved it, it is time to celebrate in the graduation!");
		Text("This is your evolution during the night");

		SliderFloat("Hour", &m_selected->m_model->m_anim_handler->m_blend_tree->m_blend_param->x, 0.f, 7.f);
		Text("OUUHH YEAHHHHHHHHHHHH, NICES MOVEESSSSSS!!!              Time to go home    UPPSSSS to much drinking");

		End();
	}

	void Editor::show_demo_2D() {
		AnimationHandler* m_anim_handler = m_selected->m_model->m_anim_handler;
		bool open = true;
		Begin("2d Example", &open, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);
		SliderFloat("Blend factor x", &m_anim_handler->m_blend_tree->m_blend_param->x, 0.f, 10.f);
		SliderFloat("Blend factor y", &m_anim_handler->m_blend_tree->m_blend_param->y, 0.f, 10.f);
		BeginChild("g", ImVec2(0, 0), true);
		EndChild();
		blending_2D_blendspace();


		End();
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
		}
	}

	void Editor::picking() {
		double xPos;		double yPos;
		int window_width;		int window_height;
		//get cursor postion and window scale
		glfwGetCursorPos(m_window->get_window(), &xPos, &yPos);
		glfwGetWindowSize(m_window->get_window(), &window_width, &window_height);
		//create viewport array
		glm::vec4 viewport(0, 0, window_width, window_height);
		//unproject cursos position
		glm::vec3 near_point = glm::unProject(glm::vec3(xPos, m_window->get_size().y - yPos, m_scene->m_renderer->m_camera->nearPlane), m_scene->m_renderer->m_camera->view_mtx,
			m_scene->m_renderer->m_camera->proj_mtx, viewport);
		glm::vec3 far_point = glm::unProject(glm::vec3(xPos, m_window->get_size().y - yPos, m_scene->m_renderer->m_camera->farPlane), m_scene->m_renderer->m_camera->view_mtx,
			m_scene->m_renderer->m_camera->proj_mtx, viewport);

		Geometry::Ray r(near_point, (far_point - near_point));

		const std::unordered_map<std::string, Node*>& all_nodes = m_scene->m_all_nodes;

		float t_min = 1000000.f; //very big number
		for (auto& it : m_scene->m_all_nodes) {
			float t = RayCast::intersection_ray_aabb(r, Geometry::AABB(glm::vec3(-2), glm::vec3(10)));

			if (t > 0.f && t < t_min) {
				t_min = t;
				m_selected = it.second;
			}
		}

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