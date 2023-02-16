/*-------------------------------------------------------
File Name: editor.hpp
Author: Unai Morentin
---------------------------------------------------------*/

#pragma once

#include "../include/imgui/imgui.h"
#include "../include/imgui/imgui_impl_glfw.h"
#include "../include/imgui/imgui_impl_opengl3.h"
#include "../include/tinygltf/tiny_gltf.h"
#include "../include/imgui/ImGuizmo.h"
#include <string>

namespace cs460 {

	class Node;
	class Scene;
	class Window;
	class Editor {
		Scene* m_scene = nullptr;
		Window* m_window = nullptr;
		float m_window_x;
		float m_window_y;
	public:
		std::string m_path;
		Node* m_selected = nullptr;
		Editor();
		~Editor();

		void create(GLFWwindow* _window);
		void update();
		void render();
		void destroy();
		void updateGizmos();

		ImGuizmo::OPERATION m_guizmo_op = ImGuizmo::OPERATION::TRANSLATE;
	private:
		void new_frame();
		void compute_target();
		void show_node_data();
		void show_nodes_list();
		void show_node_tree(Node* _node, int& counter);
		void create_model_instances();
		void two_bone_IK_GUI();
		void IK_GUI();

	};
}