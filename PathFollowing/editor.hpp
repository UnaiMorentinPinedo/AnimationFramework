/*-------------------------------------------------------
File Name: curves.hpp
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
	struct CurveManager;
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
		CurveManager* m_curve;
		std::string m_active_curve;
		Node* m_walker;

		struct {
			float m_travel_rate = 1.f;
			bool m_using_const_speed = true;
		} m_movement_control;

	private:
		void new_frame();
		void picking();
		void show_node_data();
		void show_curve_objetcs();
		void show_path_editor();
		void curve_editor();

		bool m_table_computed = false;
		//model intances
		tinygltf::Model m_walker_model;
	};
}