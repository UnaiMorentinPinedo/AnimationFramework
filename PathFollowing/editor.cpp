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
#include "curves.hpp"

namespace cs460 {
	using namespace ImGui;
	Editor::Editor() : m_selected(nullptr), m_path("resources/models/"), m_curve(new CurveManager), m_active_curve("Hermite"){

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
		m_curve->Init();

		//create walker for the curve
		if (!Loader::load_model(m_walker_model, "resources/models/Fox/Fox.gltf"))
			return;
		m_walker = m_scene->spawn_model(m_scene->create_model_from_gltf(new Model_Data(m_walker_model), "Walker"))->m_model_root;
		m_selected = m_walker;
		m_walker->set_scale({ 0.025f,0.025f,0.025f});
		//hardcoded to walk animation
		m_walker->m_model->m_anim_handler->m_current = m_walker->m_model->m_anim_handler->m_animations[1];
		m_walker->m_model->m_anim_handler->m_current->Init();
	}


	void Editor::update() {
		new_frame();
		ImGui::PushStyleColor(ImGuiCol_::ImGuiCol_TitleBg, ImVec4(1.f, 0.7f, 0.f, 0.45f)); 
		ImGui::PushStyleColor(ImGuiCol_::ImGuiCol_TitleBgActive, ImVec4(1.f, 0.7f, 0.f, 0.45f));

		//update curve
		if (!m_table_computed) {
			m_curve->m_active_curve->ComputeTable();
			m_table_computed = true;
		}
		m_curve->update(m_walker, m_movement_control.m_travel_rate/4.5f, m_movement_control.m_using_const_speed);

		show_node_data();
		show_curve_objetcs();
		show_path_editor();
		curve_editor();

		//if (glfwGetMouseButton(m_window->get_window(), GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS)
		//	picking();
		if (m_selected) 
			updateGizmos();

		ImGui::PopStyleColor();
		ImGui::PopStyleColor();

		render();
	}

	void Editor::show_node_data() {
		if (!m_selected)
			return;

		ImGui::SetNextWindowBgAlpha(0.4f);//changes el fondo

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
				ImGui::MenuItem("Skin:");
				ImGui::Checkbox("Show skeleton", &m_selected->m_model->m_skin->m_show_skeleton);
			}
			ImGui::Separator();
			//ANIMATION
			if (m_selected->m_model && m_selected->m_model->m_anim_handler) {
				if (m_selected->m_model->m_anim_handler->m_animations.size() > 0) {
					ImGui::MenuItem("Animation:");
					if (ImGui::BeginCombo("Select animation:", m_selected->m_model->m_anim_handler->m_current ? 
						m_selected->m_model->m_anim_handler->m_current->m_name.c_str() : "")) {
						for (auto& it : m_selected->m_model->m_anim_handler->m_animations) {
							if (ImGui::Selectable(it->m_name.c_str())) {
								m_selected->m_model->m_anim_handler->m_current = it;
							}
						}
						ImGui::EndCombo();
					}

					if (m_selected->m_model->m_anim_handler->m_current) {
						ImGui::SliderFloat("t", &m_selected->m_model->m_anim_handler->m_current->m_timer->m_time, 0.f,
							m_selected->m_model->m_anim_handler->m_current->biggest_t);
						if (!m_selected->m_model->m_anim_handler->m_current->m_active) {
							if (ImGui::Button("PLAY")) {
								m_selected->m_model->m_anim_handler->m_current->Init();
							}
						}
						else {
							ImGui::Checkbox("Pause", &m_selected->m_model->m_anim_handler->m_current->m_pause);
							if (ImGui::Button("Reset"))
								m_selected->m_model->m_anim_handler->m_current->Reset();
						}

						ImGui::Checkbox("Loop", &m_selected->m_model->m_anim_handler->m_current->m_loop);
					}
				}
			}
		}
		ImGui::End();
	}


	void Editor::show_path_editor() {
		if (Begin("Path editor")) {
			if (Button("Restart pathing")) {
				if (m_movement_control.m_using_const_speed) {
					m_curve->m_active_curve->s_travelled = 0.f;
				}
				else
					m_curve->m_active_curve->m_timer->Init();
			}
			BeginChild("b", ImVec2(0, 0), true);
			ImGui::Checkbox("Using constant speed", &m_movement_control.m_using_const_speed);
			BeginChild("a", ImVec2(0, 0), true);
			if (m_movement_control.m_using_const_speed) {
				SliderFloat("Travel rate", &m_movement_control.m_travel_rate, 0.f, 4.f);
				/*You should be able to compute the total time it takes to get to the end of the path given the 
				speed and the arc length table. Display this value. */
				std::string msg = "Total time: " + std::to_string( m_curve->m_active_curve->m_table.m_table.back().second/
					m_movement_control.m_travel_rate);
				MenuItem(msg.c_str());
			}
			else {
				SliderFloat("Dist time", &m_curve->m_active_curve->m_total_dist_time, 0.5f, 30.f);
				SliderFloat("t1", &m_curve->m_active_curve->t1, 0.01f, m_curve->m_active_curve->t2);
				SliderFloat("t2", &m_curve->m_active_curve->t2, m_curve->m_active_curve->t1, 1.f);
				/*The rate of travel should be calculated each frame based on the difference between the
				distance parameter for this frame and the previous frame. This should reflect in the GUI.
				*/
				std::string msg = "Rate of travel: " + std::to_string(m_curve->m_active_curve->rate_travel);
				MenuItem(msg.c_str());
			}
			
			EndChild();


			EndChild();
		}

		ImGui::End();
	}

	void Editor::show_curve_objetcs() {
		if (Begin("Curve inspector")) {
			MenuItem("Points:");
			for (auto& it : m_curve->m_active_curve->m_nodes) {
				if (Selectable(it->get_name().c_str()))
					m_selected = it;
			}
			auto bez = dynamic_cast<BezierCurve*>(m_curve->m_active_curve);
			if (bez) {
				Separator();
				MenuItem("Tangents:");
				for (auto& it : bez->m_tangents) {
					if (it.t_0)
						if (Selectable(it.t_0->get_name().c_str()))
							m_selected = it.t_0;
					if (it.t_1)
						if (Selectable(it.t_1->get_name().c_str()))
							m_selected = it.t_1;
				}
			}
			auto her = dynamic_cast<HermiteCurve*>(m_curve->m_active_curve);
			if (her) {
				Separator();
				MenuItem("Tangents:");
				for (auto& it : her->m_tangents) {
					if (it.t_0)
						if (Selectable(it.t_0->get_name().c_str()))
							m_selected = it.t_0;
					if (it.t_1)
						if (Selectable(it.t_1->get_name().c_str()))
							m_selected = it.t_1;
				}
			}

		}
		ImGui::End();
	}

	void Editor::curve_editor() {
		if (ImGui::Begin("Curve Editor")) {
			if (ImGui::BeginCombo("Change type:", m_active_curve.c_str())) {
				if (ImGui::Selectable("Hermite")) {
					m_active_curve = "Hermite";
					m_curve->m_active_curve = m_curve->m_hermite;
					m_curve->m_active_curve->ComputeTable();
					if (m_movement_control.m_using_const_speed) {
						m_curve->m_active_curve->s_travelled = 0.f;
					}
					else
						m_curve->m_active_curve->m_timer->Init();
				}
				else if (ImGui::Selectable("Bezier")) {
					m_active_curve = "Bezier";
					m_curve->m_active_curve = m_curve->m_bezier;
					m_curve->m_active_curve->ComputeTable();
					if (m_movement_control.m_using_const_speed) {
						m_curve->m_active_curve->s_travelled = 0.f;
					}
					else
						m_curve->m_active_curve->m_timer->Init();
				}
				else if (ImGui::Selectable("Catmull")) {
					m_active_curve = "Catmull";
					m_curve->m_active_curve = m_curve->m_catmull;
					m_curve->m_active_curve->ComputeTable();
					if (m_movement_control.m_using_const_speed) {
						m_curve->m_active_curve->s_travelled = 0.f;
					}
					else
						m_curve->m_active_curve->m_timer->Init();

				}
				ImGui::EndCombo();
			}

			if (ImGui::Button("Recompute table"))
				m_curve->m_active_curve->ComputeTable();
			if(Checkbox("Adaptative", &m_curve->m_active_curve->m_table.m_adaptative_mode))
				m_curve->m_active_curve->ComputeTable();
			if (m_curve->m_active_curve->m_table.m_adaptative_mode){
				SliderFloat("Tolerance value", &m_curve->m_active_curve->m_table.m_tolerance, 0.f, 3.f);
				SliderInt("Forced subdivisions", &m_curve->m_active_curve->m_table.m_force_subdivisions, 0, 10);
			}
			else
				SliderFloat("Step value", &m_curve->m_active_curve->m_table.m_step, 0.01, 0.05f);

			if (BeginChild("TABLE", ImVec2(0.f, 0.f), true)) {
				ImGui::BeginTable("Curve table", 3);
				TableNextRow();		TableSetColumnIndex(0);		Text("Index");		TableSetColumnIndex(1);		Text("Param Value");	TableSetColumnIndex(2);		Text("Arc Length");
				unsigned int i = 0;
				for (auto& it : m_curve->m_active_curve->m_table.m_table) {
					TableNextRow();	TableNextColumn(); Text(std::to_string(i).c_str());	TableNextColumn();	Text(std::to_string(it.first).c_str());
					TableNextColumn();	Text(std::to_string(it.second).c_str());
					i++;
				}
				ImGui::EndTable();
			}

			EndChild();
		}
		ImGui::End();
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
		ImVec2 mouse_pos = ImGui::GetIO().MousePos;

		//NDC coordinates
		float x = (2 * mouse_pos.x) / ImGui::GetWindowSize().x - 1.f;
		float y = -1.f + (2 * mouse_pos.y) / ImGui::GetWindowSize().y;

		auto& view = m_scene->m_renderer->m_camera->view_mtx;
		auto& proj = m_scene->m_renderer->m_camera->proj_mtx;

		// unporject point
		glm::mat4 invVP = glm::inverse(proj * view);
		glm::vec4 screenPos = glm::vec4(x, -y, 1.0f, 1.0f);
		glm::vec4 worldPos = invVP * screenPos;

		glm::vec3 dir = glm::normalize(glm::vec3(worldPos));

		//set ray
		Geometry::Ray r;
		r.start = m_scene->m_renderer->m_camera->Position;
		r.dir = dir;

		float t_min = std::numeric_limits<float>().max();
		for (auto& it : m_scene->m_all_nodes) {
			float t = RayCast::intersection_ray_aabb(r, Geometry::AABB(it.second->get_world().m_position - glm::vec3(2.f), it.second->get_world().m_position + glm::vec3(2.f)));

			if (t <= 0.f) continue;

			if (t < t_min) {
				m_selected = it.second;
				t_min = t;
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