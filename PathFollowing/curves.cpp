/*-------------------------------------------------------
File Name: curves.cpp
Author: Unai Morentin
---------------------------------------------------------*/

#include "curves.hpp"
#include "scene.hpp"
#include "renderer.hpp"
#include "node.hpp"
#include "model.hpp"


namespace cs460 {
	void Curve::update_walker_const_speed(Node* walker, float s) {
		walker->m_model->m_anim_handler->m_current->m_speed = s / m_reference_rate;
		s_travelled += s * m_timer->dt;
		float u = GetParamValue(s_travelled);
		glm::vec3 pos = compute_pos(u);
		if (s_travelled > m_table.m_table.back().second)
			End();

		walker->set_position(pos);
	}

	void Curve::update_walker_distance_fn(Node* walker) {
		float norm_t = m_timer->m_time / m_total_dist_time;
		float dist = get_resulting_function(norm_t) * m_table.m_table.back().second;
		//in order to compute the rate of traver
		rate_travel = (dist - prev_dist)/m_timer->dt;
		prev_dist = dist;

		walker->m_model->m_anim_handler->m_current->m_speed = std::abs(rate_travel) / (m_reference_rate * 1.7f);

		float u = GetParamValue(dist);

		if (dist > m_table.m_table.back().second)
			End();

		walker->set_position(compute_pos(u));
	}

	float Curve::get_resulting_function(float t) {
		float result = -1.f;
		float pi = glm::pi<float>();

		if (t < t1)
			result = t1 * (2.f / pi) * (glm::sin((t / t1) * (pi / 2) - pi / 2) + 1);
		else if (t < t2)
			result = (t - t1) + (2 * t1) / pi;
		else
			result = (1 - t2) * (2 / pi) * glm::sin(((t - t2) / (1 - t2)) * (pi / 2)) +
			(2 * t1) / pi + (t2 - t1);

		return result / ((2 * t1) / pi + t2 - t1 + (2 * (1 - t2)) / pi);
	}

	void Curve::ComputeTable() {
		m_table.m_samples = 1.f / m_table.m_step;
		m_table.m_table.clear();
		float u = m_table.m_step;

		if (m_table.m_adaptative_mode) {
			std::list <std::pair<float, float>> table_list;
			table_list.push_back({ 0.f, 0.f });
			table_list.push_back({ 1.f, 0.f });

			recursive_adaptative(table_list, m_table.m_tolerance, 0, m_table.m_force_subdivisions, table_list.begin(), --table_list.end());
			m_table.m_table.resize(table_list.size());
			std::copy(table_list.begin(), table_list.end(), m_table.m_table.begin());
		}
		else {
			m_table.m_table.push_back({ 0.f, 0.f });
			for (unsigned int i = 1; i <= m_table.m_samples; i++) {
				m_table.m_table.push_back({ u, compute_arc_length(u) + m_table.m_table[i - 1].second });
				u += m_table.m_step;
			}
		}
		return;
	}

	void Curve::recursive_adaptative(std::list<std::pair<float, float>>& list, float tolerance, int level, int subdivisions,
		std::list < std::pair<float, float>>::iterator left, std::list < std::pair<float, float>>::iterator right) {
		auto left_p = compute_pos(left->first);
		auto right_p = compute_pos(right->first);
		auto mid_u = left->first + (right->first - left->first) / 2;
		auto mid_p = compute_pos(mid_u);

		//compute vectors
		float C = glm::length(left_p - right_p);
		float A = glm::length(left_p - mid_p);
		float B = glm::length(mid_p - right_p);

		//insert middle point
		auto it = left;
		auto inserted_it = list.insert(++it, { mid_u , left->second + A });

		//forced to subidivide or not within toelrance
		if (subdivisions-- > 0 || std::abs(A + B - C) > tolerance) {
			recursive_adaptative(list, tolerance, level, subdivisions, left, inserted_it);
			right->second = inserted_it->second + B;
			recursive_adaptative(list, tolerance, level, subdivisions, inserted_it, right);
		}
	}

	float Curve::compute_arc_length(float u) {
		glm::vec3 pos0 = compute_pos(u - m_table.m_step);
		glm::vec3 pos1 = compute_pos(u);

		return glm::length(pos1 - pos0);
	}

	std::pair<float, float> Curve::GetArcLengthValue(float u) {
		int idx = std::floor(u / m_table.m_step + 0.5);

		return m_table.m_table[idx];
	}

	float Curve::GetParamValue(float dist) {
		int i = 1;
		while (dist < m_table.m_table[i].second) {
			if (++i >= m_table.m_table.size())
				return 0.f;
		}
		float local_dist = dist - m_table.m_table[i - 1].second;
		float total_dist = m_table.m_table[i].second - m_table.m_table[i - 1].second;
		auto debug = m_table.m_table[i - 1].first + (m_table.m_table[i].first - m_table.m_table[i - 1].first) * (local_dist / total_dist);
		if (debug < 0)
			debug = debug;
		return  m_table.m_table[i - 1].first + (m_table.m_table[i].first - m_table.m_table[i - 1].first) * (local_dist / total_dist);
	}


#pragma region HermiteCurves
	HermiteCurve::HermiteCurve(){
		m_timer = new Timer;
	}

	HermiteCurve::~HermiteCurve() {
		for (auto& it : m_nodes)
			delete it;

		for (auto& it : m_tangents) {
			delete it.t_0;
			delete it.t_1;
		}

		delete m_parent;
		delete m_timer;
	}

	void HermiteCurve::Init() {
		m_timer->Init();
		m_finished = false;
		m_pause = false;
		ComputeTable();
	}

	void HermiteCurve::Render() {
		//update timer
		m_timer->update(m_pause);

		//render lines and points
		for (size_t i = 0; i < m_nodes.size(); i++) {
			glm::vec3 pos = m_nodes[i]->get_world().m_position;
			Scene::get_instance().m_renderer->debug_draw_point(pos);
			if (m_tangents[i].t_0) {
				Scene::get_instance().m_renderer->debug_draw_line(pos, m_tangents[i].t_0->get_world().m_position, glm::vec4(1.f, 1.f, 1.f, 0.33f));
				Scene::get_instance().m_renderer->debug_draw_point(m_tangents[i].t_0->get_world().m_position, glm::vec4(1.f,1.f,0.9f,0.7f));
			}
			if (m_tangents[i].t_1) {
				Scene::get_instance().m_renderer->debug_draw_line(pos, m_tangents[i].t_1->get_world().m_position, glm::vec4(1.f, 1.f, 1.f, 0.33f));
				Scene::get_instance().m_renderer->debug_draw_point(m_tangents[i].t_1->get_world().m_position, glm::vec4(1.f, 1.f, 0.9f, 0.7f));
			}
		}
		//draw curve
		glm::vec3 prev_pos = compute_pos(0.f);
		for (float step = 0.05f; step < m_keys.back(); step += 0.05f) {
			glm::vec3 curr_pos = compute_pos(step);
			Scene::get_instance().m_renderer->debug_draw_line(curr_pos, prev_pos);
			prev_pos = curr_pos;
		}
	}

	glm::vec3 HermiteCurve::compute_pos(float t) {
		if (t < m_keys[0])
			return m_nodes[0]->get_world().m_position;
		if (t > m_keys[m_nodes.size() - 1]) {
			return m_nodes[m_nodes.size() - 1]->get_world().m_position;
		}

		unsigned int idx = 1;
		while (t > m_keys[idx])
			idx++;

		//find the segment to interpolate
		glm::vec3& p0 = m_nodes[idx - 1]->get_world().m_position;
		glm::vec3& p1 = m_nodes[idx]->get_world().m_position;
		glm::vec3& t0 = m_tangents[idx - 1].get_right()->get_world().m_position;
		glm::vec3& t1 = m_tangents[idx].get_left()->get_world().m_position;

		//normalize the time
		float interval_dur = m_keys[idx] - m_keys[idx - 1];
		float local_time = t - m_keys[idx - 1];

		return glm::hermite(p0, t0 - p0, p1, t1  - p1, local_time / interval_dur);
	}

	void HermiteCurve::End() {
		m_pause = true;
		s_travelled = 0.f;
		if (m_loop)
			Init();
	}

#pragma endregion HermiteCurves

#pragma region BezierCurves
	BezierCurve::BezierCurve() {
		m_timer = new Timer;
	}

	BezierCurve::~BezierCurve() {
		for (auto& it : m_nodes)
			delete it;

		for (auto& it : m_tangents) {
			delete it.t_0;
			delete it.t_1;
		}

		delete m_parent;
		delete m_timer;
	}

	void BezierCurve::Init() {
		m_timer->Init();
		m_finished = false;
		m_pause = false;
		ComputeTable();
	}

	void BezierCurve::Render() {
		//update timer
		m_timer->update(m_pause);

		//render lines and points
		for (size_t i = 0; i < m_nodes.size(); i++) {
			glm::vec3 pos = m_nodes[i]->get_world().m_position;
			Scene::get_instance().m_renderer->debug_draw_point(pos);
			if (m_tangents[i].t_0) {
				Scene::get_instance().m_renderer->debug_draw_line(pos, m_tangents[i].t_0->get_world().m_position, glm::vec4(1.f, 1.f, 1.f, 0.33f));
				Scene::get_instance().m_renderer->debug_draw_point(m_tangents[i].t_0->get_world().m_position, glm::vec4(1.f, 1.f, 0.9f, 0.7f));
			}
			if (m_tangents[i].t_1) {
				Scene::get_instance().m_renderer->debug_draw_line(pos, m_tangents[i].t_1->get_world().m_position, glm::vec4(1.f, 1.f, 1.f, 0.33f));
				Scene::get_instance().m_renderer->debug_draw_point(m_tangents[i].t_1->get_world().m_position, glm::vec4(1.f, 1.f, 0.9f, 0.7f));
			}
		}

		//draw curve
		glm::vec3 prev_pos = compute_pos(0.f);
		for (float step = 0.05f; step < m_keys.back(); step += 0.05f) {
			glm::vec3 curr_pos = compute_pos(step);
			Scene::get_instance().m_renderer->debug_draw_line(curr_pos, prev_pos);
			prev_pos = curr_pos;
		}
	}

	glm::vec3 BezierCurve::compute_pos(float t) {
		if (t < m_keys[0])
			return m_nodes[0]->get_world().m_position;
		if (t > m_keys[m_nodes.size() - 1]) {
			return m_nodes[m_nodes.size() - 1]->get_world().m_position;
		}

		unsigned int idx = 1;
		while (t > m_keys[idx])
			idx++;

		//find the segment to interpolate
		glm::vec3& p0 = m_nodes[idx - 1]->get_world().m_position;
		glm::vec3& p1 = m_nodes[idx]->get_world().m_position;
		glm::vec3& t0 = m_tangents[idx - 1].get_right()->get_world().m_position;
		glm::vec3& t1 = m_tangents[idx].get_left()->get_world().m_position;

		//normalize the time
		float interval_dur = m_keys[idx] - m_keys[idx - 1];
		float local_time = t - m_keys[idx - 1];
		float tn = local_time / interval_dur;

		return (1 - tn) * (1 - tn) * (1 - tn) * p0 + 3.0f * tn * ((1 - tn) * (1 - tn)) * t0
			+ 3.0f * (tn * tn) * (1 - tn) * t1 + (tn * tn * tn) * p1;
	}

	void BezierCurve::End() {
		m_pause = true;
		s_travelled = 0.f;
		if (m_loop)
			Init();
	}
#pragma endregion BezierCurves

#pragma region CatmullCurves
	CatmullCurve::CatmullCurve()  {
		m_timer = new Timer();
	}

	CatmullCurve::~CatmullCurve() {
		for (auto& it : m_nodes)
			delete it;

		delete m_parent;
		delete m_timer;
	}

	void CatmullCurve::Init() {
		m_timer->Init();
		m_finished = false;
		m_pause = false;
		ComputeTable();
	}

	void CatmullCurve::Render() {
		//update timer
		m_timer->update(m_pause);

		//render lines and points
		for (size_t i = 0; i < m_nodes.size(); i++) {
			glm::vec3 pos = m_nodes[i]->get_world().m_position;
			Scene::get_instance().m_renderer->debug_draw_point(pos);
		}

		//draw curve
		glm::vec3 prev_pos = compute_pos(0.f);
		for (float step = 0.05f; step < m_keys.back(); step += 0.05f) {
			glm::vec3 curr_pos = compute_pos(step);
			Scene::get_instance().m_renderer->debug_draw_line(curr_pos, prev_pos);
			prev_pos = curr_pos;
		}
	}

	glm::vec3 CatmullCurve::compute_pos(float t) {
		if (t < m_keys[0])
			return m_nodes[0]->get_world().m_position;
		if (t > m_keys[m_nodes.size() - 1]) {
			return m_nodes[m_nodes.size() - 1]->get_world().m_position;
		}

		unsigned int idx = 1;
		while (t > m_keys[idx])
			idx++;

		//normalize the time
		float interval_dur = m_keys[idx] - m_keys[idx - 1];
		float local_time = t - m_keys[idx - 1];

		if (idx == 1) {
			glm::vec3& p0 = m_nodes[idx - 1]->get_world().m_position;
			glm::vec3& p1 = m_nodes[idx]->get_world().m_position;
			glm::vec3& p2 = m_nodes[idx + 1]->get_world().m_position;


			return glm::catmullRom(p0, p0, p1, p2, local_time / interval_dur);
		}


		if (idx == m_nodes.size() - 1) {
			glm::vec3& p0 = m_nodes[idx - 2]->get_world().m_position;
			glm::vec3& p1 = m_nodes[idx - 1]->get_world().m_position;
			glm::vec3& p2 = m_nodes[idx]->get_world().m_position;

			return glm::catmullRom(p0, p1, p2, p2, local_time / interval_dur);
		}

		glm::vec3& p0 = m_nodes[idx - 2]->get_world().m_position;
		glm::vec3& p1 = m_nodes[idx - 1]->get_world().m_position;
		glm::vec3& p2 = m_nodes[idx]->get_world().m_position;
		glm::vec3& p3 = m_nodes[idx + 1]->get_world().m_position;

		return glm::catmullRom(p0, p1, p2, p3, local_time / interval_dur);
	}

	void CatmullCurve::End() {
		m_pause = true;
		s_travelled = 0.f;
		if (m_loop)
			Init();
	}

#pragma endregion CatmullCurves

	CurveManager::CurveManager() {
	}

	CurveManager::~CurveManager() {
		delete m_hermite;
		delete m_catmull;
		delete m_bezier;
	}

	void CurveManager::Init() {
		m_hermite = new HermiteCurve;
		m_catmull = new CatmullCurve;
		m_bezier = new BezierCurve;

		create_bezier_curve();
		create_catmull_curve();
		create_hermite_curve();

		m_active_curve = m_hermite;
		m_hermite->Init();
	}

	void CurveManager::update(Node* walker, float s, bool const_speed) {
		m_active_curve->Render();
		if (const_speed)
			m_active_curve->update_walker_const_speed(walker, s);
		else
			m_active_curve->update_walker_distance_fn(walker);
	}

	Node* CurveManager::create_hermite_curve() {
		m_hermite->m_parent = new Node("hermite curve", -1, Scene::get_instance().m_root);

		for (int i = 0; i < 4; i++) {
			Node* new_node = new Node("hermite curve: " + std::to_string(i), -1, m_hermite->m_parent);
			Tangent tangent;
			tangent.t_0 = new Node("hermite curve: tang0: " + std::to_string(i), -1, m_hermite->m_parent);
			if (i > 0 && i < 3) {
				tangent.t_1 = new Node("hermite curve: tang1: " + std::to_string(i), -1, m_hermite->m_parent);
			}
			m_hermite->m_tangents.push_back(tangent);
			m_hermite->m_nodes.push_back(new_node);
		}

		//set intiaal values
		m_hermite->m_nodes[0]->set_position({ -10.f, -2.f, -10.f });
		m_hermite->m_tangents[0].t_0->set_position({ -12.f, -2.f, -11.f });
		m_hermite->m_keys.push_back(0.f);
		m_hermite->m_nodes[1]->set_position({ -9.f, -2.f, -7.f });
		m_hermite->m_tangents[1].t_0->set_position({ -11.f, -2.f, -8.5f });
		m_hermite->m_tangents[1].t_1->set_position({ -7.f, -2.f, -8.5f });
		m_hermite->m_keys.push_back(0.33f);
		m_hermite->m_nodes[2]->set_position({ -6.f, -2.f, -6.f });
		m_hermite->m_tangents[2].t_0->set_position({-7.f, -2.f, -7.5f});
		m_hermite->m_tangents[2].t_1->set_position({ -3.f, -2.f, -7.5f});
		m_hermite->m_keys.push_back(0.7f);
		m_hermite->m_nodes[3]->set_position({ -2.f, -2.f, -4.f });
		m_hermite->m_tangents[3].t_0->set_position({ -2.5f, -2.f, -5.5f });
		m_hermite->m_keys.push_back(1.f);

		Scene::get_instance().insert_node(m_hermite->m_parent);
		return m_hermite->m_parent;
	}

	Node* CurveManager::create_bezier_curve() {
		m_bezier->m_parent = new Node("bezier curve", -1, Scene::get_instance().m_root);

		for (int i = 0; i < 4; i++) {
			Node* new_node = new Node("bezier curve : " + std::to_string(i), -1, m_bezier->m_parent);
			Tangent tangent;
			tangent.t_0 = new Node("bezier curve: control_p0: " + std::to_string(i), -1, m_bezier->m_parent);
			if (i > 0 && i < 3) {
				tangent.t_1 = new Node("bezier curve: control_p1: " + std::to_string(i), -1, m_bezier->m_parent);
			}
			m_bezier->m_tangents.push_back(tangent);
			m_bezier->m_nodes.push_back(new_node);
			new_node->m_active = false;
		}

		//set intiaal values
		m_bezier->m_nodes[0]->set_position({ -10.f, -2.f, -10.f });
		m_bezier->m_tangents[0].t_0->set_position({ -12.f, -2.f, -11.f });
		m_bezier->m_keys.push_back(0.f);
		m_bezier->m_nodes[1]->set_position({ -9.f, -2.f, -7.f });
		m_bezier->m_tangents[1].t_0->set_position({ -11.f, -2.f, -8.5f });
		m_bezier->m_tangents[1].t_1->set_position({ -7.f, -2.f, -8.5f });
		m_bezier->m_keys.push_back(0.33f);
		m_bezier->m_nodes[2]->set_position({ -6.f, -2.f, -6.f });
		m_bezier->m_tangents[2].t_0->set_position({ -7.f, -2.f, -7.5f });
		m_bezier->m_tangents[2].t_1->set_position({ -3.f, -2.f, -7.5f });
		m_bezier->m_keys.push_back(0.7f);
		m_bezier->m_nodes[3]->set_position({ -2.f, -2.f, -4.f });
		m_bezier->m_tangents[3].t_0->set_position({ -2.5f, -2.f, -5.5f });
		m_bezier->m_keys.push_back(1.f);

		m_bezier->Init();

		return m_bezier->m_parent;
	}

	Node* CurveManager::create_catmull_curve() {
		m_catmull->m_parent = new Node("catmull curve", -1, Scene::get_instance().m_root);

		for (int i = 0; i < 8; i++) {
			Node* new_node = new Node("catmull curve: " + std::to_string(i), -1, m_catmull->m_parent);
			m_catmull->m_nodes.push_back(new_node);
			new_node->m_active = false;
		}

		//set intiaal values
		//set intiaal values
		m_catmull->m_nodes[0]->set_position({ -11.f, -2.f, -9.f });
		m_catmull->m_keys.push_back(0.f);
		m_catmull->m_nodes[1]->set_position({ -10.f, -2.f, -8.f });
		m_catmull->m_keys.push_back(0.15f);
		m_catmull->m_nodes[2]->set_position({ -9.f, -2.f, -7.f });
		m_catmull->m_keys.push_back(0.33f);
		m_catmull->m_nodes[3]->set_position({ -8.f, -2.f, -6.f });
		m_catmull->m_keys.push_back(0.45f);
		m_catmull->m_nodes[4]->set_position({ -7.f, -2.f, -5. });
		m_catmull->m_keys.push_back(0.59f);
		m_catmull->m_nodes[5]->set_position({ -6.f, -2.f, -4.f });
		m_catmull->m_keys.push_back(0.7f);
		m_catmull->m_nodes[6]->set_position({ -5.f, -2.f, -3.f });
		m_catmull->m_keys.push_back(0.83f);
		m_catmull->m_nodes[7]->set_position({ -4.f, -2.f, -2.f });
		m_catmull->m_keys.push_back(1.f);

		m_catmull->Init();

		return m_catmull->m_parent;
	}


	void CurveManager::delete_all_curves() {
		delete m_bezier;
		delete m_hermite;
		delete m_catmull;
	}
}