/*-------------------------------------------------------
File Name: curves.hpp
Author: Unai Morentin
---------------------------------------------------------*/

#pragma once
#include <vector>
#include <unordered_map>
#include "math.hpp"

namespace cs460 {

	class Node;
	struct Timer;

	struct Tangent {
		Node* t_0 = nullptr;
		Node* t_1 = nullptr;

		Node* get_left() {
			if (t_0 != nullptr)
				return t_0;

			return t_1;
		}
		Node* get_right() {
			if (t_1 != nullptr)
				return t_1;

			return t_0;
		}
	};

	//pure virtual
	struct Curve {
		virtual void Init() = 0;
		virtual void Render() = 0;
		virtual void End() = 0;

		virtual void ComputeTable();
		virtual float GetParamValue(float dist);
		virtual std::pair<float, float> GetArcLengthValue(float u);
		virtual float compute_arc_length(float u);
		virtual void update_walker_const_speed(Node* walker, float s);
		virtual void update_walker_distance_fn(Node* walker);
		float get_resulting_function(float t);
		void recursive_adaptative(std::list<std::pair<float, float>>& list, float tolerance, int level, int subdivisions,
			std::list < std::pair<float, float>>::iterator left, std::list < std::pair<float, float>>::iterator right);

		virtual glm::vec3 compute_pos(float t) = 0;

		struct {
			std::vector<std::pair<float, float>> m_table;
			int m_samples;;
			float m_step = 0.05f;
			bool m_adaptative_mode = false;
			float m_tolerance = 1;
			int m_force_subdivisions = 5;
		} m_table;


		bool m_loop = true;
		bool m_finished = false;
		bool m_pause = true;
		float s_travelled = 0.f;
		float m_total_dist_time = 15.f;
		float t1 = 0.3f;
		float t2 = 0.57f;
		float prev_dist = 0.f;
		float rate_travel = 0.f;
		float m_reference_rate = 0.35f;

		Timer* m_timer;
		Node* m_parent = nullptr;

		std::vector<Node*> m_nodes;
		std::vector<float> m_keys;
	};

	struct HermiteCurve : public Curve {
		HermiteCurve();
		~HermiteCurve();

		virtual void Init();
		virtual void Render();
		virtual void End();

		virtual glm::vec3 compute_pos(float t);
	
		std::vector<Tangent> m_tangents;
	};

	struct BezierCurve : public Curve {
		BezierCurve();
		~BezierCurve();

		virtual void Init();
		virtual void Render();
		virtual void End();

		virtual glm::vec3 compute_pos(float t);

		std::vector<Tangent> m_tangents;
	};
	
	struct CatmullCurve : public Curve {
		CatmullCurve();
		~CatmullCurve();

		virtual void Init();
		virtual void Render();
		virtual void End();

		virtual glm::vec3 compute_pos(float t);
	};

	struct CurveManager {
		CurveManager();
		~CurveManager();

		Curve* m_active_curve;
		HermiteCurve* m_hermite;
		BezierCurve* m_bezier;
		CatmullCurve* m_catmull;

		void delete_all_curves();
		void update(Node* walker, float s, bool const_speed);
		Node* create_hermite_curve();
		void Init();
		Node* create_bezier_curve();
		Node* create_catmull_curve();
	};
}