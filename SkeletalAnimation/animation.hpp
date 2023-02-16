/*-------------------------------------------------------
File Name: animation.hpp
Author: Unai Morentin
---------------------------------------------------------*/

#pragma once
#include <unordered_map>
#include <vector>

#include "../include/tinygltf/tiny_gltf.h"
#include "timer.hpp"


namespace cs460 {
	class Node;
	struct Model;

	struct Datas {
		struct Data {
			//channel
			int target; //maybee more than 1 target
			std::string path;
			//sampler
			//input
			std::vector<float> input; //key frames
			//output
			std::vector<glm::vec3> output_vec3;
			std::vector<glm::quat> output_vec4;
		};
		std::string m_name;
		std::vector<Data> m_datas;
		float bigges_t = -1.f;
	};

	struct Channel_Sampler {
		Node* target; //more than 1?
		enum class PATH {
			TRANSLATION,
			ROTATION,
			OTHER,
		} m_path = PATH::OTHER;

		//data
		std::vector<float> input; //key frames
		std::vector<glm::vec3> output_vec3;
		std::vector<glm::quat> output_quat;

		Channel_Sampler(Datas::Data& data, Model* model);
		void Init();
		void tick(float t);
		void End();
		bool m_finished = false;
		float biggest_t;
	};

	// an spawned model contains an animation handler which contains differnetn animations
	struct Animation {
		Animation(Datas* data, Model* model, std::string name);
		~Animation();

		void Init();
		void tick();
		void End();
		void Reset();
		std::vector<Channel_Sampler*> m_channels;
		Timer* m_timer;
		bool m_active = false;

		float biggest_t;
		bool m_pause = false;
		bool m_loop = true;
		std::string m_name;
	};

	struct AnimationHandler {
		AnimationHandler(std::vector<Datas*> all_model_anim_data, Model* model);
		~AnimationHandler();
		Animation* m_current;
		std::vector<Animation*> m_animations;
	};
}