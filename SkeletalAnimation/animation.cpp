/*-------------------------------------------------------
File Name: animation.cpp
Author: Unai Morentin
---------------------------------------------------------*/ 

#include "animation.hpp"
#include "node.hpp"
#include "model.hpp"

namespace cs460 {

	Channel_Sampler::Channel_Sampler(Datas::Data& data, Model* model)
	: input(data.input)
	{
		target = model->m_nodes.find(data.target)->second;
		if (data.path.compare("translation") == 0) {
			m_path = PATH::TRANSLATION;
			output_vec3 = data.output_vec3;
		}
		else if (data.path.compare("rotation") == 0) {
			m_path = PATH::ROTATION;
			output_quat = data.output_vec4;
		}
	}

	Animation::Animation(Datas* data, Model* model, std::string name) : m_timer(new Timer), m_name(name), biggest_t(data->bigges_t) {
		for (auto& it : data->m_datas)
			m_channels.push_back(new Channel_Sampler(it, model));
	}

	Animation::~Animation() {
		for (auto& it : m_channels)
			delete it;
	}

	void Channel_Sampler::Init() {
		m_finished = false;
	}

	void Channel_Sampler::tick(float t) {
		if (m_path == PATH::TRANSLATION) {
			if (t < input[0]) {
				target->set_position(output_vec3[0]);
				return;
			}
			if (t > input[output_vec3.size() - 1]) {
				target->set_position(output_vec3[output_vec3.size() - 1]);
				m_finished = true;
				return;
			}

			unsigned int idx = 1;
			while (t > input[idx])
				idx++;

			//find the segment to interpolate
			glm::vec3 p0 = output_vec3[idx - 1];
			glm::vec3 p1 = output_vec3[idx];

			//normalize the time
			float interval_dur = input[idx] - input[idx - 1];
			float local_time = t - input[idx - 1];

			target->set_position(p0 + (p1 - p0) * (local_time / interval_dur));
			return;
		}
		else if (m_path == PATH::ROTATION) {
			if (t < input[0]) {
				target->set_rotation(output_quat[0]);
				return;
			}
			if (t > input[output_quat.size() - 1]) {
				target->set_rotation(output_quat[output_quat.size() - 1]);
				m_finished = true;
				return;
			}

			unsigned int idx = 1;
			while (t > input[idx])
				idx++;

			//find the segment to interpolate
			glm::quat p0 = output_quat[idx - 1];
			glm::quat p1 = output_quat[idx];

			//normalize the time
			float interval_dur = input[idx] - input[idx - 1];
			float local_time = t - input[idx - 1];

			target->set_rotation(glm::slerp(p0, p1, local_time/interval_dur));
			return;
		}
	}

	void Channel_Sampler::End() {
		m_finished = true;
	}

	void Animation::Init() {
		m_timer->Init();
		m_active = true;
		for (auto& it : m_channels)
			it->Init();
	}

	void Animation::tick() {
		//update time
		m_timer->update(m_pause);
		// update animations, if all are finished call AnimationHandler::End
		bool all_finished = true;
		for (auto& it : m_channels)
			if (!it->m_finished)
			{
				it->tick(m_timer->m_time);
				all_finished = false;
			}

		if(all_finished)
			End();
	}

	void Animation::End() {
		m_active = false;
		if (m_loop)
			Init();
	}

	void Animation::Reset() {
		for (auto& it : m_channels)
			it->End();

		Init();
	}

	AnimationHandler::AnimationHandler(std::vector<Datas*> all_model_anim_data, Model* model) {
		for (auto& it : all_model_anim_data)
			m_animations.push_back(new Animation(it, model, it->m_name));

		if (m_animations.size() > 0)
			m_current = m_animations[0];
	}

	AnimationHandler::~AnimationHandler() {
		for (auto& it : m_animations)
			delete it;
	}
}