/*-------------------------------------------------------
File Name: blending.cpp
Author: Unai Morentin
---------------------------------------------------------*/


#include "blending.hpp"
#include "node.hpp"
#include "delaunator.hpp"

namespace cs460 {
	BlendNode::BlendNode() :m_blend_param(new glm::vec2(0.f)){

	}

	BlendNode::~BlendNode() {
		if (!m_children.empty())
			delete m_blend_param;
		for (auto& it : m_children)
			delete it;
	}

	void BlendNode::ProducePose(float t) {
		//clear pose vector each frame
		m_current_pose.clear();
		if (m_children.empty()) {
			if (m_animation)
				m_animation->ProducePose(t, m_current_pose);
		}
		else if (m_children.size() == 1)
			m_children[0]->m_animation->ProducePose(t, m_current_pose);
		else
			BlendChildren(t);
	}

	bool BlendNode1D::FindSegment(float param, BlendNode*& n0, BlendNode*& n1) {
		int idx = 0;

		//check smaller than first or bigger than last
		if (param <= m_children[0]->m_blend_pos.x || param > m_children[m_children.size() - 1]->m_blend_pos.x) {
			return false;
		}


		while (m_children[idx]->m_blend_pos.x < param) {
			if (idx == m_children.size()-1) {
				n0 = m_children[m_children.size() - 2];
				n1 = m_children[m_children.size() - 1];
				return true;
			}
			idx++;
		}

		n0 = m_children[idx - 1];
		n1 = m_children[idx];
		return true;
	}

	void BlendNode1D::BlendChildren(float t) {
		//get the blend nodes for the param
		BlendNode* n0, *n1;
		bool lerp = FindSegment(m_blend_param->x, n0, n1);

		//if there is no lerp to do, it is because param is smaller than first pos or 
		//bigger than last
		if (!lerp) {
			if(m_blend_param->x <= m_children[0]->m_blend_pos.x)
				m_children[0]->m_animation->ProducePose(t, m_current_pose);
			else
				m_children.back()->m_animation->ProducePose(t, m_current_pose);
			return;
		}
			

		//produce both poses
		n0->ProducePose(t);
		n1->ProducePose(t);

		//compute blend param respect to the segment
		float B;
		if (n1->m_blend_pos.x - n0->m_blend_pos.x == 0) //sanity check
			B = 0.f;
		else
			B = (m_blend_param->x - n0->m_blend_pos.x) / (n1->m_blend_pos.x - n0->m_blend_pos.x);

		BlendPoseLerp(n0->m_current_pose, n1->m_current_pose, this->m_current_pose, B);
	}

	void BlendPoseLerp(const std::unordered_map<Node*, Transform>& pose0, std::unordered_map<Node*, Transform>& pose1,
		std::unordered_map<Node*, Transform>& resulting_pose, float blend_param) {
		for (auto& it_p0 : pose0) {
			auto it_p1 = pose1.find(it_p0.first);
			if (it_p1 == pose1.end()) {// if the joint is modified by first anim but not by the second
				//add the transform from first anim
				resulting_pose[it_p0.first] = it_p0.second;
			}
			else {
				//blend both transforms
				Transform resulting_tr;
				resulting_tr.m_position = LerpVec3(it_p0.second.m_position, it_p1->second.m_position, blend_param);
				resulting_tr.m_rotation = LerpQuat(it_p0.second.m_rotation, it_p1->second.m_rotation, blend_param);
				//add it to the resulting pose map
				resulting_pose[it_p0.first] = resulting_tr;

				//erase from second pose, necessary  cause last step will check joints that were modified by second animation
				// but no by first animation
				pose1.erase(it_p1);
			}
		}

		//iterate through remaining joints in second animation
		for (auto& it : pose1)
			resulting_pose[it.first] = it.second;
	}

	glm::vec3 LerpVec3(const glm::vec3& vec0, const glm::vec3& vec1, float blend_param) {
		return (1 - blend_param) * vec0 + blend_param * vec1;
	} 

	glm::quat LerpQuat(const glm::quat& quat0, const glm::quat& quat1, float blend_param) {
		return glm::slerp(quat0, quat1, blend_param);
	}

	void BlendNode2D::GenerateTriangles() {
		m_triangles.clear();
		std::vector<double> positions;
		for(auto& it : m_children){
			positions.push_back(it->m_blend_pos.x);
			positions.push_back(it->m_blend_pos.y);
		}

		// need 3 triangles (2 components each triangle vtx)
		if (positions.size() < 6)
			return;

		delaunator::Delaunator del(positions);

		for (size_t i = 0; i < del.triangles.size(); i += 3) {
			std::array<unsigned int, 3> arr;
			arr[0] = del.triangles[i];
			arr[1] = del.triangles[i+1];
			arr[2] = del.triangles[i+2];
			m_triangles.push_back(arr);
		}
	}

	void BlendNode2D::BlendChildren(float t) {
		// we assume that the children are sorted incrementally by their blend position
		BlendNode* n0, * n1, * n2;
		float a0, a1, a2;
		bool found = FindNodesBarycentric(*m_blend_param, n0, n1, n2, a0, a1, a2);
		
		if (!found) return;
		
		// produce the pose from each
		n0->ProducePose(t); n1->ProducePose(t); n2->ProducePose(t);
		
		// blend the pose into ours
		BlendPoseBarycentric(n0->m_current_pose, n1->m_current_pose, n2->m_current_pose, this->m_current_pose,
			a0, a1, a2);
	}
	
	bool BlendNode2D::FindNodesBarycentric(glm::vec2 param, BlendNode*& n0, BlendNode*& n1, BlendNode*& n2, float& a0, float& a1, float& a2) {
		for (auto& it : m_triangles) {
			//take the 3 nodes that represent the triangle
			BlendNode* node_0 = m_children[it[0]]; BlendNode* node_1 = m_children[it[1]]; BlendNode* node_2 = m_children[it[2]];

			//get triangle area
			float A = glm::abs(cross_product_2D((node_1->m_blend_pos - node_0->m_blend_pos), (node_2->m_blend_pos - node_0->m_blend_pos)));
			
			// delta = subArea/Area
			float delta0 = cross_product_2D((node_1->m_blend_pos - param), (node_2->m_blend_pos - param))/A;
			float delta1 = cross_product_2D((node_2->m_blend_pos - param), (node_0->m_blend_pos - param))/A;
			float delta2 = cross_product_2D((node_0->m_blend_pos - param), (node_1->m_blend_pos - param))/A;

			//if param is inside the triangle then all deltas are positive(possible becasue checking cross product counterclowise)
			if (delta0 <= 0 && delta1 <= 0 && delta2 <= 0) {
				n0 = node_0; n1 = node_1; n2 = node_2;
				//convert all delta to positive
				a0 = -delta0; a1 = -delta1; a2 = -delta2;
				return true;
			}
		}

		return false;
	}

	void BlendPoseBarycentric(const std::unordered_map<Node*, Transform>& pose0, const std::unordered_map<Node*, Transform>& pose1,
		const std::unordered_map<Node*, Transform>& pose2, std::unordered_map<Node*, Transform>& resulting_pose, float a0, float a1, float a2) {
		//check if any of the weights is one, in that case that pose is the resultant
		if (a0 == 1.f) {
			resulting_pose = pose0;		return;
		}
		if(a1 == 1.f){
			resulting_pose = pose1;		return;
		}
		if (a2 == 1.f){
			resulting_pose = pose2;		return;
		}

		for (auto& it_p0 : pose0) {
			Transform tr_0, tr_1, tr_2;
			tr_0 = it_p0.second;
			auto it_p1 = pose1.find(it_p0.first);
			if (it_p1 == pose1.end()) {
				tr_1 = Transform();
				a1 = 0.f;
			}
			else
				tr_1 = it_p1->second;
			auto it_p2 = pose2.find(it_p0.first);
			if (it_p2 == pose2.end()) {
				tr_2 = Transform();
				a2 = 0.f;
			}
			else
				tr_2 = it_p2->second;

			//lerp components
			Transform resulting_tr;
			resulting_tr.m_position = tr_0.m_position * a0 + tr_1.m_position * a1 + tr_2.m_position * a2;
			resulting_tr.m_rotation = glm::normalize(tr_0.m_rotation * a0 + tr_1.m_rotation * a1 + tr_2.m_rotation * a2);
			//add it to the resulting pose map
			resulting_pose[it_p0.first] = resulting_tr;
		}
	}

	float cross_product_2D(const glm::vec2& v0, const glm::vec2& v1) {
		return v0.x * v1.y - v0.y * v1.x;
	}

}