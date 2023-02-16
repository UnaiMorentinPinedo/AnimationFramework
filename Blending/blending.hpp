/*-------------------------------------------------------
File Name: blending.hpp
Author: Unai Morentin
---------------------------------------------------------*/

#include <vector>
#include <array>
#include "math.hpp"
#include "animation.hpp"

namespace cs460 {
	void BlendPoseLerp(const std::unordered_map<Node*, Transform>& pose0, std::unordered_map<Node*, Transform>& pose1,
		std::unordered_map<Node*, Transform>& resulting_pose, float blend_param);
	void BlendPoseBarycentric(const std::unordered_map<Node*, Transform>& pose0, const std::unordered_map<Node*, Transform>& pose1, 
		const std::unordered_map<Node*, Transform>& pose2, std::unordered_map<Node*, Transform>& resulting_pose, float a0, float a1, float a2);
	float cross_product_2D(const glm::vec2& v0, const glm::vec2& v1);
	glm::vec3 LerpVec3(const glm::vec3& vec0, const glm::vec3& vec1, float blend_param);
	glm::quat LerpQuat(const glm::quat& quat0, const glm::quat& quat1, float blend_param);

	struct BlendNode {
		BlendNode();
		virtual ~BlendNode();
		BlendNode* m_parent = nullptr;
		std::vector<BlendNode*> m_children;

		glm::vec2 m_blend_pos;	// blend position, set by the editor
		glm::vec2* m_blend_param = nullptr; // blend parameters used for 1D or 2D blending, this is the input to the node, set by the application

		Animation* m_animation = nullptr; // animation that this node represents

		//produced pose
		std::unordered_map<Node*, Transform> m_current_pose;

		void ProducePose(float t);

		// Called on internal nodes in the blend tree
		virtual void BlendChildren(float t) {}
	};

	//1D blending node
	struct BlendNode1D : public BlendNode {
		bool FindSegment(float param, BlendNode*& n0, BlendNode*& n1);

		void BlendChildren(float t);
	};

	struct BlendNode2D : public BlendNode {
		//triangle container
		std::vector<std::array<unsigned int, 3>> m_triangles;

		// generates the triangles using delaunay triangulation 
		// based on the children's blendPosition
		void GenerateTriangles();

		// determine in which triangle the param is located and find n0, n1, n2
		// perform barycentric compute to extract a0,a1,a2
		bool FindNodesBarycentric(glm::vec2 param, BlendNode*& n0, BlendNode*& n1, BlendNode*& n2, float& a0, float& a1, float& a2);
		
		// Performs two dimensional blending using the children nodes. 
		void BlendChildren(float t);
	};
}