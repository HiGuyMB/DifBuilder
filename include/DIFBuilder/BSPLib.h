#pragma once
#include <stdlib.h>
#include <glm/glm.hpp>
#include <vector>

class Plane
{
public:
	Plane()
	{

	}
	~Plane()
	{

	}

	Plane(glm::vec3 a, glm::vec3 b, glm::vec3 c)
	{
		glm::vec3 v1 = a - b;
		glm::vec3 v2 = c - b;
		glm::vec3 res = glm::cross(v1, v2);

		this->normal = res;
		float w = sqrt(normal.x * normal.x + normal.y * normal.y + normal.z * normal.z);
		this->normal.x /= w;
		this->normal.y /= w;
		this->normal.z /= w;

		//normal = glm::normalize(normal);

		d = -(glm::dot(b, normal));
		pt = b;
	}

	Plane(glm::vec3 a, float d)
	{
		this->normal = a;
		this->d = d;
	}

	Plane(glm::vec3 point, glm::vec3 normal)
	{
		this->normal = normal;
		float w = sqrt(normal.x * normal.x + normal.y * normal.y + normal.z * normal.z);
		this->normal.x /= w;
		this->normal.y /= w;
		this->normal.z /= w;
		//normal = glm::normalize(normal);

		d = -(glm::dot(point, normal));
		pt = point;
	}

	float DistanceToPoint(glm::vec3& point)
	{
		return (glm::dot(this->normal, point)) + this->d;
	}

	glm::vec3 pt;
	glm::vec3 normal;
	float d = 0;
};

class Polygon;

class BSPNodeAllocator;

struct BSPNode
{
	BSPNodeAllocator* alloc;

	bool IsLeaf = false;
	Plane plane;
	BSPNode* Front = NULL;
	BSPNode* Back = NULL;
	std::vector<Polygon*> polygons;

public:
	bool subdivide();
};

class BSPNodeAllocator
{
	std::vector<BSPNode*> nodes;

public:
	BSPNodeAllocator();
	~BSPNodeAllocator();

	BSPNode* allocate();
};


struct Vertex
{
	glm::vec3 p;
	glm::vec2 uv;
	int index = -1;
};

class Polygon
{
public:
	std::vector<Vertex> VertexList;
	std::vector<int> Indices;
	glm::vec3 Normal;
	Plane plane;
	long TextureIndex = 0;
	BSPNode* node = NULL;
	bool IsUsed = false;
	int surfaceIndex = -1;
	std::vector<int> finalWindings;
	int planeIndex;
	bool planeExported = false;
	glm::vec3 min;
	glm::vec3 max;

	Polygon() {

	}
	Polygon(const Polygon&) = delete;

	int classify(Plane& splitting) {
		float firstDot = splitting.DistanceToPoint(VertexList[0].p);
		int cl = firstDot >= 0 ? 1 : -1;

		for (int i = 1; i < VertexList.size(); i++) {
			float nextDot = splitting.DistanceToPoint(VertexList[i].p);
			int nextCl = nextDot >= 0 ? 1 : -1;

			if (cl != nextCl) {
				return 0;
			}
		}

		return cl;
	}

	void calculateBounds() {
		min = glm::vec3(1e8, 1e8, 1e8);
		max = glm::vec3(-1e8, -1e8, -1e8);
		for (auto& v : VertexList) {
			if (v.p.x < min.x) min.x = v.p.x;
			if (v.p.y < min.y) min.y = v.p.y;
			if (v.p.z < min.z) min.z = v.p.z;
			if (v.p.x > max.x) max.x = v.p.x;
			if (v.p.y > max.y) max.y = v.p.y;
			if (v.p.z > max.z) max.z = v.p.z;
		}
	}
};

std::vector<BSPNode*> BuildBSP(std::vector<BSPNode*> Nodes, BSPNodeAllocator& alloc);

BSPNode* BuildBSPRecurse(std::vector<BSPNode*> Nodes, BSPNodeAllocator& alloc);

//void SplitPolygon(POLYGON *Poly, Plane *Plane, POLYGON *FrontSplit, POLYGON *BackSplit);

//bool Get_Intersect(glm::vec3 *linestart, glm::vec3 *lineend, glm::vec3 *vertex, glm::vec3 *normal, glm::vec3 & intersection, float &percentage);
//void DeletePolygon(POLYGON *Poly);
//void GatherBrushes(BSPNode* node, std::vector<Polygon*>* list);