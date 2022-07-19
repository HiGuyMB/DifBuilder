#include "DIFBuilder/BSPLib.h"
#include <glm/glm.hpp>
#include <algorithm>
#include <future>
#include <unordered_set>
#include <map>
#include "Nanoflann/include/nanoflann.hpp"
#include <sstream>

template<typename T>
struct PointList
{
	struct Point
	{
		T x, y, z;
	};

public:
	std::vector<Point> pts;

	inline size_t kdtree_get_point_count() const { return pts.size(); }

	// Returns the dim'th component of the idx'th point in the class:
	// Since this is inlined and the "dim" argument is typically an immediate value, the
	//  "if/else's" are actually solved at compile time.
	inline T kdtree_get_pt(const size_t idx, const size_t dim) const
	{
		if (dim == 0) return pts[idx].x;
		else if (dim == 1) return pts[idx].y;
		else return pts[idx].z;
	}

	// Optional bounding-box computation: return false to default to a standard bbox computation loop.
	//   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
	//   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
	template <class BBOX>
	bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }

	void copyfrom(std::vector<glm::vec3> pts)
	{
		for (auto& it : pts)
		{
			Point p = { it.x,it.y,it.z };
			this->pts.push_back(p);
		}
	}
};

typedef nanoflann::KDTreeSingleIndexDynamicAdaptor<nanoflann::L2_Simple_Adaptor<double,PointList<double>>,PointList<double>,3> SearchTree;

BSPNodeAllocator::BSPNodeAllocator()
{

}

BSPNodeAllocator::~BSPNodeAllocator()
{
	for (auto& node : nodes)
	{
		delete node;
	}
}

BSPNode* BSPNodeAllocator::allocate()
{
	BSPNode* node = new BSPNode();
	node->alloc = this;
	this->nodes.push_back(node);
	return node;
}

void swap(int* a, int* b)
{
	int temp = *a;
	*a = *b;
	*b = temp;
}

Plane selectBestSplitter(std::vector<Polygon*>& polygons)
{
	// KD-Tree kinda
	std::vector<float> xlist;
	std::vector<float> ylist;
	std::vector<float> zlist;
	glm::vec3 min(1e8, 1e8, 1e8);
	glm::vec3 max(-1e8, -1e8, -1e8);
	for (auto& poly : polygons)
	{
		for (auto& v : poly->VertexList)
		{
			xlist.push_back(v.p.x);
			ylist.push_back(v.p.y);
			zlist.push_back(v.p.z);

			if (v.p.x > max.x)
				max.x = v.p.x;
			if (v.p.y > max.y)
				max.y = v.p.y;
			if (v.p.z > max.z)
				max.z = v.p.z;

			if (v.p.x < min.x)
				min.x = v.p.x;
			if (v.p.y < min.y)
				min.y = v.p.y;
			if (v.p.z < min.z)
				min.z = v.p.z;
		}
	}
	std::sort(xlist.begin(), xlist.end());
	std::sort(ylist.begin(), ylist.end());
	std::sort(zlist.begin(), zlist.end());

	glm::vec3 centre = (max + min) * 0.5f;

	float xmed = xlist[floor(xlist.size() / 2)];
	float ymed = ylist[floor(ylist.size() / 2)];
	float zmed = zlist[floor(zlist.size() / 2)];

	Plane planes[] = { Plane(glm::vec3(xmed, centre.y, centre.z), glm::vec3(1, 0, 0)), Plane(glm::vec3(centre.x, ymed, centre.z), glm::vec3(0, 1, 0)), Plane(glm::vec3(centre.x, centre.y, zmed), glm::vec3(0, 0, 1))};

	int scores[3] = { 0, 0, 0 };
	int indices[3] = { 0, 1, 2 };

	for (int i = 0; i < 3; i++)
	{
		for (auto& poly : polygons)
		{
			int cl = poly->classify(planes[i]);
			if (cl == 0)
				cl = 2;

			scores[i] += cl;
		}
	}

	// manual bubble sort
	if (scores[0] > scores[1])
	{
		swap(&scores[0], &scores[1]);
		swap(&indices[0], &indices[1]);
	}

	if (scores[1] > scores[2])
	{
		swap(&scores[1], &scores[2]);
		swap(&indices[1], &indices[2]);
	}

	if (scores[0] > scores[1])
	{
		swap(&scores[0], &scores[1]);
		swap(&indices[0], &indices[1]);
	}

	return planes[indices[0]];
}

bool BSPNode::subdivide() {
	if (!this->IsLeaf) 
	{
		return false;
	}

	if (this->polygons.size() < 8)
	{
		return false;
	}

	std::vector<Polygon*> frontList;
	std::vector<Polygon*> backList;
	std::vector<Polygon*> commonList;

	Plane splitter = selectBestSplitter(this->polygons);

	for (auto& it : this->polygons)
	{
		int score = it->classify(splitter);

		if (score > 0) {
			frontList.push_back(it);
		}
		if (score < 0) {
			backList.push_back(it);
		}

		if (score == 0) {
			commonList.push_back(it);
		}
	}

	if (frontList.size() == 0 || backList.size() == 0)
		return false; // Can't split

	frontList.reserve(frontList.size() + std::distance(commonList.begin(), commonList.end()));
	frontList.insert(frontList.end(), commonList.begin(), commonList.end());

	backList.reserve(backList.size() + std::distance(commonList.begin(), commonList.end()));
	backList.insert(backList.end(), commonList.begin(), commonList.end());

	BSPNode* front = this->alloc->allocate();
	BSPNode* back = this->alloc->allocate();

	front->IsLeaf = true;
	front->polygons = frontList;

	back->IsLeaf = true;
	back->polygons = backList;

	front->subdivide();
	back->subdivide();

	this->IsLeaf = false;
	this->polygons.clear();
	this->Front = front;
	this->Back = back;

	this->plane = splitter;

	return true;
}

//void GatherBrushes(BSPNode* node, std::vector<Polygon*>* list)
//{
//	if (node->IsLeaf)
//	{
//		if (std::find(list->begin(), list->end(), node->poly) == list->end())
//			list->push_back(node->poly);
//		//if (node.FrontLeaf != NULL)
//		//{
//		//	for (int i = 0; i < node.FrontLeaf->polygons.size(); i++)
//		//		list->push_back(node.FrontLeaf->polygons[i]);
//		//}
//		//if (node.BackLeaf != NULL)
//		//{
//		//	for (int i = 0; i < node.BackLeaf->polygons.size(); i++)
//		//		list->push_back(node.BackLeaf->polygons[i]);
//		//}
//	}
//	else
//	{
//		if (node->Front != NULL)
//		{
//			GatherBrushes(node->Front, list);
//		}
//		if (node->Back != NULL)
//		{
//			GatherBrushes(node->Back, list);
//		}
//	}
//}

//void CalculateCenter(BSPNode* bsp)
//{
//	if (bsp->IsLeaf)
//	{
//		glm::vec3 centroid = glm::vec3(0, 0, 0);
//		for (int i = 0; i < bsp->poly->VertexList.size(); i++)
//			centroid += bsp->poly->VertexList[i].p;
//
//		centroid /= bsp->poly->VertexList.size();
//
//		bsp->center = glm::vec3(centroid);
//		bsp->hasCenter = true;
//	}
//	else
//	{
//		glm::vec3 avgcenter = glm::vec3(0, 0, 0);
//		int c = 0;
//		if (bsp->Front != NULL)
//		{
//			if (!bsp->Front->hasCenter)
//				CalculateCenter(bsp->Front);
//
//			c++;
//			avgcenter += (bsp->Front->center);
//		}
//		if (bsp->Back != NULL)
//		{
//			if (!bsp->Back->hasCenter)
//				CalculateCenter(bsp->Back);
//
//			c++;
//			avgcenter += (bsp->Back->center);
//		}
//		avgcenter /= c;
//
//		bsp->center = glm::vec3(avgcenter);
//		bsp->hasCenter = true;
//	}
//}

//std::string prettifyBSP(BSPNode n)
//{
//
//	std::stringstream s;
//
//	s << "{ IsLeaf : \"" << n.IsLeaf << "\", Plane : \"" << n.plane.normal.x << " " << n.plane.normal.y << " " << n.plane.normal.z << " " << n.plane.d << "\", Front : " << ((n.Front == NULL) ? "null" : prettifyBSP(*n.Front)) << ", Back : " << ((n.Back == NULL) ? "null" : prettifyBSP(*n.Back)) << ", Center : \"" << n.center.x << " " << n.center.y << " " << n.center.z << "\" }";
//
//	return s.str();
//}
//
//std::vector<BSPNode*> BuildBSP(std::vector<BSPNode*> Nodes, BSPNodeAllocator& alloc)
//{
//
//	std::vector<glm::vec3> pts;
//	for (auto& it : Nodes)
//	{
//		CalculateCenter(it);
//		pts.push_back(it->center);
//	}
//
//	float minx = -100000, miny = -100000, minz = -100000, maxx = 100000, maxy = 100000, maxz = 100000;
//
//	for (auto& it : Nodes)
//	{
//		auto v = it->center;
//		if (v.x < minx) minx = v.x;
//		if (v.y < miny) miny = v.y;
//		if (v.z < minz) minz = v.z;
//		if (v.x > maxx) maxx = v.x;
//		if (v.y > maxy) maxy = v.y;
//		if (v.z > maxz) maxz = v.z;
//	}
//
//	glm::vec3 min = glm::vec3(minx, miny, minz);
//	glm::vec3 max = glm::vec3(maxx, maxy, maxz);
//
//	float dim = fmax(fmax(maxx - minx, maxy - miny), maxz - minz);
//
//	PointList<double> plist;
//	plist.copyfrom(pts);
//
//	SearchTree finder(3, plist);
//
//	//finder.initialize(pts, unibn::OctreeParams());
//
//	std::vector<BSPNode*> newnodes;
//	std::vector<bool> containedptlist = std::vector<bool>();
//	for (int i = 0; i < pts.size(); i++)
//	{
//		containedptlist.push_back(false);
//	}
//
//	int previndex = -1;
//	for (int i = 0; i < pts.size();i++)
//	{
//		if (containedptlist[i] == true)
//			continue;
//
//		if (i == pts.size() - 1)
//		{
//			newnodes.push_back(Nodes[i]);
//			break;
//		}
//
//		glm::vec3 pt = pts[i];
//		finder.removePoint(i);
//		
//		size_t retIndex;
//		double outsqrtdist;
//		nanoflann::KNNResultSet<double> resultset(1);
//		resultset.init(&retIndex, &outsqrtdist);
//
//		double querypoint[] = { pt.x,pt.y,pt.z };
//
//		bool found = finder.findNeighbors(resultset, querypoint, nanoflann::SearchParams());
//
//		previndex = retIndex;
//
//		//finder.knnSearch(&pt,1,//finder.findNeighbor<unibn::L2Distance<glm::vec3>>(pt, 0.01f);
//
//		glm::vec3 nb = pts[retIndex];
//
//		glm::vec3 center = (pt + nb) * 0.5f;
//
//		Plane p = Plane(center, nb - pt);
//
//		BSPNode* node = alloc.allocate();
//		node->center = glm::vec3(center);
//		node->Front = Nodes[retIndex];
//		node->Back = Nodes[i];
//		node->plane = p;
//		newnodes.push_back(node);
//
//		finder.removePoint(retIndex);
//		containedptlist[retIndex] = true;
//
//		//if (pts.size() != 0)
//			//finder.initialize(pts, unibn::OctreeParams());
//	}
//
//	//for (auto& n : newnodes)
//	//{
//	//	std::string pretty = prettifyBSP(n);
//	//}
//
//	return newnodes;
//}
//
//BSPNode* BuildBSPRecurse(std::vector<BSPNode*> Nodes, BSPNodeAllocator& alloc)
//{
//	while (Nodes.size() > 1)
//		Nodes = BuildBSP(Nodes, alloc);
//
//	return Nodes[0];
//}