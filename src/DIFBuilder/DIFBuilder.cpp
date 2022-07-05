//-----------------------------------------------------------------------------
// Copyright (c) 2016, HiGuy Smith
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the project nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//-----------------------------------------------------------------------------

#include <DIFBuilder/DIFBuilder.hpp>
#include "DIFBuilder/BSPLib.h"
#include <iomanip>
#include <math.h>
#include <algorithm>
#include <unordered_map>
#include <sstream>

DIF_NAMESPACE

struct ObjectHash
{
	int hash;
	int obj;
};

struct HullPoly
{
	std::vector<int> points;
	int planeIndex;
};

//Because floating point math sucks
template<typename T>
bool closeEnough(const T& p1, const T& p2, const F32 distance = 0.0001f) {
	return glm::distance(p1, p2) < distance;
}

double roundDoubleAppox(double x)
{
	return floor(x * 1e5 + 0.5) / 1e5;
}

class PlaneMap
{
	//wtf
	std::unordered_map<uint64_t, std::unordered_map<uint64_t, std::unordered_map<uint64_t, std::unordered_map<uint64_t, short>>>> cells;

public:
	PlaneMap() = default;
	~PlaneMap() = default;

	void insert(Plane& p, short index) {
		double xin = roundDoubleAppox(p.normal.x);
		double yin = roundDoubleAppox(p.normal.y);
		double zin = roundDoubleAppox(p.normal.z);
		double din = roundDoubleAppox(p.d);

		uint64_t x = *(uint64_t*)&xin;
		uint64_t y = *(uint64_t*)&yin;
		uint64_t z = *(uint64_t*)&zin;
		uint64_t d = *(uint64_t*)&din;

		if (this->cells.find(x) != this->cells.end()) {
			auto& yfind = this->cells[x];
			if (yfind.find(y) != yfind.end()) {
				auto& zfind = yfind[y];
				if (zfind.find(z) != zfind.end()) {
					auto& dfind = zfind[z];
					if (dfind.find(d) == dfind.end()) {
						dfind.insert(std::make_pair(d, index));
					}
				} else {
					std::unordered_map<uint64_t, short> ins;
					ins.insert(std::make_pair(d, index));
					zfind.insert(std::make_pair(z, std::move(ins)));
				}
			} else {
				std::unordered_map<uint64_t, std::unordered_map<uint64_t, short>> ins2;
				std::unordered_map<uint64_t, short> ins;
				ins.insert(std::make_pair(d, index));
				ins2.insert(std::make_pair(z, std::move(ins)));
				yfind.insert(std::make_pair(y, std::move(ins2)));
			}
		} else {
			std::unordered_map<uint64_t, std::unordered_map<uint64_t, std::unordered_map<uint64_t, short>>> ins3;
			std::unordered_map<uint64_t, std::unordered_map<uint64_t, short>> ins2;
			std::unordered_map<uint64_t, short> ins;
			ins.insert(std::make_pair(d, index));
			ins2.insert(std::make_pair(z, std::move(ins)));
			ins3.insert(std::make_pair(y, std::move(ins2)));
			this->cells.insert(std::make_pair(x, std::move(ins3)));
		}
	}
	short* get(Plane& p) {
		double xin = roundDoubleAppox(p.normal.x);
		double yin = roundDoubleAppox(p.normal.y);
		double zin = roundDoubleAppox(p.normal.z);
		double din = roundDoubleAppox(p.d);

		uint64_t x = *(uint64_t*)&xin;
		uint64_t y = *(uint64_t*)&yin;
		uint64_t z = *(uint64_t*)&zin;
		uint64_t d = *(uint64_t*)&din;

		if (this->cells.find(x) != this->cells.end()) {
			auto& yfind = this->cells[x];
			if (yfind.find(y) != yfind.end()) {
				auto& zfind = yfind[y];
				if (zfind.find(z) != zfind.end()) {
					auto& dfind = zfind[z];
					if (dfind.find(d) != dfind.end()) {
						return &dfind[d];
					}
				}
			}
		}

		return NULL;

	}
};

Interior::TexGenEq getTexGenFromPoints(const glm::vec3 &point0, const glm::vec3 &point1, const glm::vec3 &point2, glm::vec2 uv0, glm::vec2 uv1, glm::vec2 uv2);

PolygonAllocator::PolygonAllocator() = default;

PolygonAllocator::~PolygonAllocator() {
	for (auto& node : nodes) {
		delete node;
	}
}

Polygon* PolygonAllocator::allocate() {
	Polygon* node = new Polygon();
	this->nodes.push_back(node);
	return node;
}

DIFBuilder::DIFBuilder(const DIF &dif) : mScale(1.0f) {
	//TODO: Extract triangles
}

bool checkTriangle(const DIFBuilder::Triangle& triangle)
{
	glm::vec3 ab = triangle.points[1].vertex - triangle.points[0].vertex;
	glm::vec3 ac = triangle.points[2].vertex - triangle.points[0].vertex;
	glm::vec3 cp = glm::cross(ab, ac);
	if (glm::dot(cp, cp) <= 0.001f)
		return false;
	return true;
}

void DIFBuilder::addTriangle(const Triangle &triangle) {
	if (!checkTriangle(triangle))
		return;
	mTriangles.push_back(triangle);
}

void DIFBuilder::addTriangle(const Triangle &triangle, const std::string &material) {
	if (!checkTriangle(triangle))
		return;

	Triangle tri(triangle);

	//Find the material
	auto find = std::find(mMaterials.begin(), mMaterials.end(), material);
	if (find == mMaterials.end()) {
		tri.material = mMaterials.size();
		mMaterials.push_back(material);
	} else {
		//Distance from the start
		tri.material = (find - mMaterials.begin());
	}

	addTriangle(tri);
}

void DIFBuilder::addPathedInterior(const Interior &interior, std::vector<Marker>& path)
{
	mPathedInteriors.push_back(std::pair<Interior, std::vector<Marker>>(interior, path));
}

void DIFBuilder::addEntity(const GameEntity& entity)
{
	mGameEntities.push_back(entity);
}

void DIFBuilder::addTrigger(const Trigger& trigger)
{
	mTriggers.push_back(trigger);
}


struct plane_hash {
	std::size_t operator()(Plane const& p) const {
		double xin = roundDoubleAppox(p.normal.x);
		double yin = roundDoubleAppox(p.normal.y);
		double zin = roundDoubleAppox(p.normal.z);
		double din = roundDoubleAppox(p.d);

		uint64_t x = *(uint64_t*)&xin;
		uint64_t y = *(uint64_t*)&yin;
		uint64_t z = *(uint64_t*)&zin;
		uint64_t d = *(uint64_t*)&din;
		return std::hash<uint64_t>{}(x) ^ std::hash<uint64_t>{}(y) ^ std::hash<uint64_t>{}(z) ^ std::hash<uint64_t>{}(d);
	}
};

struct plane_equal {
	bool operator()(Plane const& p1, Plane const& p2) const
	{
		return closeEnough(p1.normal.x, p2.normal.x, 1e-5) && closeEnough(p1.normal.y, p2.normal.y, 1e-5) && closeEnough(p1.normal.z, p2.normal.z, 1e-5) && closeEnough(p1.d, p1.d, 1e-5);
	}
};

uint64_t hashPlane(Plane& pl)
{
	double xin = roundDoubleAppox(pl.normal.x);
	double yin = roundDoubleAppox(pl.normal.y);
	double zin = roundDoubleAppox(pl.normal.z);
	double din = roundDoubleAppox(pl.d);

	uint64_t x = *(uint64_t*)&xin;
	uint64_t y = *(uint64_t*)&yin;
	uint64_t z = *(uint64_t*)&zin;
	uint64_t d = *(uint64_t*)&din;

	uint64_t hash = 14695981039346656037;
	hash ^= x;
	hash *= 1099511628211;
	hash ^= y;
	hash *= 1099511628211;
	hash ^= z;
	hash *= 1099511628211;
	hash ^= d;
	hash *= 1099511628211;

	return hash;
}

//Here come the dif writing functions, algorithm is nearly the same as the ones used in map2dif

short ExportPlane(Interior* interior, Plane& testplane, PlaneMap& planeIndices)
{

	short* pindex = planeIndices.get(testplane);
	if (pindex != NULL)
	{
		short planeIndex = *pindex;
		glm::vec3& norm = interior->normal[interior->plane[planeIndex].normalIndex];
		return planeIndex;
	}

	// try flipped
	Plane flippedplane = testplane;
	flippedplane.normal *= -1;
	flippedplane.d *= -1;

	pindex = planeIndices.get(flippedplane);
	if (pindex != NULL)
	{
		short planeIndex = *pindex;
		glm::vec3& norm = interior->normal[interior->plane[planeIndex].normalIndex];
		return planeIndex | 0x8000;
	}

	int index = interior->plane.size();

	if (index >= 32767)
		throw new std::exception("Out of plane indices");

	Interior::Plane p = Interior::Plane();

	int normindex;

	normindex = interior->normal.size();
	interior->normal.push_back(testplane.normal);
	interior->normal2.push_back(testplane.normal);
	p.normalIndex = normindex;
	p.planeDistance = testplane.d;

	planeIndices.insert(testplane, index);

	interior->plane.push_back(p);

	return index;

}

short ExportPlane(Interior *interior, Polygon* poly, PlaneMap& planeIndices)
{
	Plane testplane = Plane(poly->VertexList[poly->Indices[0]].p, poly->VertexList[poly->Indices[1]].p, poly->VertexList[poly->Indices[2]].p);

	return ExportPlane(interior, testplane, planeIndices);
}

short ExportTexture(Interior *interior,std::string tex)
{
	for (int i = 0; i < interior->materialName.size(); i++)
		if (interior->materialName[i] == tex)
			return i;

	int index = interior->materialName.size();
	interior->materialName.push_back(tex);
	return index;
}

short ExportTexGen(Interior *interior, Polygon* poly)
{
	glm::vec3 v1 = poly->VertexList[poly->Indices[0]].p;
	glm::vec3 v2 = poly->VertexList[poly->Indices[1]].p;
	glm::vec3 v3 = poly->VertexList[poly->Indices[2]].p;

	glm::vec2 uv1 = poly->VertexList[poly->Indices[0]].uv;
	glm::vec2 uv2 = poly->VertexList[poly->Indices[1]].uv;
	glm::vec2 uv3 = poly->VertexList[poly->Indices[2]].uv;

	Interior::TexGenEq texgen = getTexGenFromPoints(v1, v2, v3, uv1, uv2, uv3);

	int index = interior->texGenEq.size();
	interior->texGenEq.push_back(texgen);
	return index;
}

int ExportPoint(Interior *interior, Vertex& p)
{
	if (p.index != -1)
		return p.index;

	//
	//for (int i = 0; i < pointhashes->size(); i++)
	//	if (pointhashes->at(i).hash == hash)
	//		return pointhashes->at(i).obj;

	int index = interior->point.size();

	interior->point.push_back(p.p);
	interior->pointVisibility.push_back(-1);

	p.index = index;

	//ObjectHash o = ObjectHash();
	//o.hash = hash;
	//o.obj = index;
	//pointhashes->push_back(o);
	return index;
}

void ExportWinding(Interior *interior, Polygon* poly)
{
	std::vector<int> finalWinding = std::vector<int>();

	for (int i = 0; i < poly->Indices.size(); i++)
	{
		Vertex& p = poly->VertexList[poly->Indices[i]];
		finalWinding.push_back(ExportPoint(interior, p));
	}
	Interior::WindingIndex a = Interior::WindingIndex();
	a.windingStart = interior->index.size();
	a.windingCount = finalWinding.size();


	interior->windingIndex.push_back(a);

	for (int i = 0; i < finalWinding.size(); i++)
		interior->index.push_back(finalWinding[i]);
}

void ExportSurfaces(Interior *interior, std::vector<Polygon*>& polys, PlaneMap& planeIndices, std::vector<std::string> materialList)
{
	for (int i = 0; i < polys.size(); i++)
	{
		Polygon* poly = polys[i];

		Interior::Surface rSurface = Interior::Surface();
		rSurface.planeIndex = ExportPlane(interior, poly, planeIndices);
		rSurface.textureIndex = ExportTexture(interior, materialList[poly->TextureIndex]);
		rSurface.texGenIndex = ExportTexGen(interior, poly);
		rSurface.surfaceFlags = 16;
		rSurface.fanMask = 15;
		ExportWinding(interior, poly);
		Interior::WindingIndex last = interior->windingIndex.back();
		rSurface.windingStart = last.windingStart;
		rSurface.windingCount = last.windingCount;
		interior->windingIndex.pop_back();
		rSurface.lightCount = 0;
		rSurface.lightStateInfoStart = 0;
		rSurface.mapSizeX = 0;
		rSurface.mapSizeY = 0;
		rSurface.mapOffsetX = 0;
		rSurface.mapOffsetY = 0;
		interior->surface.push_back(rSurface);
		interior->normalLMapIndex.push_back(0);
		interior->alarmLMapIndex.push_back(0);
	}
}

void assertFloat(float u, float v)
{
	if (!closeEnough(u, v, 1e-5)) {
		printf("FLOATS NOT CLOSE ENOUGH: %f %f", u, v);
		assert(false);
	}
}

int ExportSurface(Interior *interior, Polygon* poly, PlaneMap& planeIndices, std::vector<std::string> materialList)
{
	int ret = interior->surface.size();
	poly->surfaceIndex = ret;
	Interior::Surface rSurface = Interior::Surface();
	rSurface.planeIndex = ExportPlane(interior, poly, planeIndices);

	// CHECK POLY NORMALS 
	bool isPlaneFlipped = (rSurface.planeIndex & 0x8000) > 0;
	int planeIndex = rSurface.planeIndex & ~0x8000;
	Interior::Plane p = interior->plane[planeIndex];
	glm::vec3 n = interior->normal[p.normalIndex];

	if (isPlaneFlipped) {
		n *= -1;
		p.planeDistance *= -1;
	}
	assertFloat(p.planeDistance, poly->plane.d);
	assertFloat(n.x, poly->Normal.x);
	assertFloat(n.y, poly->Normal.y);
	assertFloat(n.z, poly->Normal.z);

	rSurface.textureIndex = ExportTexture(interior, materialList[poly->TextureIndex]);
	rSurface.texGenIndex = ExportTexGen(interior, poly);
	rSurface.surfaceFlags = 16;
	ExportWinding(interior, poly);
	Interior::WindingIndex last = interior->windingIndex.back();
	rSurface.windingStart = last.windingStart;
	rSurface.windingCount = last.windingCount;
	interior->windingIndex.pop_back();
	rSurface.fanMask = 0;
	for (int i = 0; i < rSurface.windingCount; i++)
	{
		rSurface.fanMask |= (1 << i);
	}
	rSurface.lightCount = 0;
	rSurface.lightStateInfoStart = 0;
	rSurface.mapSizeX = 0;
	rSurface.mapSizeY = 0;
	rSurface.mapOffsetX = 0;
	rSurface.mapOffsetY = 0;
	interior->surface.push_back(rSurface);
	interior->normalLMapIndex.push_back(0);
	interior->alarmLMapIndex.push_back(0);
	return ret;
}

int CreateLeafIndex(int baseIndex, bool isSolid)
{
	int baseRet;
	if (isSolid)
	{
		baseRet = 0xC000;
	}
	else
	{
		baseRet = 0x8000;
	}
	return baseRet | baseIndex;
}

int ExportBSP(Interior *interior, BSPNode* n, std::vector<Polygon*>* polys, PlaneMap& planeIndices, std::vector<std::string> materialList, std::vector<Polygon*>* orderedpolys)
{
	if (n->IsLeaf)
	{
		int leafindex = interior->bspSolidLeaf.size();


		Interior::BSPSolidLeaf leaf = Interior::BSPSolidLeaf();
		leaf.surfaceIndex = interior->solidLeafSurface.size();
		leaf.surfaceCount = 0;

		std::vector<int> leafPolyIndices = std::vector<int>();

		//for (int i = 0; i < polys->size(); i++)
		//	if (polys->at(i).leaf == n.FrontLeaf)
		//	{
		//		leafPolyIndices.push_back(i);
		//	}
		//for (int i = 0; i < polys->size(); i++)
		//	if (polys->at(i).leaf == n.BackLeaf)
		//	{
		//		leafPolyIndices.push_back(i);
		//	}

		leafPolyIndices.push_back(ExportSurface(interior, n->poly, planeIndices, materialList));
		orderedpolys->push_back(n->poly);

		for (int i = 0; i < leafPolyIndices.size(); i++)
		{
			interior->solidLeafSurface.push_back(leafPolyIndices[i]);
			leaf.surfaceCount++;
		}

		interior->bspSolidLeaf.push_back(leaf);
		return CreateLeafIndex(leafindex, true);
	}
	else
	{
		int frontIndex = n->Front != NULL ? ExportBSP(interior, n->Front, polys, planeIndices, materialList, orderedpolys) : CreateLeafIndex(0, false);
		int backIndex = n->Back != NULL ? ExportBSP(interior, n->Back, polys, planeIndices, materialList, orderedpolys) : CreateLeafIndex(0, false);

		Interior::BSPNode bspnode = Interior::BSPNode();
		bspnode.frontIndex = frontIndex;
		bspnode.backIndex = backIndex;
		bspnode.planeIndex = ExportPlane(interior, n->plane, planeIndices);
		int nodeindex = interior->bspNode.size();
		interior->bspNode.push_back(bspnode);

		return nodeindex;
	}
}

int ExportEmitString(Interior* interior,std::vector<U8>& emitstring, std::vector<ObjectHash>* emitstrHashes)
{
	// Search for already included strings.

	std::size_t strhash;
	for (int i = 0; i < emitstring.size(); i++)
	{
		std::size_t charhash = std::hash<char>{}(emitstring[i]);
		strhash ^= charhash;
		strhash ^= std::hash<int>{}(i);
	}

	for (int i = 0; i < emitstrHashes->size(); i++)
		if (emitstrHashes->at(i).hash == strhash)
			return emitstrHashes->at(i).obj;

	int index = interior->convexHullEmitStringCharacter.size();

	auto hashobj = ObjectHash();
	hashobj.hash = strhash;
	hashobj.obj = index;

	emitstrHashes->push_back(hashobj);

	for (int i = 0; i < emitstring.size(); i++)
		interior->convexHullEmitStringCharacter.push_back(emitstring.at(i));

	return index;

	//auto res = std::search(interior->convexHullEmitStringCharacter.begin(), interior->convexHullEmitStringCharacter.end(), emitstring.begin(), emitstring.end());

	//if (res == interior->convexHullEmitStringCharacter.end())
	//{
	//	int insertpt = interior->convexHullEmitStringCharacter.size();
	//	for (int i = 0; i < emitstring.size(); i++)
	//		interior->convexHullEmitStringCharacter.push_back(emitstring[i]);

	//	return insertpt;
	//}
	//else
	//{
	//	return std::distance(interior->convexHullEmitStringCharacter.begin(), res);
	//}
}

void ExportConvexHulls(Interior* interior, std::vector<std::vector<Polygon*>> polys, PlaneMap& planeIndices, std::vector<ObjectHash>* emitstrHashes, bool exportEmitStrings = false)
{
	for (int polyIndex = 0; polyIndex < polys.size(); polyIndex++)
	{
		Interior::ConvexHull hull = Interior::ConvexHull();
		hull.surfaceStart = interior->hullSurfaceIndex.size();
		hull.surfaceCount = polys[polyIndex].size();

		for (int i = 0; i < hull.surfaceCount; i++) {
			if (polys[polyIndex][i]->surfaceIndex == -1) {
				printf("%d %d", polyIndex, i);
				throw new std::exception("Invalid surface index");
			}
			interior->hullSurfaceIndex.push_back(polys[polyIndex][i]->surfaceIndex);
		}

		hull.hullStart = interior->hullIndex.size();
		hull.hullCount = 0;

		for (int i = 0; i < polys[polyIndex].size(); i++)
			hull.hullCount += polys[polyIndex][i]->VertexList.size();

		std::vector<int> hullPoints = std::vector<int>();
		std::vector<HullPoly> hullpolys = std::vector<HullPoly>();

		hull.polyListPointStart = interior->polyListPointIndex.size();

		for (int i = 0; i < polys[polyIndex].size(); i++)
		{
			HullPoly hp = HullPoly();
			hp.points = std::vector<int>();
			for (int j = 0; j < polys[polyIndex][i]->Indices.size(); j++)
			{
				int pt = ExportPoint(interior, polys[polyIndex][i]->VertexList[polys[polyIndex][i]->Indices[j]]);
				interior->hullIndex.push_back(pt);
				interior->polyListPointIndex.push_back(pt);
				hp.points.push_back(pt);
				hullPoints.push_back(pt);
			}
			hp.planeIndex = ExportPlane(interior, polys[polyIndex][i], planeIndices);
			hullpolys.push_back(hp);
		}

		hull.polyListPlaneStart = interior->polyListPlaneIndex.size();
		hull.planeStart = interior->hullPlaneIndex.size();

		for (int i = 0; i < polys[polyIndex].size(); i++)
		{
			int planeindex = ExportPlane(interior, polys[polyIndex][i], planeIndices);
			interior->polyListPlaneIndex.push_back(planeindex);
			interior->hullPlaneIndex.push_back(planeindex);
		}

		float minx = 1E+08f;
		float miny = 1E+08f;
		float minz = 1E+08f;
		float maxx = -1E+08f;
		float maxy = -1E+08f;
		float maxz = -1E+08f;

		for (int i = 0; i < polys[polyIndex].size(); i++)
		{
			for (int j = 0; j < polys[polyIndex][i]->VertexList.size(); j++)
			{
				glm::vec3 v = polys[polyIndex][i]->VertexList[j].p;
				if (v.x < minx)
					minx = v.x;
				if (v.y < miny)
					miny = v.y;
				if (v.z < minz)
					minz = v.z;
				if (v.x > maxx)
					maxx = v.x;
				if (v.y > maxy)
					maxy = v.y;
				if (v.z > maxz)
					maxz = v.z;
			}
		}

		hull.minX = minx;
		hull.minY = miny;
		hull.minZ = minz;
		hull.maxX = maxx;
		hull.maxY = maxy;
		hull.maxZ = maxz;

		//This is straight up copied from map2dif
		if (exportEmitStrings)
		{
			for (int i = 0; i < hullPoints.size(); i++)
			{
				static std::vector<U32> emitPoints(128);
				static std::vector<U16> emitEdges(128);
				static std::vector<U32> emitPolys;

				emitPoints.clear();
				emitEdges.clear();
				emitPolys.clear();

				int point = hullPoints[i];

				for (int j = 0; j < hullpolys.size(); j++)
				{
					bool found = false;
					for (int k = 0; k < hullpolys[j].points.size(); k++)
						if (hullpolys[j].points[k] == point)
							found = true;

					if (found)
					{
						for (int k = 0; k < hullpolys[j].points.size(); k++)
							emitPoints.push_back(hullpolys[j].points[k]);

						for (int k = 0; k < hullpolys[j].points.size(); k++)
						{
							int first = hullpolys[j].points[k];
							int next = hullpolys[j].points[(k + 1) % hullpolys[j].points.size()];

							int edge = (((int)fmin(first, next) << 8) | ((int)fmax(first, next)) << 0);
							emitEdges.push_back(edge);
						}

						emitPolys.push_back(j);
					}
				}

				for (int j = 0; j < hullpolys.size(); j++)
				{
					for (int k = 0; k < emitPolys.size(); k++)
					{
						if (emitPolys[k] == j) continue;

						if (hullpolys[emitPolys[k]].planeIndex == hullpolys[j].planeIndex)
						{
							bool found = false;

							for (int l = 0; l < emitPolys.size(); l++)
								if (emitPolys[l] == j)
									found = true;

							if (!found)
							{
								for (int l = 0; l < hullpolys[j].points.size(); l++)
								{
									emitPoints.push_back(hullpolys[j].points[l]);

									int first = hullpolys[j].points[l];
									int next = hullpolys[j].points[(l + 1) % hullpolys[j].points.size()];

									int edge = (((int)fmin(first, next) << 8) | ((int)fmax(first, next)) << 0);
									emitEdges.push_back(edge);
								}
								emitPolys.push_back
								(j);
							}
						}
					}
				}

				std::sort(emitPoints.begin(), emitPoints.end());
				auto last = std::unique(emitPoints.begin(), emitPoints.end());
				emitPoints.erase(last, emitPoints.end());
				std::sort(emitPoints.begin(), emitPoints.end());


				std::sort(emitEdges.begin(), emitEdges.end());
				auto last2 = std::unique(emitEdges.begin(), emitEdges.end());
				emitEdges.erase(last2, emitEdges.end());
				std::sort(emitEdges.begin(), emitEdges.end());

				for (int j = 0; j < emitEdges.size(); j++)
				{
					int firstIndex = emitEdges[j] >> 8;
					int nextIndex = emitEdges[j] & 0xFF;

					int newFirst = 0xFFFF;
					int newNext = 0xFFFF;
					for (int k = 0; k < emitPoints.size(); k++)
					{
						if (emitPoints[k] == firstIndex)
							newFirst = k;
						if (emitPoints[k] == nextIndex)
							newNext = k;
					}

					int newSignature = (((int)fmin(newFirst, newNext) << 8) |
						((int)fmax(newFirst, newNext) << 0));
					emitEdges[j] = newSignature;
				}

				std::vector<U8> emitString = std::vector<U8>();

				emitString.push_back(emitPoints.size());
				for (auto& p : emitPoints)
					emitString.push_back(i);

				emitString.push_back(emitEdges.size());
				for (auto& e : emitEdges)
				{
					emitString.push_back(e >> 8);
					emitString.push_back(e & 0xFF);
				}

				emitString.push_back(emitPolys.size());

				for (int j = 0; j < emitPolys.size(); j++)
				{
					emitString.push_back(hullpolys[emitPolys[j]].points.size());
					emitString.push_back(emitPolys[j]);
					for (int k = 0; k < hullpolys[emitPolys[j]].points.size(); k++)
					{
						bool found = false;
						for (int l = 0; l < emitPoints.size(); l++)
						{
							if (emitPoints[l] == hullpolys[emitPolys[j]].points[k])
							{
								emitString.push_back(l);
								found = true;
								break;
							}
						}
					}
				}

				interior->hullEmitStringIndex.push_back(ExportEmitString(interior, emitString, emitstrHashes));

				//interior->hullEmitStringIndex.push_back(0);
			}
		}
		else
		{
			for (int i = 0; i < hullPoints.size(); i++)
				interior->hullEmitStringIndex.push_back(0);
		}
		interior->convexHull.push_back(hull);			
	}
	if (!exportEmitStrings)
	{
		interior->convexHullEmitStringCharacter.push_back(0);
	}
	//interior->convexHullEmitStringCharacter.push_back(0);
	interior->polyListStringCharacter.push_back(0);
}

void ExportCoordBins(Interior* interior)
{
	for (int i = 0; i < 256; i++)
		interior->coordBin.push_back(Interior::CoordBin());

	for (int i = 0; i < 16; i++)
	{
		float minX = interior->boundingBox.minX;
		float maxX = interior->boundingBox.minX;
		minX += i * ((interior->boundingBox.maxX - interior->boundingBox.minX) / 16);
		maxX += (i+1) * ((interior->boundingBox.maxX - interior->boundingBox.minX) / 16);
		for (int j = 0; j < 16; j++)
		{
			float minY = interior->boundingBox.minY;
			float maxY = interior->boundingBox.minY;
			minY += j * ((interior->boundingBox.maxY - interior->boundingBox.minY) / 16);
			maxY += (j + 1) * ((interior->boundingBox.maxY - interior->boundingBox.minY) / 16);

			int binIndex = (i * 16) + j;
			interior->coordBin[binIndex].binStart = interior->coordBinIndex.size();

			for (int k = 0; k < interior->convexHull.size(); k++)
			{
				const auto& hull = interior->convexHull[k];

				if (!(minX > hull.maxX || maxX < hull.minX || maxY < hull.minY || minY > hull.maxY))
				{
					interior->coordBinIndex.push_back(k);
				}
			}
			interior->coordBin[binIndex].binCount = interior->coordBinIndex.size() - interior->coordBin[binIndex].binStart;
		}
	}
}

void ExportTrigger(DIF* dif, DIFBuilder::Trigger& trigger)
{
	Trigger trig = Trigger();
	trig.datablock = trigger.datablock;
	trig.name = trigger.name;
	trig.offset = trigger.position;
	trig.properties = Dictionary(trigger.properties);
	trig.polyhedron = Trigger::PolyHedron();

	trig.polyhedron.pointList.push_back(glm::vec3(-0.5, -0.5, 0.5));
	trig.polyhedron.pointList.push_back(glm::vec3(-0.5, 0.5, 0.5));
	trig.polyhedron.pointList.push_back(glm::vec3(0.5, 0.5, 0.5));
	trig.polyhedron.pointList.push_back(glm::vec3(-0.5, -0.5, 0.5));
	trig.polyhedron.pointList.push_back(glm::vec3(-0.5, -0.5, -0.5));
	trig.polyhedron.pointList.push_back(glm::vec3(-0.5, 0.5, -0.5));
	trig.polyhedron.pointList.push_back(glm::vec3(0.5, 0.5, -0.5));
	trig.polyhedron.pointList.push_back(glm::vec3(0.5, -0.5, -0.5));

	trig.polyhedron.planeList.push_back(PlaneF(-1, 0, 0, -0.5));
	trig.polyhedron.planeList.push_back(PlaneF(0, 1, 0, -0.5));
	trig.polyhedron.planeList.push_back(PlaneF(1, 0, 0, -0.5));
	trig.polyhedron.planeList.push_back(PlaneF(0, -1, 0, -0.5));
	trig.polyhedron.planeList.push_back(PlaneF(0, 0, 1, -0.5));
	trig.polyhedron.planeList.push_back(PlaneF(0, 0, -1, -0.5));

	Trigger::PolyHedronEdge e1;
	e1.vertex[0] = 0;
	e1.vertex[1] = 1;
	e1.face[0] = 0;
	e1.face[1] = 4;

	Trigger::PolyHedronEdge e2;
	e2.vertex[0] = 4;
	e2.vertex[1] = 5;
	e2.face[0] = 5;
	e2.face[1] = 0;

	Trigger::PolyHedronEdge e3;
	e3.vertex[0] = 0;
	e3.vertex[1] = 4;
	e3.face[0] = 3;
	e3.face[1] = 0;

	Trigger::PolyHedronEdge e4;
	e4.vertex[0] = 1;
	e4.vertex[1] = 2;
	e4.face[0] = 1;
	e4.face[1] = 4;

	Trigger::PolyHedronEdge e5;
	e5.vertex[0] = 5;
	e5.vertex[1] = 6;
	e5.face[0] = 5;
	e5.face[1] = 1;

	Trigger::PolyHedronEdge e6;
	e6.vertex[0] = 1;
	e6.vertex[1] = 5;
	e6.face[0] = 0;
	e6.face[1] = 1;

	Trigger::PolyHedronEdge e7;
	e2.vertex[0] = 2;
	e2.vertex[1] = 3;
	e2.face[0] = 2;
	e2.face[1] = 4;

	Trigger::PolyHedronEdge e8;
	e8.vertex[0] = 6;
	e8.vertex[1] = 7;
	e8.face[0] = 5;
	e8.face[1] = 2;

	Trigger::PolyHedronEdge e9;
	e9.vertex[0] = 2;
	e9.vertex[1] = 6;
	e9.face[0] = 1;
	e9.face[1] = 2;

	Trigger::PolyHedronEdge e10;
	e10.vertex[0] = 3;
	e10.vertex[1] = 0;
	e10.face[0] = 3;
	e10.face[1] = 4;

	Trigger::PolyHedronEdge e11;
	e11.vertex[0] = 7;
	e11.vertex[1] = 4;
	e11.face[0] = 5;
	e11.face[1] = 3;

	Trigger::PolyHedronEdge e12;
	e12.vertex[0] = 3;
	e12.vertex[1] = 7;
	e12.face[0] = 2;
	e12.face[1] = 3;

	trig.polyhedron.edgeList.push_back(e1);
	trig.polyhedron.edgeList.push_back(e2);
	trig.polyhedron.edgeList.push_back(e3);
	trig.polyhedron.edgeList.push_back(e4);
	trig.polyhedron.edgeList.push_back(e5);
	trig.polyhedron.edgeList.push_back(e6);
	trig.polyhedron.edgeList.push_back(e7);
	trig.polyhedron.edgeList.push_back(e8);
	trig.polyhedron.edgeList.push_back(e9);
	trig.polyhedron.edgeList.push_back(e10);
	trig.polyhedron.edgeList.push_back(e11);
	trig.polyhedron.edgeList.push_back(e12);

	dif->trigger.push_back(trig);
}

struct TempProcSurface {
	U32 numPoints;
	U32 pointIndices[32];
	U16 planeIndex;
	U8  mask;
};

struct PlaneGrouping {
	U32 numPlanes;
	U16 planeIndices[32];
	U8  mask;
};

void collisionFanFromSurface(Interior& interior, const Interior::Surface& rSurface, U32* fanIndices, U32* numIndices)
{
	U32 tempIndices[32];

	tempIndices[0] = 0;
	U32 idx = 1;
	U32 i;
	for (i = 1; i < rSurface.windingCount; i += 2)
		tempIndices[idx++] = i;
	for (i = ((rSurface.windingCount - 1) & (~0x1)); i > 0; i -= 2)
		tempIndices[idx++] = i;

	idx = 0;
	for (i = 0; i < rSurface.windingCount; i++) {
		if (rSurface.fanMask & (1 << i)) {
			fanIndices[idx++] = interior.index[rSurface.windingStart + tempIndices[i]];
		}
	}
	*numIndices = idx;
}

void buildHullPolyLists(Interior& interior) 
{
	interior.polyListStringCharacter.clear();
	interior.polyListPointIndex.clear();
	interior.polyListPlaneIndex.clear();

	std::vector<TempProcSurface> tempSurfaces;
	std::vector<PlaneGrouping>   planeGroups;
	std::vector<U16>             planeIndices;
	std::vector<U32>             pointIndices;
	std::vector<U8>              pointMasks;
	std::vector<U8>              planeMasks;

	for (U32 i = 0; i < interior.convexHull.size(); i++) {
		U32 j, k, l, m;

		Interior::ConvexHull& rHull = interior.convexHull[i];

		planeIndices.clear();
		pointIndices.clear();
		tempSurfaces.clear();
		planeGroups.clear();
		planeMasks.clear();
		pointMasks.clear();

		// Extract all the surfaces from this hull into our temporary processing format
		{
			for (j = 0; j < rHull.surfaceCount; j++) {
				TempProcSurface temp;

				U32 surfaceIndex = interior.hullSurfaceIndex[j + rHull.surfaceStart];
				{
					const Interior::Surface& rSurface = interior.surface[surfaceIndex];

					temp.planeIndex = rSurface.planeIndex;
					collisionFanFromSurface(interior, rSurface, temp.pointIndices, &temp.numPoints);
				}

				tempSurfaces.push_back(temp);
			}
		}

		// First order of business: extract all unique planes and points from
		//  the list of surfaces...
		{
			for (j = 0; j < tempSurfaces.size(); j++) {
				const TempProcSurface& rSurface = tempSurfaces[j];

				bool found = false;
				for (k = 0; k < planeIndices.size() && !found; k++) {
					if (rSurface.planeIndex == planeIndices[k])
						found = true;
				}
				if (!found)
					planeIndices.push_back(rSurface.planeIndex);

				for (k = 0; k < rSurface.numPoints; k++) {
					found = false;
					for (l = 0; l < pointIndices.size(); l++) {
						if (pointIndices[l] == rSurface.pointIndices[k])
							found = true;
					}
					if (!found)
						pointIndices.push_back(rSurface.pointIndices[k]);
				}
			}
		}

		// Now that we have all the unique points and planes, remap the surfaces in
		//  terms of the offsets into the unique point list...
		{
			for (j = 0; j < tempSurfaces.size(); j++) {
				TempProcSurface& rSurface = tempSurfaces[j];

				// Points
				for (k = 0; k < rSurface.numPoints; k++) {
					bool found = false;
					for (l = 0; l < pointIndices.size(); l++) {
						if (pointIndices[l] == rSurface.pointIndices[k]) {
							rSurface.pointIndices[k] = l;
							found = true;
							break;
						}
					}
				}
			}
		}

		// Now we group the planes together, and merge the closest groups until we're left
		//  with <= 8 groups
		{
			for (j = 0; j < planeIndices.size(); j++) {
				PlaneGrouping pg;
				pg.numPlanes = 1;
				pg.planeIndices[0] = planeIndices[j];
				planeGroups.push_back(pg);
			}

			while (planeGroups.size() > 8) {
				// Find the two closest groups.  If mdp(i, j) is the value of the
				//  largest pairwise dot product that can be computed from the vectors
				//  of group i, and group j, then the closest group pair is the one
				//  with the smallest value of mdp.
				F32 currmin = 2;
				S32 firstGroup = -1;
				S32 secondGroup = -1;

				for (j = 0; j < planeGroups.size(); j++) {
					PlaneGrouping& first = planeGroups[j];
					for (k = j + 1; k < planeGroups.size(); k++) {
						PlaneGrouping& second = planeGroups[k];

						F32 max = -2;
						for (l = 0; l < first.numPlanes; l++) {
							for (m = 0; m < second.numPlanes; m++) {
								glm::vec3 firstNormal = interior.normal[interior.plane[first.planeIndices[l] & ~0x8000].normalIndex];
								if ((first.planeIndices[l] >> 15) != 0)
									firstNormal *= -1;
								glm::vec3 secondNormal = interior.normal[interior.plane[first.planeIndices[m] & ~0x8000].normalIndex];
								if ((second.planeIndices[l] >> 15) != 0)
									secondNormal *= -1;

								F32 dot = glm::dot(firstNormal, secondNormal);
								if (dot > max)
									max = dot;
							}
						}

						if (max < currmin) {
							currmin = max;
							firstGroup = j;
							secondGroup = k;
						}
					}
				}
				assert(firstGroup != -1 && secondGroup != -1, "Error, unable to find a suitable pairing?");

				// Merge first and second
				PlaneGrouping& to = planeGroups[firstGroup];
				PlaneGrouping& from = planeGroups[secondGroup];
				while (from.numPlanes != 0) {
					to.planeIndices[to.numPlanes++] = from.planeIndices[from.numPlanes - 1];
					from.numPlanes--;
				}

				// And remove the merged group
				planeGroups.erase(planeGroups.begin() + secondGroup);
			}
			assert(planeGroups.size() <= 8, "Error, too many plane groupings!");


			// Assign a mask to each of the plane groupings
			for (j = 0; j < planeGroups.size(); j++)
				planeGroups[j].mask = (1 << j);
		}

		// Now, assign the mask to each of the temp polys
		{
			for (j = 0; j < tempSurfaces.size(); j++) {
				bool assigned = false;
				for (k = 0; k < planeGroups.size() && !assigned; k++) {
					for (l = 0; l < planeGroups[k].numPlanes; l++) {
						if (planeGroups[k].planeIndices[l] == tempSurfaces[j].planeIndex) {
							tempSurfaces[j].mask = planeGroups[k].mask;
							assigned = true;
							break;
						}
					}
				}
				assert(assigned, "Error, missed a plane somewhere in the hull poly list!");
			}
		}

		// Copy the appropriate group mask to the plane masks
		{

			for (j = 0; j < planeIndices.size(); j++) {
				bool found = false;
				for (k = 0; k < planeGroups.size() && !found; k++) {
					for (l = 0; l < planeGroups[k].numPlanes; l++) {
						if (planeGroups[k].planeIndices[l] == planeIndices[j]) {
							planeMasks.push_back(planeGroups[k].mask);
							found = true;
							break;
						}
					}
				}
				assert(planeMasks[j] != 0, "Error, missing mask for plane!");
			}
		}

		// And whip through the points, constructing the total mask for that point
		{

			for (j = 0; j < pointIndices.size(); j++) {
				pointMasks.push_back(0);
				for (k = 0; k < tempSurfaces.size(); k++) {
					for (l = 0; l < tempSurfaces[k].numPoints; l++) {
						if (tempSurfaces[k].pointIndices[l] == j) {
							pointMasks[j] |= tempSurfaces[k].mask;
							break;
						}
					}
				}
				assert(pointMasks[j] != 0, "Error, point must exist in at least one surface!");
			}
		}

		// Create the emit strings, and we're done!
		{
			// Set the range of planes
			rHull.polyListPlaneStart = interior.polyListPlaneIndex.size();
			for (j = 0; j < planeIndices.size(); j++)
				interior.polyListPlaneIndex.push_back(planeIndices[j]);

			// Set the range of points
			rHull.polyListPointStart = interior.polyListPointIndex.size();
			for (j = 0; j < pointIndices.size(); j++)
				interior.polyListPointIndex.push_back(pointIndices[j]);

			// Now the emit string.  The emit string goes like: (all fields are bytes)
			//  NumPlanes (PLMask) * NumPlanes
			//  NumPointsHi NumPointsLo (PtMask) * NumPoints
			//  NumSurfaces
			//   (NumPoints SurfaceMask PlOffset (PtOffsetHi PtOffsetLo) * NumPoints) * NumSurfaces
			//
			U32 stringLen = 1 + planeIndices.size();
			stringLen += 2 + pointIndices.size();
			stringLen += 1;
			for (j = 0; j < tempSurfaces.size(); j++)
				stringLen += 1 + 1 + 1 + (tempSurfaces[j].numPoints * 2);

			rHull.polyListStringStart = interior.polyListStringCharacter.size();

			// Planes
			interior.polyListStringCharacter.push_back(planeIndices.size());
			for (j = 0; j < planeIndices.size(); j++)
				interior.polyListStringCharacter.push_back(planeMasks[j]);

			// Points
			interior.polyListStringCharacter.push_back((pointIndices.size() >> 8) & 0xFF);
			interior.polyListStringCharacter.push_back((pointIndices.size() >> 0) & 0xFF);
			for (j = 0; j < pointIndices.size(); j++)
				interior.polyListStringCharacter.push_back(pointMasks[j]);

			// Surfaces
			interior.polyListStringCharacter.push_back(tempSurfaces.size());
			for (j = 0; j < tempSurfaces.size(); j++) {
				interior.polyListStringCharacter.push_back(tempSurfaces[j].numPoints);
				interior.polyListStringCharacter.push_back(tempSurfaces[j].mask);

				bool found = false;
				for (k = 0; k < planeIndices.size(); k++) {
					if (planeIndices[k] == tempSurfaces[j].planeIndex) {
						interior.polyListStringCharacter.push_back(k);
						found = true;
						break;
					}
				}
				assert(found, "Error, missing planeindex!");

				for (k = 0; k < tempSurfaces[j].numPoints; k++) {
					interior.polyListStringCharacter.push_back((tempSurfaces[j].pointIndices[k] >> 8) & 0xFF);
					interior.polyListStringCharacter.push_back((tempSurfaces[j].pointIndices[k] >> 0) & 0xFF);
				}
			}
		}
	} // for (i = 0; i < mConvexHulls.size(); i++)

}

void DIFBuilder::build(DIF& dif, bool flipNormals)
{

	std::vector<Polygon*> polyList;
	PolygonAllocator polygonAlloc;

	for (int i = 0; i < mTriangles.size(); i++)
	{
		Polygon* poly = polygonAlloc.allocate();
		for (int j = 0; j < 3; j++)
		{
			poly->VertexList.push_back(Vertex());
			poly->Indices.push_back(0);
		}
		if (!flipNormals)
		{
			for (int j = 0; j < 3; j++)
				poly->VertexList[j].p = glm::vec3(mTriangles[i].points[j].vertex.x, mTriangles[i].points[j].vertex.y, mTriangles[i].points[j].vertex.z);
		}
		else
		{
			for (int j = 0; j < 3; j++)
				poly->VertexList[2 - j].p = glm::vec3(mTriangles[i].points[j].vertex.x, mTriangles[i].points[j].vertex.y, mTriangles[i].points[j].vertex.z);
		}

		for (int j = 0; j < 3; j++)
			poly->Indices[j] = j;

		poly->TextureIndex = mTriangles[i].material;

		if (!flipNormals)
		{
			for (int j = 0; j < 3; j++)
				poly->VertexList[j].uv = mTriangles[i].points[j].uv;
		}
		else
		{
			for (int j = 0; j < 3; j++)
				poly->VertexList[2 - j].uv = mTriangles[i].points[j].uv;
		}
		if (!flipNormals)
		{
			// poly->plane = Plane(mTriangles[i].points[0].vertex, mTriangles[i].points[0].normal);
			poly->plane = Plane(mTriangles[i].points[0].vertex, mTriangles[i].points[1].vertex, mTriangles[i].points[2].vertex);
			poly->Normal = poly->plane.normal;
		}
		else
		{
			// poly->plane = Plane(mTriangles[i].points[0].vertex, mTriangles[i].points[0].normal);
			poly->plane = Plane(mTriangles[i].points[2].vertex, mTriangles[i].points[1].vertex, mTriangles[i].points[0].vertex);
			poly->Normal = poly->plane.normal;
		}
		polyList.push_back(poly);
	}

	printf("Generating BSP\n");

	BSPNodeAllocator nodeAlloc;

	std::vector<BSPNode*> nodes;
	for (auto& poly : polyList)
	{
		BSPNode* n = nodeAlloc.allocate();
		BSPNode* leafnode = nodeAlloc.allocate();
		leafnode->IsLeaf = true;
		leafnode->poly = poly;
		n->Front = leafnode;
		n->plane = Plane(poly->VertexList[0].p, poly->Normal);
		nodes.push_back(n);
	}

	BSPNode* root = BuildBSPRecurse(nodes, nodeAlloc);

	//for (auto& poly : polyList)
	//{
	//	if (poly.surfaceIndex == -1) {
	//		throw new std::exception("Polygon not used!");
	//	}
	//}

	printf("Gathering primitives\n");
	std::vector<Polygon*> polys = std::vector<Polygon*>();
	std::vector<Polygon*> orderedpolys = std::vector<Polygon*>();
	// polys.clear();
	// GatherBrushes(root, &polys);

	Interior interior = Interior();

	PlaneMap planeIndices;

	//printf("Exporting Surfaces\n");
	//ExportSurfaces(&interior, polys, &planehashes, mMaterials, &pointhashes);
	printf("Exporting BSP\n");
	ExportBSP(&interior, root, &polys, planeIndices, mMaterials, &orderedpolys);

	for (auto& poly : polyList) {
		if (poly->surfaceIndex == -1)
			throw new std::exception("Unused poly");
	}

	//orderedpolys->clear();
	//for (auto& poly : polyList)
	//{
	//	orderedpolys->push_back(&poly);
	//}

	std::vector<std::vector<Polygon*>> groupedPolys = std::vector<std::vector<Polygon*>>();

	int fullpolycount = orderedpolys.size() / 8;
	int rem = orderedpolys.size() % 8;

	for (int i = 0; i < orderedpolys.size() - rem; i += 8)
	{
		std::vector<Polygon*> polysList = std::vector<Polygon*>();
		for (int j = 0; j < 8; j++)
			polysList.push_back(orderedpolys.at(i + j));

		groupedPolys.push_back(polysList);
	}
	std::vector<Polygon*> lastPolys = std::vector<Polygon*>();
	for (int i = orderedpolys.size() - rem; i < orderedpolys.size(); i++)
	{
		lastPolys.push_back(orderedpolys.at(i));
	}
	if (lastPolys.size() != 0)
		groupedPolys.push_back(lastPolys);

	printf("Exporting Convex Hulls\n");
	std::vector<ObjectHash> emitStrHashes;
	ExportConvexHulls(&interior, groupedPolys, planeIndices, &emitStrHashes, this->exportEmitStrings);

	printf("Exporting Convex Hull PolyLists\n");
	buildHullPolyLists(interior);

	printf("Exporting Zones\n");
	Interior::Zone z = Interior::Zone();
	z.portalStart = 0;
	z.portalCount = 0;
	z.surfaceStart = 0;
	z.surfaceCount = interior.surface.size();
	z.flags = 0;
	z.staticMeshStart = 0;
	z.staticMeshCount = 0;

	interior.zone.push_back(z);

	for (int i = 0; i < interior.surface.size(); i++)
		interior.zoneSurface.push_back(i);

	interior.coordBinMode = 0;
	interior.baseAmbientColor = glm::cvec4(1, 1, 1, 1);
	interior.alarmAmbientColor = glm::cvec4(1, 1, 1, 1);
	interior.detailLevel = 0;
	interior.lightMapBorderSize = 0;
	interior.minPixels = 250;
	interior.hasAlarmState = 0;
	interior.numLightStateEntries = 0;
	interior.boundingBox = getBoundingBox();
	interior.boundingSphere = getBoundingSphere();

	printf("Exporting CoordBins\n");

	ExportCoordBins(&interior);

	printf("Exporting PathedInteriors\n");
	for (auto& it : mPathedInteriors)
	{
		int index = dif.subObject.size();
		dif.subObject.push_back(it.first);
		InteriorPathFollower pathedInterior;
		pathedInterior.datablock = std::string("PathedDefault");
		pathedInterior.name = std::string("MustChange");
		pathedInterior.offset = glm::vec3(0, 0, 0);
		pathedInterior.interiorResIndex = index;
		pathedInterior.totalMS = 0;

		for (auto& it2 : it.second)
		{
			InteriorPathFollower::WayPoint marker;
			marker.position = it2.position;
			marker.rotation = glm::quat();
			marker.msToNext = it2.msToNext;
			marker.smoothingType = it2.smoothing;
			pathedInterior.totalMS += it2.msToNext;
			pathedInterior.wayPoint.push_back(marker);
		}

		if (it.second[0].initialPathPosition != -1)
		{
			char buffer[32]; //How big can this number be, should be enough
			sprintf(buffer, "%d", it.second[0].initialPathPosition);
			pathedInterior.properties.push_back(std::pair<std::string, std::string>(std::string("initialPathPosition"), std::string(buffer)));
		}

		char buffer[32];
		sprintf(buffer, "%d", it.second[0].initialTargetPosition);
		pathedInterior.properties.push_back(std::pair<std::string, std::string>(std::string("initialTargetPosition"), std::string(buffer)));

		dif.interiorPathFollower.push_back(pathedInterior);
	}

	printf("Exporting Entities\n");
	dif.gameEntity = mGameEntities;
	dif.readGameEntities = 2;

	printf("Exporting Triggers\n");
	for (auto& it : mTriggers)
	{
		ExportTrigger(&dif, it);
	}

	printf("Finalizing\n");
	//FixPlanes(interior.plane, interior.normal);
	dif.interior.push_back(interior);
}

BoxF DIFBuilder::getBoundingBox() {
	BoxF box(1e8, 1e8, 1e8, -1e8, -1e8, -1e8);
	for (const auto &triangle : mTriangles) {
		box.unionPoint(triangle.points[0].vertex);
		box.unionPoint(triangle.points[1].vertex);
		box.unionPoint(triangle.points[2].vertex);
	}
	return box;
}

SphereF DIFBuilder::getBoundingSphere() {
	BoxF box = getBoundingBox();

	SphereF sphere;
	glm::vec3 center = box.getCenter();
	sphere.x = center.x;
	sphere.y = center.y;
	sphere.z = center.z;
	glm::vec3 rvec = (box.getMax() - box.getCenter());
	sphere.radius = sqrt(rvec.x * rvec.x + rvec.y * rvec.y + rvec.z * rvec.z);
	return sphere;
}

glm::vec3 DIFBuilder::getAverageNormal(const Triangle &triangle) {
	const glm::vec3 &point0 = triangle.points[0].normal;
	const glm::vec3 &point1 = triangle.points[1].normal;
	const glm::vec3 &point2 = triangle.points[2].normal;

	//average
	return (point0 + point1 + point2) / 3.0f;

	//n = (v2 - v1) x (v2 - v0)
//	glm::vec3 crossNorm = glm::normalize(glm::cross(point2 - point1, point2 - point0));

//	return crossNorm;
}

F32 DIFBuilder::getPlaneDistance(const Triangle &triangle, const glm::vec3 &center) {
	//Use the center of the plane, probably correct
	glm::vec3 averagePoint(0);
	const glm::vec3 &point0 = triangle.points[0].vertex;
	const glm::vec3 &point1 = triangle.points[1].vertex;
	const glm::vec3 &point2 = triangle.points[2].vertex;
	averagePoint += point0;
	averagePoint += point1;
	averagePoint += point2;
	averagePoint /= 3.0f;

	//Get the normal
	const glm::vec3 &normal = getAverageNormal(triangle);

	//http://mathworld.wolfram.com/Point-PlaneDistance.html
	//D = n • (x_0 - x_i)
	// Where x_0 = (0, 0, 0)
	return glm::dot(-averagePoint, normal);
}

std::ostream &operator<<(std::ostream &stream, const glm::mat3x4 &input) {
	return stream << std::setw(10) << input[0][0] << " " << std::setw(10) << input[0][1] << " " << std::setw(10) << input[0][2] << " " << std::setw(10) << input[0][3] << std::endl
	              << std::setw(10) << input[1][0] << " " << std::setw(10) << input[1][1] << " " << std::setw(10) << input[1][2] << " " << std::setw(10) << input[1][3] << std::endl
	              << std::setw(10) << input[2][0] << " " << std::setw(10) << input[2][1] << " " << std::setw(10) << input[2][2] << " " << std::setw(10) << input[2][3] << std::endl;
}

//The 3 elementary matrix row operations that you can apply without changing the result
void swapRows(glm::mat3x4& mat, int rowA, int rowB) {
	std::swap(mat[rowA], mat[rowB]);
}
void scaleRow(glm::mat3x4& mat, int row, float scale) {
	mat[row] *= scale;
}
void addRow(glm::mat3x4& mat, int destRow, int srcRow, float factor) {
	mat[destRow] += mat[srcRow] * factor;
}

glm::vec3 solveMatrix(glm::mat3x4 pointMatrix, bool print = false) {
#define MaybePrint(a) if (print) { std::cout << a << std::endl; }

	//Clean up stuff that is almost zero
	MaybePrint(pointMatrix);
	if (closeEnough(pointMatrix[0][0], 0.f)) pointMatrix[0][0] = 0.f;
	if (closeEnough(pointMatrix[1][0], 0.f)) pointMatrix[1][0] = 0.f;
	if (closeEnough(pointMatrix[2][0], 0.f)) pointMatrix[2][0] = 0.f;
	if (closeEnough(pointMatrix[0][1], 0.f)) pointMatrix[0][1] = 0.f;
	if (closeEnough(pointMatrix[1][1], 0.f)) pointMatrix[1][1] = 0.f;
	if (closeEnough(pointMatrix[2][1], 0.f)) pointMatrix[2][1] = 0.f;
	if (closeEnough(pointMatrix[0][2], 0.f)) pointMatrix[0][2] = 0.f;
	if (closeEnough(pointMatrix[1][2], 0.f)) pointMatrix[1][2] = 0.f;
	if (closeEnough(pointMatrix[2][2], 0.f)) pointMatrix[2][2] = 0.f;
	if (closeEnough(pointMatrix[0][3], 0.f)) pointMatrix[0][3] = 0.f;
	if (closeEnough(pointMatrix[1][3], 0.f)) pointMatrix[1][3] = 0.f;
	if (closeEnough(pointMatrix[2][3], 0.f)) pointMatrix[2][3] = 0.f;
	MaybePrint(pointMatrix);

	//For checking at the end
	glm::mat3x4 test = pointMatrix;

	glm::vec3 offset = glm::vec3(0, 0, 0);
	int iteration = 0;
	while (glm::determinant(glm::mat3x3(pointMatrix)) == 0)
	{
		if (iteration > 0)
			printf("Can't find texgen for this specific surface: iteration %d\n", iteration);
		offset += glm::vec3(1, 1, 1);
		pointMatrix[0][0] += offset.x;
		pointMatrix[0][1] += offset.y;
		pointMatrix[0][2] += offset.z;
		pointMatrix[1][0] += offset.x;
		pointMatrix[1][1] += offset.y;
		pointMatrix[1][2] += offset.z;
		pointMatrix[2][0] += offset.x;
		pointMatrix[2][1] += offset.y;
		pointMatrix[2][2] += offset.z;
		iteration++;

		if (iteration >= 10)
		{
			printf("skipping texgen");
		}
	}

	/*
	 We have three simultaneous equations:
	 ax + by + cz = uv0
	 dx + ey + fz = uv1
	 gx + hy + iz = uv2
	 We can arrange them in a matrix like this:
	 [ a b c ]   ( x )   ( uv0 )
	 [ d e f ] x ( y ) = ( uv1 )
	 [ g h i ]   ( z )   ( uv2 )

	 And then if we can get the matrix into reduced echelon form we can solve
	 for (x y z) with no trouble.
	 */

	 //Swap around rows so that we can get something non-zero for [0][0]
	if (closeEnough(pointMatrix[0][0], 0.f)) {
		if (closeEnough(pointMatrix[1][0], 0.f)) {
			swapRows(pointMatrix, 0, 2);
			MaybePrint(pointMatrix);
		}
		else {
			swapRows(pointMatrix, 0, 1);
			MaybePrint(pointMatrix);
		}
	}
	//If all three have zero for [0][0] then we're fine here
	if (!closeEnough(pointMatrix[0][0], 0.f)) {
		/*
		 Reduce second and third rows so the first column is zero
		 To get
		 [ a b c ]
		 [ 0 d e ]
		 [ 0 f g ]
		 */
		if (!closeEnough(pointMatrix[1][0], 0.f)) {
			addRow(pointMatrix, 1, 0, -pointMatrix[1][0] / pointMatrix[0][0]);
			MaybePrint(pointMatrix);
		}
		if (!closeEnough(pointMatrix[2][0], 0.f)) {
			addRow(pointMatrix, 2, 0, -pointMatrix[2][0] / pointMatrix[0][0]);
			MaybePrint(pointMatrix);
		}
	}

	//If mat[1][1] is zero we should swap rows 1 and 2 so we can reduce the other row
	if (closeEnough(pointMatrix[1][1], 0.f)) {
		swapRows(pointMatrix, 1, 2);
		MaybePrint(pointMatrix);
	}
	//If mat[1][1] is zero then both [1][1] and [2][1] are zero and we can continue
	if (!closeEnough(pointMatrix[1][1], 0.f)) {
		/*
		 Reverse third row so the second column is zero
		 To get
		 [ a b c ]
		 [ 0 d e ]
		 [ 0 0 f ]
		 */
		if (!closeEnough(pointMatrix[2][1], 0.f)) {
			addRow(pointMatrix, 2, 1, -pointMatrix[2][1] / pointMatrix[1][1]);
			MaybePrint(pointMatrix);
		}
	}

	/*
	 Scale each of the rows so the first component is one
	 To get
	 [ 1 a b ]
	 [ 0 1 c ]
	 [ 0 0 1 ]
	 */
	if (!closeEnough(pointMatrix[0][0], 0.f, 0.00001f)) {
		scaleRow(pointMatrix, 0, 1.0f / pointMatrix[0][0]);
		MaybePrint(pointMatrix);
	}
	if (!closeEnough(pointMatrix[1][1], 0.f, 0.00001f)) {
		scaleRow(pointMatrix, 1, 1.0f / pointMatrix[1][1]);
		MaybePrint(pointMatrix);
	}
	if (!closeEnough(pointMatrix[2][2], 0.f, 0.00001f)) {
		scaleRow(pointMatrix, 2, 1.0f / pointMatrix[2][2]);
		MaybePrint(pointMatrix);
	}

	/*
	 At this point the matrix is
	 [ 1 a b ]   ( x )   ( uv0' )
	 [ 0 1 c ] x ( y ) = ( uv1' )
	 [ 0 0 1 ]   ( z )   ( uv2' )
	 where
	 x + ay + bz = uv0
		  y + cz = uv1
			   z = uv2
	 therefore
	 z = uv2'
	 y = uv1' - cz
	 x = uv0' - ay - bz
	 */

	 //Convenience
	const glm::vec4& xvec = pointMatrix[0];
	const glm::vec4& yvec = pointMatrix[1];
	const glm::vec4& zvec = pointMatrix[2];

	//These are easy now
	float z = zvec[3];
	float y = yvec[3] - z * yvec[2];
	float x = xvec[3] - y * xvec[1] - z * xvec[2];

	//Check our work
//	assert(closeEnough(x * test[0][0] + y * test[0][1] + z * test[0][2], test[0][3]));
//	assert(closeEnough(x * test[1][0] + y * test[1][1] + z * test[1][2], test[1][3]));
//	assert(closeEnough(x * test[2][0] + y * test[2][1] + z * test[2][2], test[2][3]));

	//And there we go
	return glm::vec3(x, y, z);
}

Interior::TexGenEq getTexGenFromPoints(const glm::vec3& point0, const glm::vec3& point1, const glm::vec3& point2,
	glm::vec2 uv0, glm::vec2 uv1, glm::vec2 uv2) {
	Interior::TexGenEq texGen;

	//Construct these matrices for the solver to figure out
	glm::mat3x4	xTexMat = glm::mat3x4(glm::vec4(point0, uv0.x), glm::vec4(point1, uv1.x), glm::vec4(point2, uv2.x));
	glm::mat3x4 yTexMat = glm::mat3x4(glm::vec4(point0, uv0.y), glm::vec4(point1, uv1.y), glm::vec4(point2, uv2.y));

	//Solving is rather simple
	glm::vec3 xsolve = solveMatrix(xTexMat);
	glm::vec3 ysolve = solveMatrix(yTexMat);

	//Rigorous checking because I don't like being wrong
	if (!closeEnough(xsolve.x * point0.x + xsolve.y * point0.y + xsolve.z * point0.z, uv0.x, 0.001f)) {
		solveMatrix(xTexMat, false);
	}
	if (!closeEnough(xsolve.x * point1.x + xsolve.y * point1.y + xsolve.z * point1.z, uv1.x, 0.001f)) {
		solveMatrix(xTexMat, false);
	}
	if (!closeEnough(xsolve.x * point2.x + xsolve.y * point2.y + xsolve.z * point2.z, uv2.x, 0.001f)) {
		solveMatrix(xTexMat, false);
	}
	if (!closeEnough(ysolve.x * point0.x + ysolve.y * point0.y + ysolve.z * point0.z, uv0.y, 0.001f)) {
		solveMatrix(yTexMat, false);
	}
	if (!closeEnough(ysolve.x * point1.x + ysolve.y * point1.y + ysolve.z * point1.z, uv1.y, 0.001f)) {
		solveMatrix(yTexMat, false);
	}
	if (!closeEnough(ysolve.x * point2.x + ysolve.y * point2.y + ysolve.z * point2.z, uv2.y, 0.001f)) {
		solveMatrix(yTexMat, false);
	}

	//And there we go
	texGen.planeX.x = xsolve.x;
	texGen.planeX.y = xsolve.y;
	texGen.planeX.z = xsolve.z;
	texGen.planeY.x = ysolve.x;
	texGen.planeY.y = ysolve.y;
	texGen.planeY.z = ysolve.z;

	return texGen;
}

Interior::TexGenEq DIFBuilder::getTexGen(const Triangle &triangle) {
	return getTexGenFromPoints(triangle.points[0].vertex,
							   triangle.points[1].vertex,
							   triangle.points[2].vertex,
							   triangle.points[0].uv,
							   triangle.points[1].uv,
							   triangle.points[2].uv);
}

DIF_NAMESPACE_END
