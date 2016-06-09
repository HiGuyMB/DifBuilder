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

DIF_NAMESPACE

DIFBuilder::DIFBuilder(const DIF &dif) : mScale(1.0f) {
	//TODO: Extract triangles
}

void DIFBuilder::addTriangle(const Triangle &triangle) {
	mTriangles.push_back(triangle);
}

void DIFBuilder::addTriangle(const Triangle &triangle, const std::string &material) {
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

void DIFBuilder::build(DIF &dif) {
	typedef U32 PointAssocIndex;
	typedef U32 DifPointIndex;

	std::map<PointAssocIndex, DifPointIndex> pointAssoc;

	Interior interior;
	F32 scale = 0.1;

	interior.detailLevel = 0;
	interior.minPixels = 250;
	interior.boundingBox = getBoundingBox();
	interior.boundingSphere = getBoundingSphere();

	interior.hasAlarmState = 0;
	interior.numLightStateEntries = 0;

	for (const auto &material : mMaterials) {
		interior.materialName.push_back(material);
	}

	//TODO: this seems wrong (way too easy)
	for (U32 i = 0; i < 256; i ++) {
		Interior::CoordBin bin;
		bin.binStart = i;
		bin.binCount = 1;
		interior.coordBin.push_back(bin);
		interior.coordBinIndex.push_back(0);
	}
	interior.coordBinMode = 0;

	interior.baseAmbientColor.a = 255;
	interior.alarmAmbientColor.a = 255;

	interior.extendedLightMapData = 0;
	interior.lightMapBorderSize = 0;

	//Add vertices to the interior
	for (const Triangle &triangle : mTriangles) {
		interior.point.push_back(triangle.points[0].vertex * scale);
		interior.point.push_back(triangle.points[1].vertex * scale);
		interior.point.push_back(triangle.points[2].vertex * scale);
	}

	//TODO: Multiple zones?
	Interior::Zone zone;
	zone.portalStart = 0;
	zone.portalCount = 0;
	zone.surfaceStart = interior.zoneSurface.size();
	zone.surfaceCount = mTriangles.size();
	zone.staticMeshStart = 0;
	zone.staticMeshCount = 0;
	zone.flags = 0;

	//TODO: Multiple convex hulls to speed up stuff?
	BoxF box = getBoundingBox();
	Interior::ConvexHull hull;
	hull.hullStart = interior.hullIndex.size();
	hull.hullCount = mTriangles.size();
	hull.minX = box.minX;
	hull.minY = box.minY;
	hull.minZ = box.minZ;
	hull.maxX = box.maxX;
	hull.maxY = box.maxY;
	hull.maxZ = box.maxZ;
	hull.surfaceStart = interior.solidLeafSurface.size();
	hull.surfaceCount = mTriangles.size();
	hull.planeStart = 0;
	hull.polyListPlaneStart = 0;
	hull.polyListPointStart = 0;
	hull.polyListStringStart = 0;

	interior.convexHull.push_back(hull);

	//TODO: Figure out how their BSP system works
	Interior::BSPSolidLeaf leaf;
	leaf.surfaceIndex = interior.solidLeafSurface.size();
	leaf.surfaceCount = mTriangles.size();

	interior.bspSolidLeaf.push_back(leaf);

	glm::vec3 center = getBoundingBox().getCenter();

	//Add surfaces
	for (U32 triIndex = 0; triIndex < mTriangles.size(); triIndex ++) {
		const Triangle &triangle = mTriangles[triIndex];
		U32 assoc0 = triIndex * 3 + 0;
		U32 assoc1 = triIndex * 3 + 1;
		U32 assoc2 = triIndex * 3 + 2;

		//Averages for normals and planes
		glm::vec3 averageNormal = getAverageNormal(triangle);
		F32 distance = getPlaneDistance(triangle, center);

		Interior::Plane plane;
		plane.normalIndex = interior.normal.size();
		plane.planeDistance = distance;

		//BSP Node
		Interior::BSPNode bsp;
		//TODO: This is really, really inefficient. It puts everything into one
		// giant BSP tree.
		bsp.planeIndex = ((interior.plane.size() + mTriangles.size() - triIndex) - triIndex);
		bsp.frontIndex = 0x8000;
		bsp.backIndex = (triIndex == mTriangles.size() - 1 ? 0xC000 : interior.plane.size() + 1);

		Interior::TexGenEq texGen = getTexGen(triangle);

		//Create a surface
		Interior::Surface surface;
		surface.windingStart = interior.index.size();
		surface.windingCount = 3;
		surface.planeIndex = interior.plane.size();
		surface.textureIndex = 0; //todo: more textures?
		surface.texGenIndex = interior.texGenEq.size();
		surface.surfaceFlags = 16; //todo: maybe we have some of these
		surface.fanMask = 7; //todo: ??

		//Lightmap stuff
		surface.lightMap.finalWord = 0;
		surface.lightMap.texGenXDistance = 0;
		surface.lightMap.texGenYDistance = 0;
		surface.lightCount = 0;
		surface.lightStateInfoStart = 0;
		surface.mapOffsetX = 0;
		surface.mapOffsetY = 0;
		surface.mapSizeX = 0;
		surface.mapSizeY = 0;
		surface.brushId = 0;

		//TODO: No idea what these are
		interior.hullPlaneIndex.push_back(interior.plane.size());
		interior.hullSurfaceIndex.push_back(interior.surface.size());
		interior.hullIndex.push_back(assoc0);
		interior.hullIndex.push_back(assoc1);
		interior.hullIndex.push_back(assoc2);

		//Windings
		interior.index.push_back(assoc0);
		interior.index.push_back(assoc1);
		interior.index.push_back(assoc2);

		//TODO: What are these?
		interior.polyListPlaneIndex.push_back(interior.plane.size());
		interior.polyListPointIndex.push_back(assoc0);
		interior.polyListPointIndex.push_back(assoc1);
		interior.polyListPointIndex.push_back(assoc2);

		interior.texGenEq.push_back(texGen);

		//TODO: Figure these out too
		interior.zoneSurface.push_back(interior.surface.size());
		interior.solidLeafSurface.push_back(interior.surface.size());

		interior.normal.push_back(averageNormal);
		interior.plane.push_back(plane);
		interior.pointVisibility.push_back(-1);
		interior.normalLMapIndex.push_back(0);
		interior.alarmLMapIndex.push_back(-1);
		interior.bspNode.push_back(bsp);
		interior.surface.push_back(surface);
	}

	interior.zone.push_back(zone);

	//TODO: Need to solve these too.

	//These all need at least one item to not crash
	interior.polyListPlaneIndex.push_back(0);
	interior.polyListPointIndex.push_back(0);
	interior.polyListStringCharacter.push_back(0);
	interior.hullEmitStringIndex.push_back(0);
	interior.convexHullEmitStringCharacter.push_back(0);

	dif.interior.push_back(interior);
}

BoxF DIFBuilder::getBoundingBox() {
	BoxF box(1e8, 1e8, 1e8, -1e8, -1e8, -1e8);
	for (const auto &triangle : mTriangles) {
		box.unionPoint(triangle.points[0].vertex * mScale);
		box.unionPoint(triangle.points[1].vertex * mScale);
		box.unionPoint(triangle.points[2].vertex * mScale);
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
	sphere.radius = (box.getMax() - box.getCenter()).length();
	return sphere;
}

glm::vec3 DIFBuilder::getAverageNormal(const Triangle &triangle) {
	const glm::vec3 &point0 = triangle.points[0].vertex;
	const glm::vec3 &point1 = triangle.points[1].vertex;
	const glm::vec3 &point2 = triangle.points[2].vertex;

	//n = (v2 - v1) x (v2 - v0)
	glm::vec3 crossNorm = glm::normalize(glm::cross(point2 - point1, point2 - point0));

	return crossNorm;
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
	return glm::dot(-averagePoint * mScale, normal);
}

glm::vec3 solveSystem(F32 a, F32 b, F32 c, F32 d, F32 e, F32 f, F32 g, F32 h, F32 i, F32 j, F32 k, F32 l) {
	/*
	 Solve the system

	 { ax + by + cz = d
	 { ex + fy + gz = h
	 { ix + jy + kz = l

	 For (x, y, z)

	 Solved this out on paper, should work as long as a is nonzero and a few other
	 things work out to nonzero.
	 */

	//Slight optimization, so we don't have to do a bunch of divisions
	F32 _a = 1.0f / a;
	F32 _f_sub_eb_a = 1.0f / (f - (e * b * _a));

	F32 zTop = (l - (i * d * _a)) - (((j - (i * b * _a)) * (h - (e * d * _a))) * _f_sub_eb_a);
	F32 zBot = (((j - (i * b * _a)) * ((e * c * _a) - g)) * _f_sub_eb_a) + (k - (i * c * _a));
	F32 z = zTop / zBot;

	F32 y = ((h - (e * d * _a)) + (((e * c * _a) - g) * z)) * _f_sub_eb_a;

	F32 x = (d - (b * y) - (c * z)) * _a;

	return glm::vec3(x, y, z);
}

Interior::TexGenEq getTexGenFromPoints(const glm::vec3 &point0, const glm::vec3 &point1, const glm::vec3 &point2,
								  glm::vec2 uv0, glm::vec2 uv1, glm::vec2 uv2) {
	Interior::TexGenEq texGen;

	glm::vec3 xsolve = solveSystem(point0.x, point0.y, point0.z, uv0.x, point1.x, point1.y, point1.z, uv1.x, point2.x, point2.y, point2.z, uv2.x);
	glm::vec3 ysolve = solveSystem(point0.x, point0.y, point0.z, uv0.y, point1.x, point1.y, point1.z, uv1.y, point2.x, point2.y, point2.z, uv2.y);
	texGen.planeX.x = xsolve.x;
	texGen.planeX.y = xsolve.y;
	texGen.planeX.z = xsolve.z;
	texGen.planeY.x = ysolve.x;
	texGen.planeY.y = ysolve.y;
	texGen.planeY.z = ysolve.z;

	return texGen;
}

Interior::TexGenEq DIFBuilder::getTexGen(const Triangle &triangle) {
	return getTexGenFromPoints(triangle.points[0].vertex * mScale,
							   triangle.points[1].vertex * mScale,
							   triangle.points[2].vertex * mScale,
							   triangle.points[0].uv,
							   triangle.points[1].uv,
							   triangle.points[2].uv);
}

DIF_NAMESPACE_END
