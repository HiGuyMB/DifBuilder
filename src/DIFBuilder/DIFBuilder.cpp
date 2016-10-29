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

	for (auto &triangle : mTriangles) {
		triangle.points[0].vertex *= mScale;
		triangle.points[1].vertex *= mScale;
		triangle.points[2].vertex *= mScale;
	}

	std::map<PointAssocIndex, DifPointIndex> pointAssoc;

	Interior interior;

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
		interior.point.push_back(triangle.points[0].vertex);
		interior.point.push_back(triangle.points[1].vertex);
		interior.point.push_back(triangle.points[2].vertex);
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
	return glm::dot(-averagePoint, normal);
}

//Because floating point math sucks
template<typename T>
bool closeEnough(const T &p1, const T &p2) {
	return glm::distance(p1, p2) < 0.0001f;
}

//The 3 elementary matrix row operations that you can apply without changing the result
void swapRows(glm::mat3x4 &mat, int rowA, int rowB) {
	std::swap(mat[rowA], mat[rowB]);
}
void scaleRow(glm::mat3x4 &mat, int row, float scale) {
	mat[row] *= scale;
}
void addRow(glm::mat3x4 &mat, int destRow, int srcRow, float factor) {
	mat[destRow] += mat[srcRow] * factor;
}

glm::vec3 solveMatrix(glm::mat3x4 pointMatrix) {
	//Clean up stuff that is almost zero
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

	//For checking at the end
	glm::mat3x4 test = pointMatrix;

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
		} else {
			swapRows(pointMatrix, 0, 1);
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
		}
		if (!closeEnough(pointMatrix[2][0], 0.f)) {
			addRow(pointMatrix, 2, 0, -pointMatrix[2][0] / pointMatrix[0][0]);
		}
	}

	//If mat[1][1] is zero we should swap rows 1 and 2 so we can reduce the other row
	if (closeEnough(pointMatrix[1][1], 0.f)) {
		swapRows(pointMatrix, 1, 2);
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
		}
	}

	/*
	 Scale each of the rows so the first component is one
	 To get
	 [ 1 a b ]
	 [ 0 1 c ]
	 [ 0 0 1 ]
	 */
	if (!closeEnough(pointMatrix[0][0], 0.f)) {
		scaleRow(pointMatrix, 0, 1.0f / pointMatrix[0][0]);
	}
	if (!closeEnough(pointMatrix[1][1], 0.f)) {
		scaleRow(pointMatrix, 1, 1.0f / pointMatrix[1][1]);
	}
	if (!closeEnough(pointMatrix[2][2], 0.f)) {
		scaleRow(pointMatrix, 2, 1.0f / pointMatrix[2][2]);
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
	const glm::vec4 &xvec = pointMatrix[0];
	const glm::vec4 &yvec = pointMatrix[1];
	const glm::vec4 &zvec = pointMatrix[2];

	//These are easy now
	float z = zvec[3];
	float y = yvec[3] - z * yvec[2];
	float x = xvec[3] - y * xvec[1] - z * xvec[2];

	//Check our work
	assert(closeEnough(x * test[0][0] + y * test[0][1] + z * test[0][2], test[0][3]));
	assert(closeEnough(x * test[1][0] + y * test[1][1] + z * test[1][2], test[1][3]));
	assert(closeEnough(x * test[2][0] + y * test[2][1] + z * test[2][2], test[2][3]));

	//And there we go
	return glm::vec3(x, y, z);
}

Interior::TexGenEq getTexGenFromPoints(const glm::vec3 &point0, const glm::vec3 &point1, const glm::vec3 &point2,
								  glm::vec2 uv0, glm::vec2 uv1, glm::vec2 uv2) {
	Interior::TexGenEq texGen;

	//Construct these matrices for the solver to figure out
	glm::mat3x4	xTexMat = glm::mat3x4(glm::vec4(point0, uv0.x), glm::vec4(point1, uv1.x), glm::vec4(point2, uv2.x));
	glm::mat3x4 yTexMat = glm::mat3x4(glm::vec4(point0, uv0.y), glm::vec4(point1, uv1.y), glm::vec4(point2, uv2.y));

	//Solving is rather simple
	glm::vec3 xsolve = solveMatrix(xTexMat);
	glm::vec3 ysolve = solveMatrix(yTexMat);

	//Rigorous checking because I don't like being wrong
	assert(closeEnough(xsolve.x * point0.x + xsolve.y * point0.y + xsolve.z * point0.z, uv0.x));
	assert(closeEnough(xsolve.x * point1.x + xsolve.y * point1.y + xsolve.z * point1.z, uv1.x));
	assert(closeEnough(xsolve.x * point2.x + xsolve.y * point2.y + xsolve.z * point2.z, uv2.x));
	assert(closeEnough(ysolve.x * point0.x + ysolve.y * point0.y + ysolve.z * point0.z, uv0.y));
	assert(closeEnough(ysolve.x * point1.x + ysolve.y * point1.y + ysolve.z * point1.z, uv1.y));
	assert(closeEnough(ysolve.x * point2.x + ysolve.y * point2.y + ysolve.z * point2.z, uv2.y));

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
