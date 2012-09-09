//
//  KinectDraw.cpp
//  Carver
//
//  Created by paulobala on 14/05/12.
//  Copyright (c) 2012 __MyCompanyName__. All rights reserved.
//
#include "kinectDrawArea.h"

#include <iostream>

ofVec3f getNormal(Triangle& triangle) {
	ofVec3f a = triangle.vert1 - triangle.vert2;
	ofVec3f b = triangle.vert3 - triangle.vert2;
	ofVec3f normal = b.cross(a);
	normal.normalize();
	return normal;
}

void calculateNormals(vector<Triangle>& triangles, vector<ofVec3f>& normals) {
	normals.resize(triangles.size() * 3);
	
	int j = 0;
	ofVec3f normal;
	for(int i = 0; i < triangles.size(); i++) {
		normal = getNormal(triangles[i]);
		for(int m = 0; m < 3; m++) {
			normals[j++] = normal;
		}
	}
}

const float FovH=1.0144686707507438;
const float FovV=0.78980943449644714;
const float XtoZ = tanf(FovH/2)*2;
const float YtoZ = tanf(FovV/2)*2;
const unsigned int Xres = 640;
const unsigned int Yres = 480;

int getSurfaceIndex(int x, int y) {
	return y * Xres + x;
}
//
//ofVec3f testApp::getSurface(XYZ& position) {
//	return surface[position.y * Xres + position.x];
//}

ofVec3f ConvertProjectiveToRealWorld(float x, float y, float z) {
	return ofVec3f((x/Xres-.5f) * z * XtoZ,
                   (.5f-y/Yres) * z * YtoZ,
                   z);
}
