
#ifndef tCAD_triangle_h
#define tCAD_triangle_h
#include "ofVec3f.h"

class TCADTriangle {
public:
	ofVec3f vert1, vert2, vert3;
	TCADTriangle() {
	}
	TCADTriangle(ofVec3f vert1_, ofVec3f vert2_, ofVec3f vert3_):
	vert1(vert1_), vert2(vert2_), vert3(vert3_) {
	}
};

#endif
