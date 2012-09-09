#include "ofMeshtCAD.h"

void ofMeshtCAD::addFace(ofVec3f a, ofVec3f b, ofVec3f c) {
    ofVec3f normal = ((c - a).cross(b - a)).normalize();
	addNormal(normal);
	addVertex(a);
	addNormal(normal);
	addVertex(b);
	addNormal(normal);
	addVertex(c);
    addIndex(getNumIndices());
    addIndex(getNumIndices());
    addIndex(getNumIndices());    
}

void ofMeshtCAD::addFace(ofVec3f a, ofVec3f b, ofVec3f c, ofVec3f d) {
    addFace(a, b, d);
	addFace(b, c, d);
}

void ofMeshtCAD::addFace(ofRectangle r) {
    ofVec2f lt(r.x,r.y);
    ofVec2f rt(r.x+r.width,r.y);
    ofVec2f rb(r.x+r.width,r.y+r.height);
    ofVec2f lb(r.x,r.y+r.height);
    addFace(lt,lb,rb,rt);
}

void ofMeshtCAD::addBox(ofRectangle r, float height) {
    
    ofVec3f a(r.x,r.y);
    ofVec3f b(r.x,r.y+r.height);
    ofVec3f c(r.x+r.width,r.y+r.height);
    ofVec3f d(r.x+r.width,r.y);
    ofVec3f e = a+ofVec3f(0,0,height);
    ofVec3f f = b+ofVec3f(0,0,height);
    ofVec3f g = c+ofVec3f(0,0,height);
    ofVec3f h = d+ofVec3f(0,0,height);
    ofMeshtCAD mesh;
    mesh.addFace(a,b,c,d); //top
    mesh.addFace(h,g,f,e); //bottom
    mesh.addFace(b,f,g,c); //front
    mesh.addFace(c,g,h,d); //right
    mesh.addFace(d,h,e,a); //back
    mesh.addFace(a,e,f,b); //left
    
    addMesh(mesh);
}

void ofMeshtCAD::translate(ofVec3f pos) {
    for (int i=0; i<getNumVertices(); i++) {
        getVertices()[i] += pos;
    }
}

void ofMeshtCAD::translate(float x, float y, float z) {
    for (int i=0; i<getNumVertices(); i++) {
        getVertices()[i] += ofVec3f(x,y,z);
    }
}

void ofMeshtCAD::rotate(float angle, ofVec3f axis) {
    for (int i=0; i<getNumVertices(); i++) {
        getVertices()[i].rotate(angle, axis);
    }
}

void ofMeshtCAD::scale(float x, float y, float z) {
    ofVec3f centroid = getCentroid();
    for (int i=0; i<getNumVertices(); i++) {
        getVertices()[i].x *= x;
        getVertices()[i].y *= y;
        getVertices()[i].z *= z;
    }
    ofVec3f newCentroid = getCentroid();
    newCentroid = centroid - newCentroid;
    translate(newCentroid);
}

void ofMeshtCAD::invertNormals() {
    for (int i=0; i<getNumNormals(); i++) {
        getNormals()[i] *= -1;
    }
}

ofMeshtCAD &ofMeshtCAD::addMesh(ofMesh b) {
    
    int numVertices = getNumVertices();    
    int numIndices = getNumIndices();
    
    //add b
    addVertices(b.getVertices());
    addNormals(b.getNormals());
    addIndices(b.getIndices());
    
    //shift indices for b
    for (int i=0; i<b.getNumIndices(); i++) {
        getIndices()[numIndices+i] += numVertices;
    }
    
    return *this;
}

ofVec3f ofMeshtCAD::getCentroid() {
	if(getNumVertices() == 0) {
		//ofLogWarning() << "Called ofMesh::getCentroid() on mesh with zero vertices, returned ofPoint(0, 0, 0)";
		return ofPoint(0, 0, 0);
	}
    
	ofVec3f runningAverage =  getVertices()[0];
	for(unsigned long int v = 1; v < (unsigned long int)getNumVertices() ; v++){
		unsigned long int contributingVertexCount = v + 1;
		runningAverage = runningAverage * float(v) / float(contributingVertexCount) +  getVertices()[v] * 1.0 / float(contributingVertexCount);
	}
	return runningAverage;
}

vector<TCADTriangle> ofMeshtCAD::getTriangles(){
    vector<TCADTriangle> temp = vector<TCADTriangle>();
    
    for(int i = 0; i < getNumVertices(); i= i+3){
        temp.push_back(TCADTriangle(getVertices()[i], getVertices()[i+1], getVertices()[i+2]));
    }
    
    return temp;
}

void ofMeshtCAD::ofDrawAxis(float size) {
    ofVec3f centroid = getCentroid();
    ofVec3f centroidx1, centroidy1, centroidz1,centroidx2,centroidy2, centroidz2;
    centroidx1 = centroidy1 = centroidz1= centroidx2=centroidy2= centroidz2 = centroid;
    centroidx1.x += size;
    centroidy1.y += size;
    centroidz1.z += size;
    centroidx2.x -= size;
    centroidy2.y -= size;
    centroidz2.z -= size;
	//ofPushStyle();
	ofSetLineWidth(3);
    ofSetColor(ofColor::green);
    
	//draw x axis
    	ofSetColor(ofColor::red);
    	ofLine(centroidx1, centroidx2);
    	
    	// draw y axis
    	ofSetColor(ofColor::green);
    	ofLine(centroidy1, centroidy2);
       
    	// draw z axis
    	ofSetColor(ofColor::blue);
    	ofLine(centroidz1,centroidz2);
	
	//ofPopStyle();
}

vector<float> ofMeshtCAD::ofLimits(){
    
    vector<float> limits;
    
    vector<ofPoint> points = getVertices();
    if (points.size()<1) return limits;
    float xMin=9999,xMax=-9999, yMin=9999, yMax=-9999, zMin = 9999, zMax =-9999;
    for (int i=0; i<points.size(); i++) {
        ofPoint &pt = points[i];
        xMin = min(xMin,pt.x);
        xMax = max(xMax,pt.x);
        yMin = min(yMin,pt.y);
        yMax = max(yMax,pt.y);
        zMin = min(zMin,pt.z);
        zMax = max(zMax,pt.z);
    }
   
    limits.push_back(xMin);  
    limits.push_back(xMax); 
    limits.push_back(yMin); 
    limits.push_back(yMax); 
    limits.push_back(zMin); 
    limits.push_back(zMax); 
    
    return limits;
}
