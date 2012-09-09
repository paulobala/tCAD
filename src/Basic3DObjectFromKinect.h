//
//  KinectDraw.h
//  Carver
//
//  Created by paulobala on 14/05/12.
//  Copyright (c) 2012 __MyCompanyName__. All rights reserved.
//

#ifndef Carver_KinectDraw_h
#define Carver_KinectDraw_h

#include "ofEasyFingerCam.h"
#include "ofxDelaunay.h"
#include "kinectDrawVariables.h"
#include "ofMeshtCAD.h"

class Basic3DObjectFromKinect : public Basic3DObject{
protected:
    vector<ofVec3f> surface;
	vector<TCADTriangle> triangles;
	vector<ofVec3f> normals;
	
	vector<TCADTriangle> backTriangles;
	vector<ofVec3f> backNormals;
    ofVec3f offset;
	float backOffset;
	float zCutoff;
	float globalScale;
	ofVec2f roiStart, roiEnd;
	ofxDelaunay triangulator;
    float FovH;
    float FovV;
    float XtoZ;
    float YtoZ;
    unsigned int Xres;
    unsigned int Yres;
    vector<ofVec2f> points;
    cv::Mat sobelxy;
    cv::Mat sobelbox;
    int attempts;
public:
    ofxKinect * kinect;
    ofVec3f * entry;
   
    kinectDrawVariables kinect_DrawVariables;
    
    Basic3DObjectFromKinect(ofVec3f * entryPoint, ofxKinect * kinect_):Basic3DObject()
    {
        FovH=1.0144686707507438;
        FovV=0.78980943449644714;
        XtoZ = tanf(FovH/2)*2;
        YtoZ = tanf(FovV/2)*2;
        Xres = 640;
        Yres = 480;
        kinect_DrawVariables = kinectDrawVariables();
        kinect = kinect_;
        entry = new ofVec3f(entryPoint->x, entryPoint->y, entryPoint->z); 
        surface = vector<ofVec3f>();
        surface.resize(Xres * Yres);
        triangles= vector<TCADTriangle>();
        normals = vector<ofVec3f>();
        backTriangles = vector<TCADTriangle>();
        backNormals = vector<ofVec3f>();
        backTriangles.resize(2 * (((Xres - 1) + (Yres - 1)) * 2 + 1));
        backNormals.resize(triangles.size() * 3);
        mesh.setMode(OF_PRIMITIVE_TRIANGLES);
    }
    
    
    ofVec3f getNormal(ofVec3f& a, ofVec3f& b, ofVec3f& c) {
        ofVec3f side1 = a - b;
        ofVec3f side2 = c - b;
        ofVec3f normal = side1.cross(side2);
        normal.normalize();
        return normal;
    }
    
    void update1() {
            int width = kinect->getWidth();
            int height = kinect->getHeight();
            float* distancePixels = kinect->getDistancePixels(); // distance in centimeters
            mesh.clear();
            mesh.setMode(OF_PRIMITIVE_TRIANGLES);
            for(int y = 0; y < height - 1; y++) { // don't go to the end
                for(int x = 0; x < width - 1; x++) { // don't go to the end
                    
                    // get indices for each corner
                    int nwi = (y + 0) * width + (x + 0);
                    int nei = (y + 0) * width + (x + 1);
                    int sei = (y + 1) * width + (x + 1);
                    int swi = (y + 1) * width + (x + 0);
                    
                    // get z values for each corner
                    float nwz = distancePixels[nwi];
                    float nez = distancePixels[nei];
                    float sez = distancePixels[sei];
                    float swz = distancePixels[swi];
                    
                    if(nwz > 0 && nez > 0 && sez > 0 && swz > 0) { // ignore empty depth pixels
                                                                   // get real world locations for each corner
                        ofVec3f nwv = ConvertProjectiveToRealWorld(x + 0, y + 0, nwz);
                        ofVec3f nev = ConvertProjectiveToRealWorld(x + 1, y + 0, nez);
                        ofVec3f sev = ConvertProjectiveToRealWorld(x + 1, y + 1, sez);
                        ofVec3f swv = ConvertProjectiveToRealWorld(x + 0, y + 1, swz);
                        
                        // compute normal for the upper left
                        ofVec3f normal = getNormal(nwv, nev, swv);
                        
                        // add the upper left triangle
                        mesh.addNormal(normal);
                        mesh.addVertex(nwv);
                        mesh.addNormal(normal);
                        mesh.addVertex(nev);
                        mesh.addNormal(normal);
                        mesh.addVertex(swv);
                        
                        // add the bottom right triangle
                        mesh.addNormal(normal);
                        mesh.addVertex(nev);
                        mesh.addNormal(normal);
                        mesh.addVertex(sev);
                        mesh.addNormal(normal);
                        mesh.addVertex(swv);
                    }
                }
            }
            ofVec3f centroid = mesh.getCentroid();
            ofVec3f entry2;
            entry2.x = entry->x;
            entry2.y = entry->y;    
            entry2.z = entry->z;
            
            centroid = entry2 - centroid;
            mesh.translate(centroid);
            mesh.rotate(-90, ofVec3f(1,0,0));
        }
    
    
    void update() {	
                        
            zCutoff = kinect_DrawVariables.zCutoff;
            
            float fovWidth = kinect_DrawVariables.fovWidth;
            float fovHeight = kinect_DrawVariables.fovHeight;
            int left = Xres * (1 - fovWidth) / 2;
            
            int top = Yres * (1 - fovHeight) / 2;
            int right = left + Xres * fovWidth;
            int bottom = top + Yres * fovHeight;
                         roiStart = ofVec2f(left, top);
            roiEnd = ofVec2f(right, bottom);
            
            ofVec3f nw = ConvertProjectiveToRealWorld(roiStart.x, roiStart.y, zCutoff);
            ofVec3f se = ConvertProjectiveToRealWorld(roiEnd.x - 1, roiEnd.y - 1, zCutoff);
            float width = (se - nw).x;
            float height = (se - nw).y;
            globalScale = 0.6;
            
            backOffset = kinect_DrawVariables.backOffset / globalScale;
            
            cutoffKinect();
            
            if(kinect_DrawVariables.useSmoothing) {
                smoothKinect();
            }
            
            updateSurface();
            if(kinect_DrawVariables.useRandom) {
                updateTrianglesRandom();
            } else if(kinect_DrawVariables.useSimplify) {
                updateTrianglesSimplify();
            } else {
                updateTriangles();
            }
            calculateNormals(triangles, normals);
            updateBack();
            postProcess();
            
            mesh.clear();
                for (vector<TCADTriangle>::iterator it=triangles.begin() ; it < triangles.end(); it++ ){
                    mesh.addFace((*it).vert1, (*it).vert2, (*it).vert3);
                }
                for (vector<TCADTriangle>::iterator it=backTriangles.begin() ; it < backTriangles.end(); it++ ){
                    mesh.addFace((*it).vert1, (*it).vert2, (*it).vert3);
                }
            mesh.invertNormals();
            
            ofVec3f centroid = mesh.getCentroid();
            ofVec3f entry2;
            entry2.x = entry->x;
            entry2.y = entry->y;    
            entry2.z = entry->z;
            
            centroid = entry2 - centroid;
            mesh.translate(centroid);
            mesh.rotate(-90, ofVec3f(1,0,0));
            
    }
    
    carve::mesh::MeshSet<3> * toMeshSet(){
        
        std::vector< carve::mesh::MeshSet<3>::face_t *> faces = std::vector<carve::mesh::MeshSet<3>::face_t *>();
        
        vector<TCADTriangle> triangles = mesh.getTriangles();
        
        for (vector<TCADTriangle>::iterator it=triangles.begin() ; it < triangles.end(); it++ ){
            
            std::vector< carve::mesh::MeshSet<3>::vertex_t *> v =  std::vector< carve::mesh::MeshSet<3>::vertex_t *>();
            
            carve::mesh::MeshSet<3>::vertex_t * v1 = new carve::mesh::MeshSet<3>::vertex_t(carve::geom::VECTOR( (*it).vert1.x, (*it).vert1.y,(*it).vert1.z));
            carve::mesh::MeshSet<3>::vertex_t * v2 = new carve::mesh::MeshSet<3>::vertex_t(carve::geom::VECTOR((*it).vert2.x, (*it).vert2.y,(*it).vert2.z));
            carve::mesh::MeshSet<3>::vertex_t * v3 = new carve::mesh::MeshSet<3>::vertex_t(carve::geom::VECTOR((*it).vert3.x, (*it).vert3.y,(*it).vert3.z));
            
            v.push_back(v1);
            v.push_back(v2);
            v.push_back(v3);
            
            carve::mesh::MeshSet<3>::face_t * temp = new carve::mesh::MeshSet<3>::face_t::Face(v.begin(), v.end());                 
            faces.push_back(temp);      
            
        }
        
        return new carve::mesh::MeshSet<3>(faces);
        
    }

    void meshSetToMesh(carve::mesh::MeshSet<3> * result){
        
        for (carve::mesh::MeshSet<3>::face_iter face_iter = result->faceBegin(); face_iter != result->faceEnd(); ++face_iter) {
            carve::mesh::MeshSet<3>::face_t *fa = *face_iter;
            
            std::vector<carve::mesh::MeshSet<3>::vertex_t *> verts;
            fa->getVertices(verts);
            
            ofVec3f v1 = ofVec3f(verts[0]->v.x, verts[0]->v.y, verts[0]->v.z);
            ofVec3f v2 = ofVec3f( verts[1]->v.x, verts[1]->v.y, verts[1]->v.z);
            ofVec3f v3 =   ofVec3f( verts[2]->v.x, verts[2]->v.y, verts[2]->v.z);	
            
            mesh.addFace(v1,v2,v3);
            
        }    
    }
    
        
    ofVec3f getNormal(TCADTriangle& triangle) {
        ofVec3f a = triangle.vert1 - triangle.vert2;
        ofVec3f b = triangle.vert3 - triangle.vert2;
        ofVec3f normal = b.cross(a);
        normal.normalize();
        return normal;
    }
    
    void calculateNormals(vector<TCADTriangle>& triangles, vector<ofVec3f>& normals) {
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
    
   
    
    int getSurfaceIndex(int x, int y) {
        return y * Xres + x;
    }
    
    ofVec3f getSurface(XYZ& position) {
        return surface[position.y * Xres + position.x];
    }
    
    ofVec3f ConvertProjectiveToRealWorld(float x, float y, float z) {
        return ofVec3f((x/Xres-.5f) * z * XtoZ,
                       (.5f-y/Yres) * z * YtoZ,
                       z);
    }

    
    void postProcess(ofVec3f& vert, float scale) {
        vert.z *= -1;
        vert.x *= -1;
        vert.z += zCutoff + backOffset;
        vert *= scale;
    }
    
    void postProcess(vector<TCADTriangle>& triangles, float scale) {
        for(int i = 0; i < triangles.size(); i++) {
            TCADTriangle& cur = triangles[i];
            postProcess(cur.vert1, scale);
            postProcess(cur.vert2, scale);
            postProcess(cur.vert3, scale);
        }
    }
    
    void postProcess(vector<ofVec3f>& normals) {
        for(int i = 0; i < normals.size(); i++) {
            ofVec3f& cur = normals[i];
            cur.z *= -1;
            cur.x *= -1;
        }
    }
    
    void postProcess() {	
        postProcess(triangles, globalScale);
        postProcess(normals);
        postProcess(backTriangles, globalScale);
        postProcess(backNormals);
    }
    
    
    void smoothKinect() {
        cv::Mat bfBuffer;		
        cv::Mat zMat(kinect->getHeight(), kinect->getWidth(), CV_32FC1, kinect->getDistancePixels());
        
        int k = ((int)  kinect_DrawVariables.smoothingAmount * 2) + 1;
        zMat.copyTo(bfBuffer);
        GaussianBlur(bfBuffer, zMat, cv::Size(k, k), 0);
    }
    

    void cutoffKinect() {
        float* z = kinect->getDistancePixels();
        int n = kinect->getWidth() * kinect->getHeight();
        for(int i = 0; i < n; i++) {
            if(z[i] > zCutoff || z[i] < 10) {
                z[i] = zCutoff;
            }
        }
    }
    
    
    void updateSurface() {
        float* z = kinect->getDistancePixels();
        float alpha = kinect_DrawVariables.temporalBlur;//panel.getValueF("temporalBlur");
        float beta = 1 - alpha;
        cv::Mat kinectMat = cv::Mat(480, 640, CV_32FC1, z);
        cv::Mat kinectBuffer = cv::Mat(480, 640, CV_32FC1, z);
        cv::addWeighted(kinectBuffer, alpha, kinectMat, beta, 0, kinectBuffer);
        kinectMat.copyTo(kinectBuffer);
        
        int i = 0;
        for(int y = 0; y < Yres; y++) {
            for(int x = 0; x < Xres; x++) {
                // cout << i << "at ->  " << x << " -  " << y << endl;
                surface[i] = ConvertProjectiveToRealWorld(x, y, z[i]);
                i++;
            }
        }
    }
    
    void updateTriangles() {
        triangles.resize((Xres - 1) * (Yres - 1) * 2);
        
        int j = 0;
        for(int y = 0; y < Yres - 1; y++) {
            for(int x = 0; x < Xres - 1; x++) {
                int i = y * Xres + x;
                
                int nw = i;
                int ne = nw + 1;
                int sw = i + Xres;
                int se = sw + 1;
                
                triangles[j].vert1 = surface[nw];
                triangles[j].vert2 = surface[ne];
                triangles[j].vert3 = surface[sw];
                j++;
                
                triangles[j].vert1 = surface[ne];
                triangles[j].vert2 = surface[se];
                triangles[j].vert3 = surface[sw];
                j++;
            }
        }
    }
    
    void updateTrianglesSimplify() {	
        // zero edges
        for(int x = roiStart.x; x < roiEnd.x; x++) {
            surface[getSurfaceIndex(x, roiStart.y)] = ConvertProjectiveToRealWorld(x, roiStart.y, zCutoff); // top
            surface[getSurfaceIndex(x, roiEnd.y - 1)] = ConvertProjectiveToRealWorld(x, roiEnd.y - 1, zCutoff); // bottom
        }
        for(int y = roiStart.y; y < roiEnd.y; y++) {
            surface[getSurfaceIndex(roiStart.x, y)] = ConvertProjectiveToRealWorld(roiStart.x, y, zCutoff); // left
            surface[getSurfaceIndex(roiEnd.x - 1, y)] = ConvertProjectiveToRealWorld(roiEnd.x - 1, y, zCutoff); // right
        }
        
        triangles.resize((roiEnd.x - roiStart.x) * (roiEnd.y - roiStart.y) * 2);
        int totalTriangles = 0;
        TCADTriangle* topTriangle;
        TCADTriangle* bottomTriangle;
        float* z = kinect->getDistancePixels();
        for(int y = roiStart.y; y < roiEnd.y - 1; y++) {
            bool stretching = false;
            for(int x = roiStart.x; x < roiEnd.x - 1; x++) {
                bool endOfRow = (x == roiEnd.x - 2);
                
                int nw = getSurfaceIndex(x, y);
                int ne = getSurfaceIndex(x + 1, y);
                int sw = getSurfaceIndex(x, y + 1);
                int se = getSurfaceIndex(x + 1, y + 1);
                
                bool flat = (z[nw] == z[ne] && z[nw] == z[sw] && z[nw] == z[se]);
                if(endOfRow ||
                   (stretching && flat)) {
                    // stretch the quad to our new position
                    topTriangle->vert2 = surface[ne];
                    bottomTriangle->vert1 = surface[ne];
                    bottomTriangle->vert2 = surface[se];				
                } else {			
                    topTriangle = &triangles[totalTriangles++];
                    topTriangle->vert1 = surface[nw];
                    topTriangle->vert2 = surface[ne];
                    topTriangle->vert3 = surface[sw];
                    
                    bottomTriangle = &triangles[totalTriangles++];
                    bottomTriangle->vert1 = surface[ne];
                    bottomTriangle->vert2 = surface[se];
                    bottomTriangle->vert3 = surface[sw];
                    
                    stretching = flat;
                }
            }
        }
        
        triangles.resize(totalTriangles);
    }
    
    
    void updateTrianglesRandom() {
        cv::Mat mat = cv::Mat(kinect->getHeight(), kinect->getWidth(), CV_32FC1, kinect->getDistancePixels());
        
        Sobel(mat, sobelxy, CV_32F, 1, 1);
        
        sobelxy = abs(sobelxy);
        int randomBlur = kinect_DrawVariables.randomBlur;//panel.getValueI("randomBlur") * 2 + 1;
        boxFilter(sobelxy, sobelbox, 0, cv::Size(randomBlur, randomBlur), cv::Point2d(-1, -1), false);
        
        triangulator.reset();
        points.clear();
        int i = 0;
        attempts = 0;
        int randomCount = kinect_DrawVariables.randomCount;//panel.getValueI("randomCount");
        float randomWeight = kinect_DrawVariables.randomWeight;// panel.getValueF("randomWeight");
        while(i < randomCount) {
            cv::Point2d curPosition(1 + (int) ofRandom(sobelbox.cols - 3), 
                                1 + (int) ofRandom(sobelbox.rows - 3));
            float curSample = sobelbox.at<unsigned char>(curPosition) / 255.f;
            float curGauntlet = powf(ofRandom(0, 1), 2 * randomWeight);
            if(curSample > curGauntlet) {
                points.push_back(ofVec2f(curPosition.x, curPosition.y));
                triangulator.addPoint(curPosition.x, curPosition.y, 0);
                sobelbox.at<unsigned char>(curPosition) = 0; // don't do the same point twice
                i++;
            }
            attempts++;
            if(i > attempts * 100) {
                break;
            }
        }
        
        // add the edges
        int w = mat.cols;
        int h = mat.rows;
        for(int x = 0; x < w; x++) {
            triangulator.addPoint(x, 0, 0);
            triangulator.addPoint(x, h - 1, 0);
        }
        for(int y = 0; y < h; y++) {
            triangulator.addPoint(0, y, 0);
            triangulator.addPoint(w - 1, y, 0);
        }
        
        triangulator.triangulate();
        
        int n = triangulator.triangles.size();
        triangles.resize(n);
        for(int i = 0; i < n; i++) {
            triangles[i].vert3 = triangulator.triangles[i].points[0];
            triangles[i].vert2 = triangulator.triangles[i].points[1];
            triangles[i].vert1 = triangulator.triangles[i].points[2];
        }
    }
    
    void addBack(ofVec3f& a, ofVec3f& b, ofVec3f& c) {
        backTriangles.push_back(TCADTriangle(a, b, c));
    }
    
    void addBack(ofVec3f& a, ofVec3f& b, ofVec3f& c, ofVec3f& d) {
        addBack(a, b, d);
        addBack(b, c, d);
    }
    
    void updateBack() {
        backTriangles.clear();
        TCADTriangle cur;
        ofVec3f offset(0, 0, backOffset);
        
        ofVec3f nw = surface[getSurfaceIndex(roiStart.x, roiStart.y)];
        ofVec3f ne = surface[getSurfaceIndex(roiEnd.x - 1, roiStart.y)];
        ofVec3f sw = surface[getSurfaceIndex(roiStart.x, roiEnd.y - 1)];
        ofVec3f se = surface[getSurfaceIndex(roiEnd.x - 1, roiEnd.y - 1)];
        
        ofVec3f nwo = nw + offset;
        ofVec3f swo = sw + offset;
        ofVec3f neo = ne + offset;
        ofVec3f seo = se + offset;
        
        addBack(nwo, neo, ne, nw); // top
        addBack(ne, neo, seo, se); // right
        addBack(sw, se, seo, swo); // bottom
        addBack(nwo, nw, sw, swo); // left
        
        // two back faces
        addBack(nwo, swo, seo, neo);
        
        calculateNormals(backTriangles, backNormals);
    }
    
};


#endif
