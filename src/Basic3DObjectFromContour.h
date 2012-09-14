#ifndef tCAD_Basic3DObjectFromStencil_h
#define tCAD_Basic3DObjectFromStencil_h
#include "Basic3DObject.h"
#include "ofxSTL.h"
#include "ofEasyFingerCam.h"
/*
 Object from Contour Mode
 */
class Basic3DObjectFromContour: public Basic3DObject
{
public:
    /*
     Constructor. Receives a path, tesselates it and converts it to a mesh
     */
    Basic3DObjectFromContour(ofVec3f * entryPoint, ofPolyline poly):Basic3DObject()
    {
        mesh.setMode(OF_PRIMITIVE_TRIANGLES);
        
        ofMesh meshCarve, bottomMesh, topMesh;
        vector<ofPolyline> contours;
        contours.push_back(poly);
        meshCarve.clear();
        
        float zOffset = 10;
        ofVec3f offset(0, 0, -zOffset);
        for(int k = 0; k < contours.size(); k++)
        {
            for(int i = 0; i < contours[k].size(); i++)
            {
                ofVec3f a, b;
                int cur = i;
                int next = (i + 1) % contours[k].size();
                a.set(contours[k][cur].x, contours[k][cur].y, 0);
                b.set(contours[k][next].x, contours[k][next].y, 0);
                addFace(meshCarve, b, a, a + offset, b + offset);
            }
        }
        //Top mesh
        ofPath top;
        top.setPolyWindingMode(OF_POLY_WINDING_NONZERO);
        for(int k = 0; k < contours.size(); k++)
        {
            top.newSubPath();
            for(int i = contours[k].size() - 1; i >= 0; i--)
            {
                top.lineTo(contours[k][i].x, contours[k][i].y);
            }
            top.close();
        }
        topMesh = top.getTessellation();
        vector<ofVec3f>& topVertices = topMesh.getVertices();
        for(int i = 0; i < topVertices.size(); i++)
        {
            topVertices[i] += offset;
        }
        //bottom mesh
        ofPath bottom;
        for(int k = 0; k < contours.size(); k++)
        {
            bottom.newSubPath();
            for(int i = 0; i < contours[k].size(); i++)
            {
                bottom.lineTo(contours[k][i].x, contours[k][i].y);
            }
            bottom.close();
        }
        bottomMesh = bottom.getTessellation();
        
        addMesh(meshCarve);
        addMesh(topMesh);
        addMesh(bottomMesh);
        
        ofVec3f centroid = mesh.getCentroid();
        ofVec3f entry = *entryPoint;
        centroid = entry - centroid;
        mesh.translate(centroid);//move to entry point
    }
    
    void addFace(ofMesh& amesh, ofVec3f a, ofVec3f b, ofVec3f c)
    {
        ofVec3f normal = ((c - a).cross(b - a)).normalize();
        amesh.addNormal(normal);
        amesh.addVertex(a);
        amesh.addNormal(normal);
        amesh.addVertex(b);
        amesh.addNormal(normal);
        amesh.addVertex(c);
    }
    
    void addFace(ofMesh& amesh, ofVec3f a, ofVec3f b, ofVec3f c, ofVec3f d)
    {
        addFace(amesh, a, b, d);
        addFace(amesh, b, c, d);
    }
    
    void addMesh(ofMesh& amesh)
    {
        vector<ofVec3f>& vertices = amesh.getVertices();
        vector<ofIndexType>& indices = amesh.getIndices();
        if(indices.size() > 0)
        {
            for(int i = 0; i < indices.size() - 2; i += 3)
            {
                mesh.addFace(vertices[indices[i+2]],vertices[indices[i+1]], vertices[indices[i]]);
            }
        }
        else
        {
            if(amesh.getMode() == OF_TRIANGLES_MODE)
            {
                for(int i = 0; i < vertices.size() - 2; i += 3)
                {
                    mesh.addFace(vertices[i+2],vertices[i+1], vertices[i]);
                }
            }
            else if(amesh.getMode() == OF_TRIANGLE_STRIP_MODE)
            {
                for(int i = 0; i < vertices.size() - 2; i++)
                {
                    mesh.addFace(vertices[i+2],vertices[i+1], vertices[i]);
                }
            }
        }
    }
    
    
    
};
#endif


