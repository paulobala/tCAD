//
//  Composite3DObject.h
//  Modeller
//
//  Created by paulobala on 11/04/12.
//  Copyright (c) 2012 __MyCompanyName__. All rights reserved.
//

#ifndef Modeller_Composite3DObject_h
#define Modeller_Composite3DObject_h
#include "Shape3D.h"
#include "ofxCarve.h"
#include "Base3DObject.h"

class Composite3DObject: public Base3DObject{
    
    Boolean hasChosenMesh;
    Boolean hasShape;
public:    
    enum BooleanOperation {
		BOOLEAN_INTERSECTION, 
		BOOLEAN_UNION, 
		BOOLEAN_DIFFERENCE_A_MINUS_B,
        BOOLEAN_DIFFERENCE_B_MINUS_A,
        BOOLEAN_SYMMETRIC_DIFFERENCE
	};
	ofMeshtCAD meshIntersect, meshUnion, meshDifferenceAB, meshDifferenceBA, meshSymDifference; 
 
    std::vector<Shape3D*> children; 
        
    Composite3DObject():Base3DObject(new ofColor(150,150,150)){
        hasShape = false;
        hasChosenMesh = false;
        children = std::vector<Shape3D*>();
        mesh.setMode(OF_PRIMITIVE_TRIANGLES);
        meshIntersect.setMode(OF_PRIMITIVE_TRIANGLES);
        meshUnion.setMode(OF_PRIMITIVE_TRIANGLES);
        meshDifferenceAB.setMode(OF_PRIMITIVE_TRIANGLES);
        meshDifferenceBA.setMode(OF_PRIMITIVE_TRIANGLES);
        meshSymDifference.setMode(OF_PRIMITIVE_TRIANGLES);
    };
  
    void add(Shape3D * shape_)
    {
        if(children.size()==0 || children.size() ==1){
        children.push_back(shape_);
        }
    }
    
    void drawChildren(){
    for (int i = 0; i < children.size(); i++){
         children[i]->draw();
        }
    }
    
    void draw(){  
        if(hasShape == true){
            drawStyle->draw();
        }
        else{
            drawChildren();
        }
    }
    
    carve::mesh::MeshSet<3> * readChildren(Shape3D* objA){
        
        std::vector< carve::mesh::MeshSet<3>::face_t *> faces = std::vector<carve::mesh::MeshSet<3>::face_t *>();
        
        vector<TCADTriangle> triangles = objA->mesh.getTriangles();
     
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
            ofVec3f v3 = ofVec3f( verts[2]->v.x, verts[2]->v.y, verts[2]->v.z);	
            
            mesh.addFace(v1,v2,v3);
            
        }    
    }
    
    void allOperations(){
        carve::mesh::MeshSet<3> * objA = readChildren(children[0]);
        carve::mesh::MeshSet<3> * objB = readChildren(children[1]);
        
        ofxCarve carver = ofxCarve();

        std::vector<std::pair<carve::mesh::MeshSet<3> *, ofxCarve::OP > > results = std::vector<std::pair<carve::mesh::MeshSet<3> *, ofxCarve::OP > > ();
        
                
        results = carver.allOperations(objA, objB);
        
        if(results.size() == 0){
            hasShape=false;
        }
        else{
            hasShape = true;
            
            for (std::vector<std::pair<carve::mesh::MeshSet<3> *, ofxCarve::OP > >::iterator it =results.begin() ; it < results.end(); it++ ){
                switch((*it).second){
                    case ofxCarve::UNION:
                        if((*it).first != NULL){
                            mesh.clear();
                            meshSetToMesh((*it).first);
                            mesh.invertNormals();
                            meshUnion = mesh;
                        }else{
                            meshUnion.clear();
                        }
                        break;
                    case ofxCarve::INTERSECTION:
                        if((*it).first != NULL){
                            mesh.clear();
                            meshSetToMesh((*it).first);
                            mesh.invertNormals();
                            if(mesh.getNumVertices() == 0){meshIntersect.clear();}
                            else{ meshIntersect= mesh;}
                        }
                        else{
                            meshIntersect.clear();
                        }
                        break;
                    case ofxCarve::A_MINUS_B:
                        if((*it).first != NULL){
                            mesh.clear();
                            meshSetToMesh((*it).first);
                            mesh.invertNormals();
                            meshDifferenceAB= mesh;
                        }
                        else{
                            meshDifferenceAB.clear();
                        }

                        break;
                    case ofxCarve::B_MINUS_A:
                        if((*it).first != NULL){
                            mesh.clear();
                            meshSetToMesh((*it).first);
                            mesh.invertNormals();
                            meshDifferenceBA= mesh;
                        }
                        else{
                            meshDifferenceBA.clear();
                        }

                        break;
                    case ofxCarve::SYMMETRIC_DIFFERENCE:  
                        if((*it).first != NULL){
                            mesh.clear();
                            meshSetToMesh((*it).first);
                            mesh.invertNormals();
                            if(mesh.getNumVertices() == 0){meshSymDifference.clear();}
                            else{ meshSymDifference= mesh;}
                        }
                        else{
                            meshSymDifference.clear();
                        }
                        break;
                    default:break;
                }
          
              
            }  
            mesh.clear();  
        }
    }
    
    bool chooseOperation(BooleanOperation bop){
        if(hasShape){
        switch(bop){
            case BOOLEAN_INTERSECTION:
                if(mesh.getNumVertices() == 0){
                    hasChosenMesh = false;
                }
                else{
                    mesh = meshIntersect;
                    hasChosenMesh = true;
                }
                break;
                
            case BOOLEAN_UNION:
                mesh = meshUnion;
                hasChosenMesh = true;
                break;
                
            case BOOLEAN_DIFFERENCE_A_MINUS_B:
                 mesh = meshDifferenceAB;hasChosenMesh = true;
                break;
            case BOOLEAN_DIFFERENCE_B_MINUS_A:
                 mesh = meshDifferenceBA;hasChosenMesh = true;
                break;
            case BOOLEAN_SYMMETRIC_DIFFERENCE:
                if(mesh.getNumVertices() == 0){
                    hasChosenMesh = false;
                }
                else{
                    mesh = meshSymDifference;
                    hasChosenMesh = true;
                }
                break;
            default:break;
        }
        
        }
        return hasChosenMesh;
    }

    void getFacets(std::vector<ofxSTLFacet>* facets){
        vector<TCADTriangle> triangles = mesh.getTriangles();
        
        for (vector<TCADTriangle>::iterator it=triangles.begin() ; it < triangles.end(); it++ ){
            ofVec3f normal = -(((*it).vert3 - (*it).vert1).cross((*it).vert2 - (*it).vert1)).normalize();
            ofxSTLFacet temp = ofxSTLFacet();
            temp.vert1 = (*it).vert1;
            temp.vert2 = (*it).vert2;
            temp.vert3 = (*it).vert3;
            temp.normal = normal;
            facets->push_back(temp);
        }

    }
    void scale(float scalex,float scaley,float scalez){
     if(hasShape){ 
         if(!(scalex == shapeVariables.scaleX && scaley== shapeVariables.scaleY && scalez ==shapeVariables.scaleZ)){
            mesh.scale(1/shapeVariables.scaleX, 1/shapeVariables.scaleY,1/shapeVariables.scaleZ);
            shapeVariables.scaleX = scalex;
            shapeVariables.scaleY = scaley;
            shapeVariables.scaleZ = scalez;
            shapeVariables.scaleXYZ = 1;
            mesh.scale(shapeVariables.scaleX, shapeVariables.scaleY,shapeVariables.scaleZ);
        }
     }

    }
    void scale(float scalexyz){
        if(!(scalexyz == shapeVariables.scaleXYZ)){
     
        scale(scalexyz,scalexyz,scalexyz);
        shapeVariables.scaleXYZ = scalexyz;
        }
    }
    void translate(float coordx, float coordy, float coordz){
        if(hasShape){
            if(!(coordx == shapeVariables.translateX && coordy== shapeVariables.translateY && coordz ==shapeVariables.translateZ)){
                mesh.translate(-shapeVariables.translateX, -shapeVariables.translateY, -shapeVariables.translateZ);
                shapeVariables.translateX = coordx;
                shapeVariables.translateY = coordy;
                shapeVariables.translateZ = coordz;
                mesh.translate(shapeVariables.translateX, shapeVariables.translateY, shapeVariables.translateZ);
            }
        }
    }
    void change(MANIPULATIONTYPE ct, AXIS a, INCRTYPE it){
         if(hasShape){
         Base3DObject:change(ct,a,it);
         }
    }
    void rotate(float anglex, float angley, float anglez){
        if(hasShape){
            if(!(anglex == shapeVariables.rotateX && angley== shapeVariables.rotateY && anglez ==shapeVariables.rotateZ)){
                
                ofVec3f centroid = mesh.getCentroid();
                
                if(anglex != shapeVariables.rotateX){
                    ofVec3f dist = ofVec3f(0,0,0)-centroid;
                    mesh.translate(dist);
                    mesh.rotate(-shapeVariables.rotateX, ofVec3f(1,0,0));
                    shapeVariables.rotateX = anglex;
                    mesh.rotate(shapeVariables.rotateX, ofVec3f(1,0,0));
                    mesh.translate(-dist);
                }
                if(angley != shapeVariables.rotateY){
                    ofVec3f dist = ofVec3f(0,0,0)-centroid;
                    mesh.translate(dist);
                    mesh.rotate(-shapeVariables.rotateY, ofVec3f(0,1,0));
                    shapeVariables.rotateY = angley;
                    mesh.rotate(shapeVariables.rotateY, ofVec3f(0,1,0));
                    mesh.translate(-dist);
                }
                if(anglez != shapeVariables.rotateZ){
                    ofVec3f dist = ofVec3f(0,0,0)-centroid;
                    mesh.translate(dist);
                    mesh.rotate(-shapeVariables.rotateZ, ofVec3f(0,0,1));
                    shapeVariables.rotateZ = anglez;
                    mesh.rotate(shapeVariables.rotateZ, ofVec3f(0,0,1));
                    mesh.translate(-dist);
                }
            }
        }
    }
}
;

#endif
