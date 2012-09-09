#ifndef Modeller_Basic3DObject_h
#define Modeller_Basic3DObject_h
#include "Shape3D.h"
#include "Base3DObject.h"
#include "ofEasyFingerCam.h"
#include "ofxCarve.h"


class Basic3DObject:public Base3DObject{
public:
    
    Basic3DObject():Base3DObject(new ofColor(0,100,0)){
        
    }
         
    void scale(float scalex,float scaley,float scalez){
        
        if(!(scalex == shapeVariables.scaleX && scaley== shapeVariables.scaleY && scalez ==shapeVariables.scaleZ)){
        mesh.scale(1/shapeVariables.scaleX, 1/shapeVariables.scaleY,1/shapeVariables.scaleZ);
                shapeVariables.scaleX = scalex;
                shapeVariables.scaleY = scaley;
                shapeVariables.scaleZ = scalez;
                shapeVariables.scaleXYZ = 1;
        mesh.scale(shapeVariables.scaleX, shapeVariables.scaleY,shapeVariables.scaleZ);
        }
    }
    void scale(float scalexyz){
        if(!(scalexyz == shapeVariables.scaleXYZ)){
        scale(scalexyz,scalexyz,scalexyz);
        shapeVariables.scaleXYZ = scalexyz;
        }
    }
    
    void translate(float coordx, float coordy, float coordz){
        if(!(coordx == shapeVariables.translateX && coordy== shapeVariables.translateY && coordz ==shapeVariables.translateZ)){
            mesh.translate(-shapeVariables.translateX, -shapeVariables.translateY, -shapeVariables.translateZ);
                    shapeVariables.translateX = coordx;
                    shapeVariables.translateY = coordy;
                    shapeVariables.translateZ = coordz;
            mesh.translate(shapeVariables.translateX, shapeVariables.translateY, shapeVariables.translateZ);
        }
    }
    
    void rotate(float anglex, float angley,float anglez){
        if(!(anglex == shapeVariables.rotateX && angley== shapeVariables.rotateY && anglez == shapeVariables.rotateZ)){
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
  
    void getFacets(vector<ofxSTLFacet>* facets){
        
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
private:
    
}
;

#endif
