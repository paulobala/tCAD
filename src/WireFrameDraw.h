#ifndef tCAD_WireFrame_h
#define tCAD_WireFrame_h
class WireFrameDraw: public ParentDraw{
public:
    
    WireFrameDraw(Shape3D* shape_):ParentDraw(shape_, new ofColor(200,100,100)){
    };
    WireFrameDraw(Shape3D* shape_, ofColor * color_):ParentDraw(shape_, color_){
    };
    void draw(){
        ofPushStyle();
        ofEnableLighting();
        ofSetColor(*color);
        glEnable(GL_DEPTH_TEST);
        shape3D->mesh.drawWireframe();
        ofDisableLighting();  
        glDisable(GL_DEPTH_TEST);
        ofPopStyle();
    }
};
#endif
