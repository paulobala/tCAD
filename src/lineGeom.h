#ifndef tCAD_lineGeom_h
#define tCAD_lineGeom_h
using namespace std;

/*
2D Line described in rho-theta parametrization. Necessary to deal with lines from openCV Hough lines
*/
class lineGeom{
    
protected:
    CvPoint pt1;
    CvPoint pt2;
    
    double rho; 
    double theta;
    
public:
    lineGeom( double arho,  double atheta, cv::Point apt1, cv::Point apt2){
        rho = arho;
        theta = atheta;
        pt1 = apt1;
        pt2 = apt2;
    };
    
    double getrho(){return rho;}
    double gettheta(){return theta;}
    cv::Point getpt1(){return pt1;}
    cv::Point getpt2(){return pt2;}
    
    //intersect this line with l1 line and store intersection point in r
    bool intersection(lineGeom *l1, cv::Point *r) ;
    
    float resultFromX(float x);
    
    float resultFromY(float y);
};



#endif
