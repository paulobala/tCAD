//
//  Line.cpp
//  DemoLab
//
//  Created by paulobala on 27/02/12.
//  Copyright 2012 __MyCompanyName__. All rights reserved.
//
#include "ofxOpenCv.h"
#include "lineGeom.h"
#include <iostream>
using namespace std;

bool lineGeom::intersection(lineGeom *l1, cv::Point *r) {
        //cout << "intersection" << endl;
        
        if(l1->gettheta() == this->gettheta() ){
            //cout << "parallel"<< endl;
            return false;}
        float ct1=cos(this->gettheta());     
        float st1=sin(this->gettheta());    
        float ct2=cos(l1->gettheta());     
        float st2=sin(l1->gettheta());   
        
        CvMat* A  = cvCreateMat(2,2,CV_32FC1);
        CvMat* X  = cvCreateMat(2,1,CV_32FC1);
        CvMat* B  = cvCreateMat(2,1,CV_32FC1);
        cvSetReal1D(A, 0, ct1);
        cvSetReal1D(A, 1, st1);
        cvSetReal1D(A, 2, ct2);
        cvSetReal1D(A, 3, st2);
        cvSetReal1D(B, 0, this->getrho());
        cvSetReal1D(B, 1, l1->getrho());
        cvSolve(A, B, X );
        r->x = cvGetReal1D(X,0);
        r->y = cvGetReal1D(X,1);
        //cout << r->x << " -> " << r->y << endl;
        return true;
    };
    
float lineGeom::resultFromX(float x){
        return (this->getrho()-x*cos(this->gettheta()))/sin(this->gettheta());
    }
    
float lineGeom::resultFromY(float y){
        return (this->getrho()-y*sin(this->gettheta()))/cos(this->gettheta());
    }

