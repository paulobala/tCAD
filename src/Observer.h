#ifndef tCAD_Observer_h
#define tCAD_Observer_h

#pragma once

#include <vector>
#include "ObserverMessages.h"

using namespace std;

class Subject;//fowards declaration
/*
 Part of Observer Pattern
 */
class Observer
{
public:
	virtual void Notify(Subject* subject, ObserverMessage* message){};
};


#endif
