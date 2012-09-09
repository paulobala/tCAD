#ifndef Carver_Observer_h
#define Carver_Observer_h

#pragma once

#include <vector>
#include "ObserverMessages.h"

using namespace std;

class Subject;

class Observer
{
public:
	virtual void Notify(Subject* subject, ObserverMessage* message){};
};


#endif
