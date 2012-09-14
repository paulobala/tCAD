#ifndef tCAD_Subject_h
#define tCAD_Subject_h

#pragma once

#include <vector>
#include <algorithm>
#include "Observer.h"
#include "ObserverMessages.h"

class Observer;//fowards declaration

/*
 Part of Observer Pattern
 */
class Subject
{
public:
    virtual ~Subject();
    
	bool addObserver(Observer* observer);
	bool removeObserver(Observer* observer);
	bool notifyObservers(ObserverMessage* message);

private: 
	vector<Observer*> m_ObserverVec;

};


#endif
