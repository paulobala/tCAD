#include <iostream>
#include "Subject.h"
#include "ObserverMessages.h"

class Notifier
{
public:
	Notifier(Subject* subject, ObserverMessage* amessage) : m_subject(subject), message(amessage)
	{}
    
	void operator()(Observer* observer)
	{
		observer->Notify(m_subject, message);
	}
    
private:
	Subject* m_subject;
    ObserverMessage* message;
};

Subject::~Subject(){}

//this method adds an observer to the vector of observers
bool Subject::addObserver( Observer* observer )
{
	vector<Observer*>::iterator temp = find(m_ObserverVec.begin(), m_ObserverVec.end(), observer);
	//Return false if the observer is already in the vector. This is not expected. But there is nothing really wrong either
	if ( temp != m_ObserverVec.end() )
		return false;
    
	m_ObserverVec.push_back(observer);
	return true;
}

//This method removes an observer from the vector
bool Subject::removeObserver( Observer* observer )
{
	vector<Observer*>::iterator temp = find(m_ObserverVec.begin(), m_ObserverVec.end(), observer);
	//Return false if the observer could not be found (and evidently can’t be removed.
	if ( temp == m_ObserverVec.end() )
		return false;
	else
		m_ObserverVec.erase(remove(m_ObserverVec.begin(), m_ObserverVec.end(), observer));
	return true;
    	
}

//This Method is very important, it triggers all Notify() methods of all observers.
//The specific code in each class that inherits from observer will be executed
bool Subject::notifyObservers(ObserverMessage * message)
{
	for_each(m_ObserverVec.begin(), m_ObserverVec.end(), Notifier(this, message));
	//Return false if there are no observers in the vector
	return (m_ObserverVec.size() > 0);
}
