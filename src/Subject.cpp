#include <iostream>
#include "Subject.h"
#include "ObserverMessages.h"
/*
 Part of Observer Pattern
 */
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


bool Subject::addObserver( Observer* observer )
{
	vector<Observer*>::iterator temp = find(m_ObserverVec.begin(), m_ObserverVec.end(), observer);
	//Return false if the observer is already in the vector.
	if ( temp != m_ObserverVec.end() )
		return false;
    
	m_ObserverVec.push_back(observer);
	return true;
}


bool Subject::removeObserver( Observer* observer )
{
	vector<Observer*>::iterator temp = find(m_ObserverVec.begin(), m_ObserverVec.end(), observer);
	//Return false if the observer could not be found
	if ( temp == m_ObserverVec.end() )
		return false;
	else
		m_ObserverVec.erase(remove(m_ObserverVec.begin(), m_ObserverVec.end(), observer));
	return true;
    	
}


bool Subject::notifyObservers(ObserverMessage * message)
{
	for_each(m_ObserverVec.begin(), m_ObserverVec.end(), Notifier(this, message));
	//Return false if there are no observers in the vector
	return (m_ObserverVec.size() > 0);
}
