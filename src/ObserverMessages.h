#ifndef Carver_ObserverMessages_h
#define Carver_ObserverMessages_h

class ObserverMessage{

public:
    enum MESSAGE{
       REMOVED, UPDATED
    };

    MESSAGE message;
    
    ObserverMessage(MESSAGE amessage){
        message = amessage;
    }
};

#endif
