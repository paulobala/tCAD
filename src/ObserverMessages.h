#ifndef tCAD_ObserverMessages_h
#define tCAD_ObserverMessages_h

/*
 Part of Observer Pattern
 */
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
