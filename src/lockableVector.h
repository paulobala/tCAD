#ifndef tCAD_lockableVector_h
#define tCAD_lockableVector_h

#ifndef WIN32
#include <pthread.h>
#else
#include <windows.h>
#endif
#include <vector>

/*
 Generic class for a lockable vector
 */

template <typename T>
class LockableVector { 
    
public:
    std::vector<T> vec;
    
    LockableVector(){
#ifndef WIN32	
        pthread_mutex_init(&lockMutex,NULL);
#else
        cursorMutex = CreateMutex(NULL,FALSE,"lockMutex");
#endif	
    }
    ~LockableVector(){
#ifndef WIN32	
        pthread_mutex_destroy(&lockMutex);
#else
        CloseHandle(lockMutex);
#endif		
    };

    void lockVector() {
#ifndef WIN32	
        pthread_mutex_lock(&lockMutex);
#else
        WaitForSingleObject(lockMutex, INFINITE);
#endif		
    }
;
    
    void unlockVector() {
#ifndef WIN32	
        pthread_mutex_unlock(&lockMutex);
#else
        ReleaseMutex(lockMutex);
#endif
    }; 
    
    std::vector<T> getObjects(){
        lockVector();
        std::vector<T> vecBuffer = vec;
        unlockVector();
        return vecBuffer;
    };
    
    void addElement(T element)  {
        vec.push_back(element);
    };
    void removeElement(T element){
        for (typename std::vector<T>::iterator it  = vec.begin()  ; it < vec.end(); ++it ){
            if((*it)==element){
                vec.erase(it);
            }
        }
    }
;  
    
private:

#ifndef WIN32
    pthread_mutex_t lockMutex;
#else
    HANDLE lockMutex;
#endif	

};

#endif
