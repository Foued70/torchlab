#ifndef SINGLETON_H
#define SINGLETON_H

#include <assert.h>

template < typename T >
class Singleton {
private:
        static T* ms_Singleton;

public:
        Singleton(void) {
                assert(!ms_Singleton);

                intptr_t offset = (intptr_t)(T*)1 - (intptr_t)(Singleton *)(T*)1;
                ms_Singleton = (T*)((intptr_t)this + offset);
        }

        ~Singleton(void) {
                assert(ms_Singleton);
                ms_Singleton = 0;
        }

        static T& GetSingleton(void) {
                assert(ms_Singleton);
                return *ms_Singleton;
        }

        static T* GetSingletonPtr(void) {
                return ms_Singleton;
        }
};

template < typename T > T* Singleton <T>::ms_Singleton = 0;
#endif  /* SINGLETON_H */
