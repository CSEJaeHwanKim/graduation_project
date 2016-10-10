#ifndef SINGLETON_H
#define SINGLETON_H

#include <utility>
#include <mutex>

using namespace std;

template <class T>
class singleton {
public:
   template<typename... Args>
   static T* getInstance(Args... args) {
      if (_instance == nullptr) {
         std::lock_guard<std::mutex> lock(*mutex_singletone);
         if (_instance == nullptr) {
            _instance = new T(std::forward<Args>(args)...);
         }
      }
      return _instance;
   }
   static void destroy() {
      if (_instance) {
         delete _instance;
         _instance = nullptr;
      }
   }

protected:
   singleton() {}
   singleton(singleton const&) {}
   singleton& operator=(singleton const&) { return *this; }


private:
   static T* _instance;
   static mutex* mutex_singletone;
};

template <class T> T* singleton<T>::_instance = nullptr;
template <class T> mutex* singleton<T>::mutex_singletone = new mutex;

#endif