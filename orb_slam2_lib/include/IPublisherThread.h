//
// Created by sebastiano on 8/18/16.
//

#ifndef ORB_SLAM2_IPUBLISHERTHREAD_H
#define ORB_SLAM2_IPUBLISHERTHREAD_H

#include <mutex>


namespace ORB_SLAM2
{

class System;


//Definition of Class IPublisherThread
class IPublisherThread
{
public:
    IPublisherThread(); //constructor declaration
    virtual ~IPublisherThread(); //destructor declaration

    //Virtual destructors are useful when you can delete an instance of a derived
    //class through a pointer to base class:

    virtual void Run() = 0;

    //https://stackoverflow.com/questions/2391679/why-do-we-need-virtual-functions-in-c
            // to overload function from derived class

    void SetSystem(System *sys);
    System* GetSystem() { return mpSystem; }

    void RequestStop();
    bool isStopped();
    void Release();
    void RequestFinish();
    bool isFinished();

protected:
    bool WaitCycleStart();

    bool Stop();
    bool CheckFinish();
    void SetFinish(bool value = true);
    //set this var as protected so that child class
    //Viewer can access it
    bool mbReuseMap;

private:
    std::mutex mMutexStop;
    bool mbStopRequested;
    bool mbStopped;

    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;



    // Mutex for general object mutation
    std::mutex mMutexMut;
    System *mpSystem;
};

}


#endif //ORB_SLAM2_IPUBLISHERTHREAD_H
