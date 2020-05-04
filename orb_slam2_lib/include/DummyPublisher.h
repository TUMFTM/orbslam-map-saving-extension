#ifndef DUMMYPUBLISHER_H
#define DUMMYPUBLISHER_H

#include "IPublisherThread.h"

namespace ORB_SLAM2
{

class DummyPublisher : public IPublisherThread {
public:
    virtual void Run() override {
	SetFinish(true);
    }
};

}

#endif
