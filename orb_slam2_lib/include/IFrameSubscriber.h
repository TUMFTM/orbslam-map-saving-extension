//
// Created by sebastiano on 8/19/16.
//

#ifndef ORB_SLAM2_IFRAMESUBSCRIBER_H
#define ORB_SLAM2_IFRAMESUBSCRIBER_H


namespace ORB_SLAM2
{
class Tracking;

class IFrameSubscriber
{
public:
    virtual void Update(Tracking *);
};
}

#endif //ORB_SLAM2_IFRAMESUBSCRIBER_H
