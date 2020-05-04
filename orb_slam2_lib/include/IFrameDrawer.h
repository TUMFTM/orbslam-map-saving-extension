//
// Created by sebastiano on 8/19/16.
//

#ifndef ORB_SLAM2_IFRAMEDRAWER_H
#define ORB_SLAM2_IFRAMEDRAWER_H

#include "IFrameSubscriber.h"
#include <opencv2/core/core.hpp>

namespace ORB_SLAM2
{

class IFrameRenderer
{
public:
    virtual cv::Mat DrawFrame();
};

class IFrameDrawer :
    public IFrameRenderer,
    public IFrameSubscriber
{};

}

#endif //ORB_SLAM2_IFRAMEDRAWER_H
