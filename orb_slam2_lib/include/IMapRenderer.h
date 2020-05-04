//
// Created by sebastiano on 15/08/16.
//

#ifndef ORB_SLAM2_IMAPRENDERER_H
#define ORB_SLAM2_IMAPRENDERER_H

#include <pangolin/pangolin.h>

class IMapRenderer
{
    virtual void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M) = 0;
};


#endif //ORB_SLAM2_IMAPRENDERER_H
