//
// Created by sebastiano on 8/18/16.
//

#include "IMapPublisher.h"

using namespace ORB_SLAM2;
using namespace std;

void IMapPublisher::SetCurrentCameraPose(const cv::Mat &Tcw)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone();
    mbCameraUpdated = true;
}

cv::Mat IMapPublisher::GetCameraPose()
{
    unique_lock<mutex> lock(mMutexCamera);
    return mCameraPose;
}

bool IMapPublisher::isCamUpdated()
{
    unique_lock<mutex> lock(mMutexCamera);
    return mbCameraUpdated;
}

void IMapPublisher::ResetCamFlag()
{
    unique_lock<mutex> lock(mMutexCamera);
    mbCameraUpdated = false;
}
