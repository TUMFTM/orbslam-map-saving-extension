//
// Created by sebastiano on 8/23/16.
//

#ifndef ORB_SLAM2_GUISYSTEM_H
#define ORB_SLAM2_GUISYSTEM_H

#include "System.h"

#include <memory>

namespace ORB_SLAM2
{

class IPublisherThread;
class Viewer;
class MapDrawer;
class FrameDrawer;

class GUISystemBuilder : public System::GenericBuilder
{
public:
    GUISystemBuilder(const std::string &strVocFile, const std::string &strSettingsFile, System::eSensor sensor, bool is_save_map_=false);
    virtual ~GUISystemBuilder();

    virtual IPublisherThread *GetPublisher() override;

private:
    std::unique_ptr<FrameDrawer> mpFrameDrawer;
    std::unique_ptr<MapDrawer> mpMapDrawer;
    std::unique_ptr<Viewer> mpViewer;
};

}

#endif //ORB_SLAM2_GUISYSTEM_H
