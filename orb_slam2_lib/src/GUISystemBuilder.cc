//
// Created by sebastiano on 8/23/16.
//

#include "GUISystemBuilder.h"
#include "utils.h"
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Viewer.h"

using namespace ORB_SLAM2;


GUISystemBuilder::GUISystemBuilder(const std::string &strVocFile,
                                   const std::string &strSettingsFile,
                                   System::eSensor sensor, bool is_save_map_)
    : System::GenericBuilder(strVocFile, strSettingsFile, sensor, is_save_map_)
{
    //When the GenericBuilder constructor runs, it will check if a map.bin is available
    //and set the var bReuseMap accordingly. Then we can load this var through
    //GetMapReuseOption() 
    mpFrameDrawer = make_unique<FrameDrawer>(mpMap.get(), GetMapReuseOption());
    mpMapDrawer = make_unique<MapDrawer>(mpMap.get(), strSettingsFile);
    mpViewer = make_unique<Viewer>(mpFrameDrawer.get(), mpMapDrawer.get(), mpTracker.get(), strSettingsFile, GetMapReuseOption());
}

// Empty dtor is needed to give a place to the calls to the dtors of unique_ptr members
GUISystemBuilder::~GUISystemBuilder() { }

IPublisherThread* GUISystemBuilder::GetPublisher()
{
    return mpViewer.get();
}
