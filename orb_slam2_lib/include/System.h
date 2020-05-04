/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef SYSTEM_H
#define SYSTEM_H

#include "Map.h"
#include "ORBVocabulary.h"
#include "IPublisherThread.h"
#include "Serialize.h"

#include <string>
#include <thread>
#include <mutex>
#include <opencv2/core/core.hpp>

// for map file io
#include <fstream>

namespace ORB_SLAM2
{

class IPublisherThread;
class IFrameSubscriber;
class IMapPublisher;
class Map;
class Tracking;
class LocalMapping;
class LoopClosing;
class KeyFrameDatabase;

class System
{
public:
    // Input sensor
    enum eSensor{
        MONOCULAR=0,
        STEREO=1,
        RGBD=2
    };

    // A `Builder`'s responsibility is to create, link together and destroy each individual
    // part of a `System`.  The pure virtual methods are all supposed to be simple getters.

    //IBuilder contains pure virtual fucntions and therefore it is an ABSTRACT class. Hence,
    //its derived classes MUST implement/override the pure virtual methods, or they too will become abstract.
    //Here, GenericBuilder implements the Getters, except for GetPublisher which is implemented by ROSSystemBuilder
    //in ROSPublisher.cc file

    //Note that IBuilder and GenericBuilder are nested classes, defined inside System class.
    //Like any normal member of a class, nested classes have the same access to members of
    //the enclosing class that the enclosing class does.
    //The getters below are used to provide access to PRIVATE members of the System class.

    class IBuilder {
    public:
        virtual ~IBuilder();
        virtual bool GetMapSaveOption() = 0;
        virtual string GetMapfileName() = 0;
        virtual bool GetMapReuseOption() = 0;
        virtual eSensor GetSensorType() = 0;
        virtual ORBVocabulary* GetVocabulary() = 0;
        virtual KeyFrameDatabase* GetKeyFrameDatabase() = 0;
        virtual Map* GetMap() = 0;
        virtual Tracking* GetTracker() = 0;
        virtual LocalMapping* GetLocalMapper() = 0;
        virtual LoopClosing* GetLoopCloser() = 0;
        virtual IPublisherThread *GetPublisher() = 0;

        virtual void ResetKeyFrameDatabase(KeyFrameDatabase*) = 0;
        virtual void ResetMap(Map*) = 0;

        virtual bool LoadMap(const string &filename) = 0;
    };

    // This `IBuilder` implementation  constructs a system completely with the exception of
    // the IPublisherThread and directly related objects (IFrameSubscriber, IMapPublisher).
    // Since different Builders usually differ only by the class of those publishing objects,
    // it's usually more practical to subclass this class instead of the IBuilder.
    class GenericBuilder : public IBuilder {
    public:
        GenericBuilder(const std::string &strVocFile, const std::string &strSettingsFile, eSensor sensor);
        virtual ~GenericBuilder();

        virtual bool GetMapSaveOption() override;
        virtual string GetMapfileName() override;
        virtual bool GetMapReuseOption() override;
        virtual eSensor GetSensorType() override;
        virtual ORBVocabulary *GetVocabulary() override;
        virtual KeyFrameDatabase *GetKeyFrameDatabase() override;
        virtual Map *GetMap() override;
        virtual Tracking *GetTracker() override;
        virtual LocalMapping *GetLocalMapper() override;
        virtual LoopClosing *GetLoopCloser() override;

        virtual void ResetKeyFrameDatabase(KeyFrameDatabase*) override;
        virtual void ResetMap(Map*) override;

        virtual bool LoadMap(const string &filename) override;

    protected:
        eSensor mSensor;
        cv::FileStorage mSettings;
        ORBVocabulary mVocabulary;

        string mapfile;
        int save_map;
        int load_map;
        bool bReuseMap;

        //De-serialzation calls the default Constructor and changes the adress
        //of the pointers to Map, KeyFrameDatabase.
        //Use the raw pointers below for de-serialization in LoadMap
        //and then reset the address of the smart pointers to point
        //to the new adress of the raw after de-serialzation.
        KeyFrameDatabase* pKeyFrameDatabase;
        Map* pMap;

        std::unique_ptr<KeyFrameDatabase> mpKeyFrameDatabase;
        std::unique_ptr<Map> mpMap;
        std::unique_ptr<Tracking> mpTracker;
        std::unique_ptr<LocalMapping> mpLocalMapper;
        std::unique_ptr<LoopClosing> mpLoopCloser;

    };

    // Creates the SLAM system.
    // The created `System` object takes ownership of the passed `Builder`.
    // The system will still need to be started by calling `System::Start()`.
    System(std::unique_ptr<IBuilder> builder);

    // Proccess the given stereo frame. Images must be synchronized and rectified.
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    cv::Mat TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp);

    // Process the given rgbd frame. Depthmap must be registered to the RGB frame.
    // Input image: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Input depthmap: Float (CV_32F).
    // Returns the camera pose (empty if tracking fails).
    cv::Mat TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp);

    // Proccess the given monocular frame
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    cv::Mat TrackMonocular(const cv::Mat &im, const double &timestamp);

    void Start();

    // This stops local mapping thread (map building) and performs only camera tracking.
    void ActivateLocalizationMode();
    // This resumes local mapping thread and performs SLAM again.
    void DeactivateLocalizationMode();

    // Returns true if there have been a big map change (loop closure, global BA)
    // since last call to this function
    bool MapChanged();

    // Reset the system (clear map)
    void Reset();

    // All threads will be requested to finish.
    // It waits until all threads have finished.
    // This function must be called before saving the trajectory.
    void Shutdown();

    // Save camera trajectory in the TUM RGB-D dataset format.
    // Only for stereo and RGB-D. This method does not work for monocular.
    // Call first Shutdown()
    // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
    void SaveTrajectoryTUM(const std::string &filename);

    // Save keyframe poses in the TUM RGB-D dataset format.
    // This method works for all sensor input.
    // Call first Shutdown()
    // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
    void SaveKeyFrameTrajectoryTUM(const std::string &filename);

    // Save camera trajectory in the KITTI dataset format.
    // Only for stereo and RGB-D. This method does not work for monocular.
    // Call first Shutdown()
    // See format details at: http://www.cvlibs.net/datasets/kitti/eval_odometry.php
    void SaveTrajectoryKITTI(const std::string &filename);


protected:
    // The `Builder` used to construct this `System`.
    std::unique_ptr<IBuilder> mpBuilder;

    // Information from most recent processed frame
    // You can call this right after TrackMonocular (or stereo or RGBD)
    int GetTrackingState();
    std::vector<MapPoint*> GetTrackedMapPoints();
    std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();

private:
      // Save/Load functions
      void SaveMap(const string &filename);

private:
    // Input sensor
    const eSensor mSensor;

    // ORB vocabulary used for place recognition and feature matching.
    ORBVocabulary* mpVocabulary;

    // KeyFrame database for place recognition (relocalization and loop detection).
    KeyFrameDatabase* mpKeyFrameDatabase;

    // Map structure that stores the pointers to all KeyFrames and MapPoints.
    Map* mpMap;

    string mapfile;
    //System gets this value via initializer list
    const bool is_save_map;

    bool bUseSavedMap;
    // Tracker. It receives a frame and computes the associated camera pose.
    // It also decides when to insert a new keyframe, create some new MapPoints and
    // performs relocalization if tracking fails.
    Tracking* mpTracker;

    // Local Mapper. It manages the local map and performs local bundle adjustment.
    LocalMapping* mpLocalMapper;

    // Loop Closer. It searches loops with every new keyframe. If there is a loop it performs
    // a pose graph optimization and full bundle adjustment (in a new thread) afterwards.
    LoopClosing* mpLoopCloser;

    IPublisherThread *mpPublisher;

    // System threads: Local Mapping, Loop Closing, Viewer.
    // The Tracking thread "lives" in the main execution thread that creates the System object.
    bool mbStarted;
    std::thread mtLocalMapping;
    std::thread mtLoopClosing;
    std::thread mtPublisher;

    // Reset flag
    std::mutex mMutexReset;
    bool mbReset;

    // Change mode flags
    std::mutex mMutexMode;
    bool mbActivateLocalizationMode;
    bool mbDeactivateLocalizationMode;

    // Tracking state
    int mTrackingState;
    std::vector<MapPoint*> mTrackedMapPoints;
    std::vector<cv::KeyPoint> mTrackedKeyPointsUn;
    std::mutex mMutexState;
};

}// namespace ORB_SLAM

#endif // SYSTEM_H
