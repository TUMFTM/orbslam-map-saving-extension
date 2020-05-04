//
// Created by sebastiano on 8/18/16.
//

#include "ROSPublisher.h"
#include "FrameDrawer.h"
#include "Tracking.h"
#include "utils.h"

#include <thread>
#include <sstream>
#include <cassert>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <orb_slam2_ros/ORBState.h>
#include <cv_bridge/cv_bridge.h>

#include <octomap/Pointcloud.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>

#include <chrono>


#include <iostream>
#include <fstream>
#include <nav_msgs/Odometry.h>

using namespace ORB_SLAM2;


static bool isBigEndian()
{
    volatile int num = 1;
    return *((char*) &num) == ((char) 1);
}

static const bool IS_BIG_ENDIAN = isBigEndian();

namespace std {
    std::string to_string(const cv::Mat& mat) {
        std::stringstream ss;
        ss << mat;
        return ss.str();
    }
}
// Konvertiert Rot. Matrix (SO) zu Quaternion
template<typename Q>
Q convertToQuaternion(const cv::Mat& rot)
{
    double trace = rot.at<float>(0,0) + rot.at<float>(1,1) + rot.at<float>(2,2);
    double tmp[4];

    if (trace > 0.0) {
        double s = sqrt(trace + 1.0);
        tmp[3] = s * 0.5;
        s = 0.5 / s;
        tmp[0] = ((rot.at<float>(2,1) - rot.at<float>(1,2)) * s);
        tmp[1] = ((rot.at<float>(0,2) - rot.at<float>(2,0)) * s);
        tmp[2] = ((rot.at<float>(1,0) - rot.at<float>(0,1)) * s);
    } else {
        int i;
        if (rot.at<float>(0, 0) < rot.at<float>(1,1))
            i = rot.at<float>(1,1) < rot.at<float>(2,2) ? 2 : 1;
        else
            i = rot.at<float>(0,0) < rot.at<float>(2,2) ? 2 : 0;
        int j = (i + 1) % 3;
        int k = (i + 2) % 3;

        double s = sqrt(rot.at<float>(i,i) - rot.at<float>(j,j) - rot.at<float>(k,k) + 1.0);
        tmp[i] = s * 0.5;
        s = 0.5 / s;
        tmp[3] = (rot.at<float>(k,j) - rot.at<float>(j,k)) * s;
        tmp[j] = (rot.at<float>(j,i) + rot.at<float>(i,j)) * s;
        tmp[k] = (rot.at<float>(k,i) + rot.at<float>(i,k)) * s;
    }

    return {tmp[0], tmp[1], tmp[2], tmp[3]};
}

cv::Mat computeCameraTransform(const cv::Mat& Twc)
{
    cv::Mat ret = cv::Mat::eye(4, 4, CV_32F);

    if(!Twc.empty()) {
        // auto for variables, specifies that the type of the variable that is being declared will be automatically deduced from its initializer
        auto Rwc = Twc.rowRange(0,3).colRange(0,3).t();
        ret.rowRange(0,3).colRange(0,3) = Rwc;
        // twc, the position
        ret.rowRange(0,3).col(3) = -Rwc* Twc.rowRange(0, 3).col(3);
    }
    return ret;
}

sensor_msgs::PointCloud2 convertToPCL2(const std::vector<MapPoint*> &map_points)
{
    const std::size_t n_map_points = map_points.size();
    ROS_INFO("sending PointCloud (%lu points)", n_map_points);

    // Kind of a hack, but there aren't much better ways to avoid a copy
    struct point { float x, y, z; };

    std::vector<uint8_t> data_buffer(n_map_points * sizeof(point));
    std::size_t vtop = 0;

    point *dataptr = (point*) data_buffer.data();

    for (MapPoint *map_point : map_points) {
        if (map_point->isBad())
            continue;
        cv::Mat pos = map_point->GetWorldPos();
        dataptr[vtop++] = {
            pos.at<float>(0),
            pos.at<float>(1),
            pos.at<float>(2),
        };
    }

    static const char* const names[3] = { "x", "y", "z" };
    static const std::size_t offsets[3] = { offsetof(point, x), offsetof(point, y), offsetof(point, z) };
    std::vector<sensor_msgs::PointField> fields(3);
    for (int i=0; i < 3; i++) {
        fields[i].name = names[i];
        fields[i].offset = offsets[i];
        fields[i].datatype = sensor_msgs::PointField::FLOAT32;
        fields[i].count = 1;
    }

    sensor_msgs::PointCloud2 msg;
    msg.height = 1;
    msg.width = n_map_points;
    msg.fields = fields;
    msg.is_bigendian = IS_BIG_ENDIAN;
    msg.point_step = sizeof(point);
    msg.row_step = sizeof(point) * msg.width;
    msg.data = std::move(data_buffer);
    msg.is_dense = true;  // invalid points already filtered out

    return msg;
}

/*
 * Returns a ROS parameter as generic type, defaulting to a given value if it is unspecified.
 */
template<typename T>
T getROSParam(ros::NodeHandle nh, std::string param_name, T default_value)
{
    T result;
    nh.param<T>(param_name, result, default_value);
    return result;
}

//Aufruf Constructor
ROSPublisher::ROSPublisher(Map *map, double frequency, bool bReuseMap_, ros::NodeHandle nh) :

    //Aufruf Constructor der Basisklassen
    IMapPublisher(map),
    drawer_(GetMap(), bReuseMap_),
    nh_(std::move(nh)),
    pub_rate_(frequency),
    lastBigMapChange_(-1),
    octomap_tf_based_(false),
    octomap_(getROSParam<float>(nh, "octomap_resolution", ROSPublisher::DEFAULT_OCTOMAP_RESOLUTION)),
    pointcloud_chunks_stashed_(0),
    clear_octomap_(false)
{
    //Odys: As in Viewer and Tracking, if a map is loaded we start with state as LOST
    if(bReuseMap_)
      orb_state_.state = orb_slam2_ros::ORBState::LOST;
    else
      orb_state_.state = orb_slam2_ros::ORBState::UNKNOWN;


    // initialize parameters
    nh.param<bool>("octomap_enabled", octomap_enabled_, true);
    nh.param<double>("occupancy_projection_min_height", projection_min_height_, ROSPublisher::PROJECTION_MIN_HEIGHT);
    nh.param<float>("occupancy_gradient_max_height", gradient_max_height_, ROSPublisher::GRADIENT_MAX_HEIGHT);
    nh.param<int>("occupancy_gradient_nb_erosions", gradient_nb_erosions_, ROSPublisher::GRADIENT_NB_EROSIONS);
    nh.param<float>("occupancy_gradient_low_slope", gradient_low_slope_, ROSPublisher::GRADIENT_LOW_SLOPE);
    nh.param<float>("occupancy_gradient_high_slope", gradient_high_slope_, ROSPublisher::GRADIENT_HIGH_SLOPE);

    camera_position_old={0.0, 0.0, 0.0};

    // initialize publishers
    // map includes all keypoint extracted during the slam prozes
    map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("map", 3);
    map_updates_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("map_updates", 3);

     //  odom_publisher_ = nh_.advertise<OdomMsg>("odom", 4);
    odom_publisher_ = nh_.advertise<nav_msgs::Odometry>("odom", queue_size_);

    // frame is an image, it shows where ORB-SLAM2 is extracting the orb features, can monitor if its working fine
    image_pub_ = nh_.advertise<sensor_msgs::Image>("frame", 5);
    state_pub_ = nh_.advertise<orb_slam2_ros::ORBState>("state", 10);
    state_desc_pub_ = nh_.advertise<std_msgs::String>("state_description", 10);

    if (octomap_enabled_)
    {
        octomap_pub_ = nh_.advertise<octomap_msgs::Octomap>("octomap", 3);
        projected_map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("projected_map", 5, 10);
        gradient_map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("gradient_map", 5, 10);

        // start octomap worker thread
        octomap_worker_thread_ = std::thread( [this] { octomapWorker(); } );
    }
}

/*
 * Either appends all GetReferenceMapPoints to the pointcloud stash or clears the stash and re-fills it
 * with GetAllMapPoints, in case there is a big map change in ORB_SLAM 2 or all_map_points is set to true.
 */
void ROSPublisher::stashMapPoints(bool all_map_points)
{
    std::vector<MapPoint*> map_points;

    pointcloud_map_points_mutex_.lock();

    if (all_map_points || GetMap()->GetLastBigChangeIdx() > lastBigMapChange_)
    {
        map_points = GetMap()->GetAllMapPoints();
        lastBigMapChange_ = GetMap()->GetLastBigChangeIdx();
        clear_octomap_ = true;
        pointcloud_map_points_.clear();
        pointcloud_chunks_stashed_ = 1;
    } else {
        map_points = GetMap()->GetReferenceMapPoints();
        pointcloud_chunks_stashed_++;
    }

    for (MapPoint *map_point : map_points) {
        if (map_point->isBad())
            continue;
        cv::Mat pos = map_point->GetWorldPos();
        pointcloud_map_points_.push_back(pos.at<float>(0), pos.at<float>(1), pos.at<float>(2));
    }

    pointcloud_map_points_mutex_.unlock();
}

/*
 * Octomap worker thread function, which has exclusive access to the octomap. Updates and publishes it.
 */
void ROSPublisher::octomapWorker()
{

    static std::chrono::system_clock::time_point this_cycle_time;

    octomap::pose6d frame;
    bool got_tf;
    octomap::point3d origin;

    // wait until ORB_SLAM 2 is up and running
    ROS_INFO("octomapWorker thread: waiting for ORBState OK");
    while (orb_state_.state != orb_slam2_ros::ORBState::OK)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }

    ROS_INFO("octomapWorker thread: starting to work (ORBState is OK)");
    // main thread loop
    while (!isStopped())
    {
        this_cycle_time = std::chrono::system_clock::now();

        // camera_position_ is computed in publishCameraPose() (void ROSPublisher::Run()
        // cv::Mat xf = computeCameraTransform(GetCameraPose())

        origin = {camera_position_.x(), camera_position_.y(), camera_position_.z()};


        try {
            tf::StampedTransform transform_in_target_frame;
            tf_listener_.lookupTransform(ROSPublisher::DEFAULT_BASE_FRAME, ROSPublisher::DEFAULT_CAMERA_FRAME, ros::Time(0), transform_in_target_frame);
            frame = octomap::poseTfToOctomap(transform_in_target_frame);
            got_tf = true;

        } catch (tf::TransformException &ex) {
            frame = octomap::pose6d(0, 0, 0, 0, 0, 0);
            got_tf = false;

        }


        if (got_tf || ROSPublisher::OCTOMAP_REBUILD)
        {
            clear_octomap_ |= (got_tf != octomap_tf_based_); // clear whenever TF mode changes

            if (clear_octomap_)
            {
                octomap_.clear(); // clears the whole octomap, problem for projected map
                ROS_INFO("octomapWorker thread: octomap cleared, rebuilding...");

                // TODO: if pointcloud is supposed to be a lidar scan result, this is problematic (multiple hits on one beam/previous hits getting overwritten etc.)
                stashMapPoints(true); // stash whole map
                clear_octomap_ = false; // TODO: mutex?
            }

            pointcloud_map_points_mutex_.lock();
            // insert keypoint to the octomap
            octomap_.insertPointCloud(pointcloud_map_points_, origin, frame);
            pointcloud_map_points_.clear(); //
            int pointcloud_chunks_stashed = pointcloud_chunks_stashed_;
            pointcloud_chunks_stashed_ = 0;
            pointcloud_map_points_mutex_.unlock();

            octomap_tf_based_ = got_tf;

            publishOctomap();
            publishProjectedMap();
            publishGradientMap();

            ROS_INFO("octomapWorker thread: finished cycle integrating %i pointcloud chunks.", pointcloud_chunks_stashed);
        }
        else
        {
            ROS_INFO("octomapWorker thread: missing camera TF, losing %i pointcloud chunks.", pointcloud_chunks_stashed_);
            pointcloud_map_points_mutex_.lock();
            pointcloud_map_points_.clear();
            pointcloud_chunks_stashed_ = 0;
            pointcloud_map_points_mutex_.unlock();
        }


        std::this_thread::sleep_until(this_cycle_time + std::chrono::milliseconds((int) (1000. / ROSPublisher::OCTOMAP_RATE)));
    }

    ROS_INFO("octomapWorker thread: stopped");
}

/*
 * Creates a 2D Occupancy Grid from the Octomap. Cuts out an interval between minZ_ and maxZ_ and projekts them from the octomap into an occupancy grid map
 */
void ROSPublisher::octomapCutToOccupancyGrid(const octomap::OcTree& octree, nav_msgs::OccupancyGrid& map, const double minZ_, const double maxZ_ )
{
    map.info.resolution = octree.getResolution();
    double minX, minY, minZ;
    double maxX, maxY, maxZ;
    octree.getMetricMin(minX, minY, minZ);
    octree.getMetricMax(maxX, maxY, maxZ);
    ROS_DEBUG("Octree min %f %f %f", minX, minY, minZ);
    ROS_DEBUG("Octree max %f %f %f", maxX, maxY, maxZ);
    minZ = std::max(minZ_, minZ);
    maxZ = std::min(maxZ_, maxZ);

    octomap::point3d minPt(minX, minY, minZ);
    octomap::point3d maxPt(maxX, maxY, maxZ);
    octomap::OcTreeKey minKey, maxKey, curKey;

    if (!octree.coordToKeyChecked(minPt, minKey))
    {
        ROS_ERROR("Could not create OcTree key at %f %f %f", minPt.x(), minPt.y(), minPt.z());
        return;
    }
    if (!octree.coordToKeyChecked(maxPt, maxKey))
    {
        ROS_ERROR("Could not create OcTree key at %f %f %f", maxPt.x(), maxPt.y(), maxPt.z());
        return;
    }

    map.info.width = maxKey[0] - minKey[0] + 1;
    map.info.height = maxKey[1] - minKey[1] + 1;

    // might not exactly be min / max:
    octomap::point3d origin =   octree.keyToCoord(minKey, octree.getTreeDepth());
    map.info.origin.position.x = origin.x() - octree.getResolution() * 0.5;
    map.info.origin.position.y = origin.y() - octree.getResolution() * 0.5;

    map.info.origin.orientation.x = 0.;
    map.info.origin.orientation.y = 0.;
    map.info.origin.orientation.z = 0.;
    map.info.origin.orientation.w = 1.;

    // Allocate space to hold the data
    map.data.resize(map.info.width * map.info.height, -1);

    //init with unknown
    for(std::vector<int8_t>::iterator it = map.data.begin(); it != map.data.end(); ++it) {
      *it = -1;
    }

    // iterate over all keys (key is internal adress of octree node/leaf. each keypoint is represented as node of octree)
    // every ocupied node is occupied grid cell in ocupancy grid map
    unsigned i, j;
    for (curKey[1] = minKey[1], j = 0; curKey[1] <= maxKey[1]; ++curKey[1], ++j)
    {
        for (curKey[0] = minKey[0], i = 0; curKey[0] <= maxKey[0]; ++curKey[0], ++i)
        {
            for (curKey[2] = minKey[2]; curKey[2] <= maxKey[2]; ++curKey[2])
            { //iterate over height
                octomap::OcTreeNode* node = octree.search(curKey);
                if (node)
                {
                    bool occupied = octree.isNodeOccupied(node);
                    if(occupied) {
                        map.data[map.info.width * j + i] = 100;
                        break;
                    } else {
                        map.data[map.info.width * j + i] = 0;
                    }
                }
            }
        }
    }
}

/*
 * Replaces NaN values with the mean of their 8 neighbors whenever possible.
 * Does it n times. Returns number of eroded NaNs.
 */
int erodeNaN(cv::Mat &matrix, int n)
{
    cv::Mat matrix_old = matrix.clone(); // use original matrix when looking up neighbors

    int nb_eroded_cells = 0; // rather for information/debug purposes
    for (int i = 0; i < n; ++i) // erode n times
    {
        for (int x = 0; x < matrix.cols; ++x) // iterate over matrix
        {
            for (int y = 0; y < matrix.rows; ++y)
            {
                float current_value = matrix_old.at<float>(y, x);
                if (current_value != current_value) // is NaN
                {
                    int nb_values = 0;
                    float sum = 0;
                    for (int dx = -1; dx < 2; ++dx) // iterate over neighborhood
                    {
                        for (int dy = -1; dy < 2; ++dy)
                        {
                            if ((x + dx >= 0) && (y + dy >= 0) && (x + dx < matrix.cols) && (y + dy < matrix.rows)) // within matrix bounds
                            {
                                current_value = matrix_old.at<float>(y + dy, x + dx);
                                if (current_value == current_value) // is not NaN
                                {
                                    sum += current_value;
                                    nb_values++;
                                }
                            }
                        }
                    }
                    if (nb_values > 0) // there were non-NaN neighbors
                    {
                        matrix.at<float>(y, x) = sum / nb_values;
                        nb_eroded_cells++;
                    }
                }
            }
        }
    }
    return nb_eroded_cells;
}

/*
 * Writes a fully contrast-scaled version of a 2-dimensional 1-channel matrix into a grayscale image file.
 */
void grayscaleToFile(const string& filename, const cv::Mat& img)
{
    double min_value = 0, max_value = 0;
    cv::minMaxLoc(img, &min_value, &max_value, 0, 0, img == img); // img==img masks out NaNs

    cv::Mat out;
    double scale = 255. / (max_value - min_value);
    double shift = scale * -min_value;
    img.convertTo(out, CV_8U, scale, shift);
    out.setTo(128, img != img); // set NaN to gray
    cv::imwrite(filename, out);
}

/*
 * Constructs a 2-dimensional OccupancyGrid from an Octomap by evaluating its heightmap gradients.
 */
void ROSPublisher::octomapGradientToOccupancyGrid(const octomap::OcTree& octree, nav_msgs::OccupancyGrid& map, float max_height, int nb_erosions, float low_slope, float high_slope)
{
    // get tree dimensions
    double min_x, min_y, min_z;
    double max_x, max_y, max_z;
    octree.getMetricMin(min_x, min_y, min_z);
    octree.getMetricMax(max_x, max_y, max_z);
    octomap::point3d min_point(min_x, min_y, min_z);
    octomap::point3d max_point(max_x, max_y, max_z);

    // fill in map dimensions
    map.info.resolution = octree.getResolution();
    map.info.width = (max_point.x() - min_point.x()) / map.info.resolution + 1;
    map.info.height = (max_point.y() - min_point.y()) / map.info.resolution + 1;

    map.info.origin.position.x = min_point.x() - map.info.resolution * 0.5;
    map.info.origin.position.y = min_point.y() - map.info.resolution * 0.5;
    map.info.origin.orientation.x = 0.;
    map.info.origin.orientation.y = 0.;
    map.info.origin.orientation.z = 0.;
    map.info.origin.orientation.w = 1.;

    // create CV matrix of proper size with 1 channel of 32 bit floats and init values to NaN for "unknown"
    cv::Mat height_map(map.info.height, map.info.width, CV_32FC1, NAN);

    // iterate over tree leafs to create height map
    octomap::point3d coord;
    int x, y;
    float z;
    for(octomap::OcTree::leaf_iterator it = octree.begin_leafs(), end=octree.end_leafs(); it != end; ++it)
    {
        if (octree.isNodeOccupied(*it))
        {
            coord = it.getCoordinate();
            x = (coord.x() - min_point.x()) / map.info.resolution;
            y = (coord.y() - min_point.y()) / map.info.resolution;
            z = coord.z();// z-axis is facing UP

            //  ROS_INFO("z: %f", z);


            if (z <= max_height) // only consider voxels up to specified height (e.g. for building indoor maps)
            {
                float current_height = height_map.at<float>(y, x);
                // replace if 1) current_height is Nan or 2) z is bigger than current_height
                if (current_height != current_height || z > current_height)
                {
                    height_map.at<float>(y, x) = z;
                }
            }
        }
    }

    // fill in small holes
    erodeNaN(height_map, nb_erosions);
    // store where height is unknown
    cv::Mat mask_unknown = height_map != height_map; // is NaN

    erodeNaN(height_map, 1); // avoid discontinuity (and thus a "wall") around known area

    height_map.setTo(0, height_map != height_map); // get rid of all NaN trouble makers

    // get height gradient
    cv::Mat gradient_x, gradient_y, gradient_map;
    cv::Scharr(height_map, gradient_x, CV_32F, 1, 0, 1. / 16.);
    cv::Scharr(height_map, gradient_y, CV_32F, 0, 1, 1. / 16.);
    cv::addWeighted(cv::abs(gradient_x), 0.5, cv::abs(gradient_y), 0.5, 0, gradient_map); // TODO 0.5 rly?

    // height slope thresholds:
    // values < lower are considered free space
    // values > upper are considered obstacle
    // everything inbetween is literally a gray-zone
    float threshold_lower = sin(low_slope) / cos(low_slope) * map.info.resolution;
    float threshold_upper = sin(high_slope) / cos(high_slope) * map.info.resolution;


    // map data probabilities are in range [0,100].  Unknown is -1.
    gradient_map.setTo(threshold_upper, gradient_map > threshold_upper); // clip obstacles
    gradient_map.setTo(threshold_lower, gradient_map < threshold_lower); // clip free space
    gradient_map = (gradient_map - threshold_lower) / (threshold_upper - threshold_lower) * 100.0; // convert into map data range
    gradient_map.setTo(-1, mask_unknown); //replace NaNs

    // ensure correct size of map data vector
    map.data.resize(map.info.width * map.info.height);
    // fill in map data
    for(y = 0; y < gradient_map.rows; ++y) {
        for(x = 0; x < gradient_map.cols; ++x) {
            map.data[y * map.info.width + x] = gradient_map.at<float>(y, x);
        }
    }
}

static const char *stateDescription(orb_slam2_ros::ORBState orb_state)
{
    switch (orb_state.state) {
        case orb_slam2_ros::ORBState::SYSTEM_NOT_READY: return "System not ready";
        case orb_slam2_ros::ORBState::NO_IMAGES_YET: return "No images yet";
        case orb_slam2_ros::ORBState::NOT_INITIALIZED: return "Not initialized";
        case orb_slam2_ros::ORBState::OK: return "OK";
        case orb_slam2_ros::ORBState::LOST: return "Tracking lost";
    }

    return "???";
}

static const orb_slam2_ros::ORBState toORBStateMessage(Tracking::eTrackingState trackingState)
{
    orb_slam2_ros::ORBState state_msg;
    state_msg.state = orb_slam2_ros::ORBState::UNKNOWN;

    switch (trackingState) {
        case Tracking::SYSTEM_NOT_READY: state_msg.state = orb_slam2_ros::ORBState::SYSTEM_NOT_READY;
                                         break;
        case Tracking::NO_IMAGES_YET:    state_msg.state = orb_slam2_ros::ORBState::NO_IMAGES_YET;
                                         break;
        case Tracking::NOT_INITIALIZED:  state_msg.state = orb_slam2_ros::ORBState::NOT_INITIALIZED;
                                         break;
        case Tracking::OK:               state_msg.state = orb_slam2_ros::ORBState::OK;
                                         break;
        case Tracking::LOST:             state_msg.state = orb_slam2_ros::ORBState::LOST;
                                         break;
    }

    return state_msg;
}

/*
 * Publishes ORB_SLAM 2 GetAllMapPoints() as a PointCloud2.
 */
void ROSPublisher::publishMap()
{
    if (map_pub_.getNumSubscribers() > 0)
    {
        auto msg = convertToPCL2(GetMap()->GetAllMapPoints());
        msg.header.frame_id = ROSPublisher::DEFAULT_MAP_FRAME;
        map_pub_.publish(msg);
    }
}

/*
 * Publishes ORB_SLAM 2 GetReferenceMapPoints() as a PointCloud2.
 */
void ROSPublisher::publishMapUpdates()
{
    if (map_updates_pub_.getNumSubscribers() > 0)
    {
        auto msg = convertToPCL2(GetMap()->GetReferenceMapPoints());
        msg.header.frame_id = ROSPublisher::DEFAULT_MAP_FRAME;
        map_updates_pub_.publish(msg);
    }
}

/*
 * Publishes ORB_SLAM 2 GetCameraPose() as a TF.
 */
void ROSPublisher::publishCameraPose()
{
    // number of subscribers is unknown to a TransformBroadcaster

    cv::Mat xf = computeCameraTransform(GetCameraPose());

    if (!xf.empty()) {
        camera_position_ = { xf.at<float>(0, 3), xf.at<float>(1, 3), xf.at<float>(2, 3) };
        auto orientation = convertToQuaternion<tf::Quaternion>(xf);
        tf::StampedTransform transform(
            tf::Transform(orientation, camera_position_),
            ros::Time::now(), ROSPublisher::DEFAULT_MAP_FRAME, ROSPublisher::DEFAULT_CAMERA_FRAME);
        camera_tf_pub_.sendTransform(transform);

        // get the stamp of transform and call the function to publish odom
        // pass the quaternion orientation
        // used only with RC car
        //ros::Time stamp = transform.stamp_;
        //ROSPublisher::publishOdom(orientation);
        ResetCamFlag();


    }
    else
      std::cout << "In ROSPublisher::publishCameraPose ---- CameraTransform is empty!!!!" << endl;

}

// Added Odom publisher. Get the information from TF
void ROSPublisher::publishOdom(tf::Quaternion orientation)
{

    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = ROSPublisher::DEFAULT_MAP_FRAME;

    tf::Quaternion q_orig, q_rot, q_new;
    // rotation pitch -90° with quaternion, because orbslam2 drives forward in z-direction (optical coordinate system)
    q_rot = tf::createQuaternionFromRPY(0, -M_PI/2, 0);
    tf::poseTFToMsg(tf::Transform(orientation, camera_position_), odom.pose.pose);
    q_orig = orientation;

    q_new=q_rot*q_orig;
    q_new.normalize();
    quaternionTFToMsg(q_new, odom.pose.pose.orientation);

    // velocity
    geometry_msgs::Twist twist;

    // interval from which we get the velocity
    ros::Duration interval(0.1);
    ros::Time time=ros::Time(0);
    try {
        transformer.lookupTwist("/base_link", "/odom", time, interval, twist);
    }
    catch( tf::TransformException ex) {
        ROS_ERROR("transfrom exception : %s",ex.what());
    }

    odom.twist.twist = twist;
    odom_publisher_.publish(odom);
}



/*
 * Publishes the previously built Octomap. (called from the octomap worker thread)
 */
void ROSPublisher::publishOctomap()
{
    if (octomap_pub_.getNumSubscribers() > 0)
    {
        auto t0 = std::chrono::system_clock::now();
        octomap_msgs::Octomap msgOctomap;
        msgOctomap.header.frame_id = octomap_tf_based_ ?
                                     ROSPublisher::DEFAULT_MAP_FRAME_ADJUSTED :
                                     ROSPublisher::DEFAULT_MAP_FRAME;
        msgOctomap.header.stamp = ros::Time::now();
        if (octomap_msgs::binaryMapToMsg(octomap_, msgOctomap))   // TODO: full/binary...?
        {
            auto tn = std::chrono::system_clock::now();
            auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(tn - t0);
            t0 = std::chrono::system_clock::now();
            octomap_pub_.publish(msgOctomap);
            tn = std::chrono::system_clock::now();
            dt = std::chrono::duration_cast<std::chrono::milliseconds>(tn - t0);
        }
    }

    // write out the octomap to folder orbslam2_ws/devel/lib/orb_slam2_ros for debugging purpose
    //octomap_.write("my_octomap.ot");
}

/*
 * Publishes the ORB_SLAM 2 tracking state as ORBState int and/or as a description string.
 */
void ROSPublisher::publishState(Tracking *tracking)
{
    if (tracking != NULL) {
        // save state from tracking, even if there are no subscribers
        orb_state_ = toORBStateMessage(tracking->mState);
    }

    if (state_pub_.getNumSubscribers() > 0)
    {
        // publish state as ORBState int
        orb_state_.header.stamp = ros::Time::now();
        state_pub_.publish(orb_state_);
    }

    if (state_desc_pub_.getNumSubscribers() > 0)
    {
        // publish state as string
        std_msgs::String state_desc_msg;
        state_desc_msg.data = stateDescription(orb_state_);
        state_desc_pub_.publish(state_desc_msg);
    }

    last_state_publish_time_ = ros::Time::now();
}

/*
 * Publishes the current ORB_SLAM 2 status image.
 */
void ROSPublisher::publishImage(Tracking *tracking)
{
    if (image_pub_.getNumSubscribers() > 0)
    {
        drawer_.Update(tracking);

        std_msgs::Header hdr;
        cv_bridge::CvImage cv_img {hdr, "bgr8", drawer_.DrawFrame()};

        auto image_msg = cv_img.toImageMsg();
        image_msg->header = hdr;
        image_pub_.publish(*image_msg);
    }
}

/*
 * Creates a 2D OccupancyGrid from the Octomap by performing a cut through a previously specified z interval and publishes it.
 */
void ROSPublisher::publishProjectedMap()
{
    static nav_msgs::OccupancyGrid msg;
    if (projected_map_pub_.getNumSubscribers() > 0)
    {
        msg.header.frame_id = octomap_tf_based_ ?
                              ROSPublisher::DEFAULT_MAP_FRAME_ADJUSTED :
                              ROSPublisher::DEFAULT_MAP_FRAME;
        msg.header.stamp = ros::Time::now();

        // set the max height value of points to consider to 0.15, change regarding your case
        // projection_min_height_ set as parameter
        octomapCutToOccupancyGrid(octomap_, msg, projection_min_height_, 0.15);

        projected_map_pub_.publish(msg);

    }
}

/*
 * Creates a 2D OccupancyGrid from the Octomap by evaluating its heightmap gradients and publishes it.
 * For detailed info about this process see documentation in path roborace_ws/src/modules/mod_students/ORB_SLAM2-ros/orb_slam2_ros/doc
 */
void ROSPublisher::publishGradientMap()
{
    static nav_msgs::OccupancyGrid msg;
    if (gradient_map_pub_.getNumSubscribers() > 0)
    {
        msg.header.frame_id = octomap_tf_based_ ?
                              ROSPublisher::DEFAULT_MAP_FRAME_ADJUSTED :
                              ROSPublisher::DEFAULT_MAP_FRAME;
        msg.header.stamp = ros::Time::now();

        octomapGradientToOccupancyGrid(octomap_, msg, gradient_max_height_, gradient_nb_erosions_, gradient_low_slope_, gradient_high_slope_);

        gradient_map_pub_.publish(msg);
    }
}

void ROSPublisher::Run()
{
    using namespace std::this_thread;
    using namespace std::chrono;

    SetFinish(false);

    ROS_INFO("ROS publisher started");


    while (WaitCycleStart()) {
        // only publish map, map updates and camera pose, if camera pose was updated
        if (isCamUpdated()) {
            publishMap();
            publishMapUpdates();
            publishCameraPose();

            if (octomap_enabled_)
            {
                stashMapPoints(); // store current reference map points for the octomap worker
            }
        }

        if (ros::Time::now() >= last_state_publish_time_ + ros::Duration(1. / ORBSTATE_REPUBLISH_RATE))
        {
            // it's time to re-publish ORBState
            publishState(NULL);
        }
    }

    ROS_INFO("ROS publisher finished");
    SetFinish(true);
}

bool ROSPublisher::WaitCycleStart()
{
    if (!IPublisherThread::WaitCycleStart())
        return false;

    pub_rate_.sleep();
    return true;
}

void ROSPublisher::Update(Tracking *tracking)
{
    static std::mutex mutex;

    if (tracking == nullptr)
        return;

    publishState(tracking);
    publishImage(tracking);
}

ROSSystemBuilder::ROSSystemBuilder(const std::string& strVocFile,
                const std::string& strSettingsFile,
                ORB_SLAM2::System::eSensor sensor,
                double frequency,
                ros::NodeHandle nh,
                std::string map_frame,
                std::string camera_frame) :
    System::GenericBuilder(strVocFile, strSettingsFile, sensor)
{
    //When the GenericBuilder constructor runs, it will check if a map.bin is available
    //and set the var bReuseMap accordingly. Then we can load this var through
    //GetMapReuseOption()
    mpPublisher = make_unique<ROSPublisher>(
        GetMap(), frequency, GetMapReuseOption(), std::move(nh));
    mpTracker->SetFrameSubscriber(mpPublisher.get());
    mpTracker->SetMapPublisher(mpPublisher.get());
}

// Empty dtor to give a place to the calls to the dtor of unique_ptr members
ROSSystemBuilder::~ROSSystemBuilder() { }

IPublisherThread* ROSSystemBuilder::GetPublisher()
{
    return mpPublisher.get();
}
