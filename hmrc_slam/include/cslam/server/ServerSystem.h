
#ifndef CSLAM_SERVERSYSTEM_H_
#define CSLAM_SERVERSYSTEM_H_

//C++
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <thread>
#include <mutex>

//ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

//CSLAM
#include <hmrc_slam/config.h>
#include <hmrc_slam/estd.h>
#include <hmrc_slam/Datatypes.h>
#include <hmrc_slam/ORBVocabulary.h>
#include <hmrc_slam/Map.h>
#include <hmrc_slam/Database.h>
#include <hmrc_slam/ClientHandler.h>
#include <hmrc_slam/MapMatcher.h>
#include <hmrc_slam/Viewer.h>

#include "hmrcslam/ServiceSaveMap.h"

using namespace std;
using namespace estd;

namespace hmrc_slam{

class ServerSystem
{
public:
    typedef boost::shared_ptr<ClientHandler> chptr;
    typedef boost::shared_ptr<KeyFrameDatabase> dbptr;
    typedef boost::shared_ptr<Map> mapptr;
    typedef boost::shared_ptr<MapMatcher> matchptr;
    typedef boost::shared_ptr<Viewer> viewptr;
    typedef boost::shared_ptr<CentralControl> ccptr;
public:
    ServerSystem(ros::NodeHandle Nh, ros::NodeHandle NhPrivate, const string &strVocFile);
    void InitializeClients();
    void InitializeMapMatcher();

    bool CallbackSaveMap(hmrcslam::ServiceSaveMap::Request &req, hmrcslam::ServiceSaveMap::Response &res);

private:
    void LoadVocabulary(const string &strVocFile);
    void InitializeMaps();
    void InitializeKFDB();
    void InitializeMapping();
    void InitializeViewer();

    void CleanWriteOutFile(std::string sFileName);

    //ROS infrastructure
    ros::NodeHandle mNh;
    ros::NodeHandle mNhPrivate;

    ros::ServiceServer mServiceSavemap;

    vocptr mpVoc;
    dbptr mpKFDB;
    matchptr mpMapMatcher;
    viewptr mpViewer;

    chptr mpClient0;
    mapptr mpMap0;
    chptr mpClient1;
    mapptr mpMap1;
    chptr mpClient2;
    mapptr mpMap2;
    chptr mpClient3;
    mapptr mpMap3;

    const uidptr mpUID;

    //threads
    threadptr mptMapMatching;
    threadptr mptViewer;

    int mNumOfClients;
    int mMaxClients;

    #ifdef LOGGING
    boost::shared_ptr<estd::mylog> mpLogger;
    threadptr mptLogger;
    #endif
};

} //end namespace

#endif
