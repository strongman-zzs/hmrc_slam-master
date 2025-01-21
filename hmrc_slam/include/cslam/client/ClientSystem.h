
#ifndef CSLAM_CLIENTSYSTEM_H_
#define CSLAM_CLIENTSYSTEM_H_

//C++
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <thread>
#include <mutex>

#include <time.h>

//ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

//CSLAM
#include <hmrc_slam/config.h>
#include <hmrc_slam/estd.h>
#include <hmrc_slam/Datatypes.h>
#include <hmrc_slam/Database.h>
#include <hmrc_slam/Map.h>
#include <hmrc_slam/ClientHandler.h>

using namespace std;

namespace hmrc_slam{

class ClientSystem
{
public:
    typedef boost::shared_ptr<ClientHandler> chptr;
    typedef boost::shared_ptr<KeyFrameDatabase> dbptr;
    typedef boost::shared_ptr<Map> mapptr;
public:
    ClientSystem(ros::NodeHandle Nh, ros::NodeHandle NhPrivate, const string &strVocFile, const string &strCamFile);

private:
    void LoadVocabulary(const string &strVocFile);

    //ROS infrastructure
    ros::NodeHandle mNh;
    ros::NodeHandle mNhPrivate;
    size_t mClientId;

    //MCP-SLAM Infrastructure
    vocptr mpVoc;
    const string mstrCamFile;
    dbptr mpKFDB;
    mapptr mpMap;
    chptr mpAgent;

    const uidptr mpUID;
};

} //end namespace

#endif
