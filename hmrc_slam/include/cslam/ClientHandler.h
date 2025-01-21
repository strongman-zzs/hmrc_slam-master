
#ifndef CSLAM_CLIENTHANDLER_H_
#define CSLAM_CLIENTHANDLER_H_

//C++
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <mutex>
#include <sstream>

//ROS
//...

//CSLAM
#include <hmrc_slam/config.h>
#include <hmrc_slam/estd.h>
#include <hmrc_slam/Datatypes.h>
#include <hmrc_slam/CentralControl.h>
#include <hmrc_slam/ORBVocabulary.h>
#include <hmrc_slam/Map.h>
#include <hmrc_slam/Database.h>
#include <hmrc_slam/Mapping.h>
#include <hmrc_slam/Communicator.h>
#include <hmrc_slam/MapMatcher.h>
#include <hmrc_slam/Mapping.h>
#include <hmrc_slam/Communicator.h>
#include <hmrc_slam/LoopFinder.h>
#include <hmrc_slam/Viewer.h>
#include <hmrc_slam/Tracking.h>

//Thirdparty
#include "thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

using namespace std;
using namespace estd;

namespace hmrc_slam{

//forward decs
class Tracking;
class Viewer;
class LoopFinder;
struct SystemEndpoint;
//----------------


class ClientHandler : public boost::enable_shared_from_this<ClientHandler>
{
public:
    typedef boost::shared_ptr<Tracking> trackptr;
    typedef boost::shared_ptr<Viewer> viewptr;
    typedef boost::shared_ptr<LocalMapping> mappingptr;
    typedef boost::shared_ptr<Communicator> commptr;
    typedef boost::shared_ptr<LoopFinder> lfptr;
    typedef boost::shared_ptr<Map> mapptr;
    typedef boost::shared_ptr<MapMatcher> matchptr;
    typedef boost::shared_ptr<CentralControl> ccptr;
    typedef boost::shared_ptr<KeyFrameDatabase> dbptr;
    typedef boost::shared_ptr<KeyFrame> kfptr;
public:
    ClientHandler(ros::NodeHandle Nh, ros::NodeHandle NhPrivate, vocptr pVoc, dbptr pDB, mapptr pMap, size_t ClientId, uidptr pUID, eSystemState SysState, const string &strCamFile, viewptr pViewer, bool bLoadMap = false);
    #ifdef LOGGING
    void InitializeThreads(boost::shared_ptr<estd::mylog> pLogger = nullptr);
    #else
    void InitializeThreads();
    #endif

    //---getter/setter---
    void SetMapMatcher(matchptr pMatch);
    void ChangeMap(mapptr pMap, g2o::Sim3 g2oS_wnewmap_wcurmap);
    commptr GetCommPtr(){return mpComm;}
    trackptr GetTrackPtr(){return mpTracking;}
    mappingptr GetMappingPtr(){return mpMapping;}
    dbptr GetDbPtr(){return mpKFDB;}
    vocptr GetVocPtr(){return mpVoc;}
    kfptr GetCurrentRefKFfromTracking();
    int GetNumKFsinLoopFinder();
    int GetNumKFsinMapMatcher();

    //---forwarding---
    void ClearCovGraph(size_t MapId);

    //---Agent side---
    void CamImgCb(sensor_msgs::ImageConstPtr pMsg);
    void Reset();

    //---Map Save/Load---
    void LoadMap(const string &path_name);
    void SaveMap(const string &path_name);
    bool mbLoadedMap = false; //indicates that map for this client was loaded from a file (only works for client 0)

//    #ifdef LOGGING
//    void SetLogger(boost::shared_ptr<estd::mylog> pLogger);
//    #endif

private:
    #ifdef LOGGING
    void InitializeCC(boost::shared_ptr<estd::mylog> pLogger);
    #else
    void InitializeCC();
    #endif
    void InitializeClient();
    void InitializeServer(bool bLoadMap = false);

    //infrastructure
    ccptr mpCC;
    mapptr mpMap;
    dbptr mpKFDB;
    vocptr mpVoc;
    commptr mpComm;
    mappingptr mpMapping;
    viewptr mpViewer;
    //agent only
    trackptr mpTracking;
    //server only
    lfptr mpLoopFinder;
    matchptr mpMapMatcher;
    uidptr mpUID;
    eSystemState mSysState;

    const string mstrCamFile;

    ros::NodeHandle mNh;
    ros::NodeHandle mNhPrivate;
    ros::Subscriber mSubCam;

    //threads
    threadptr mptMapping;
    threadptr mptComm;
    threadptr mptLoopClosure;
    threadptr mptViewer;

    //data
    size_t mClientId;
    g2o::Sim3 mg2oS_wcurmap_wclientmap; //transformation from map into client

    //reset
    bool mbReset;

    //mutexes
    mutex mMutexThreads;
    mutex mMutexReset;
};

} //end ns

#endif
