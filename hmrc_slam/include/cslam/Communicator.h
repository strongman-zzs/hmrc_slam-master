
#ifndef CSLAM_COMMUNICATORV2_H_
#define CSLAM_COMMUNICATORV2_H_

//C++
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/bind.hpp>
#include <mutex>
#include <sstream>
#include <algorithm>

#include <time.h>

//ROS
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

//CSLAM
#include <hmrc_slam/config.h>
#include <hmrc_slam/estd.h>
#include <hmrc_slam/Datatypes.h>
#include <hmrc_slam/CentralControl.h>
#include <hmrc_slam/ORBVocabulary.h>
#include <hmrc_slam/KeyFrame.h>
#include <hmrc_slam/MapPoint.h>
#include <hmrc_slam/Map.h>
#include <hmrc_slam/Database.h>
#include <hmrc_slam/MapMatcher.h>
#include <hmrc_slam/Mapping.h>

//Msgs
#include <hmrcslam_msgs/Map.h>

using namespace std;
using namespace estd;

namespace hmrc_slam
{

//forward decs
class MapMatcher;
class Map;
class LocalMapping;
class MapMatcher;
class CentralControl;
class KeyFrameDatabase;
class KeyFrame;
class MapPoint;
//------------------

class Communicator : public boost::enable_shared_from_this<Communicator>
{
public:
    typedef boost::shared_ptr<Communicator> commptr;
    typedef boost::shared_ptr<KeyFrame> kfptr;
    typedef boost::shared_ptr<MapPoint> mpptr;
    typedef boost::shared_ptr<Map> mapptr;
    typedef boost::shared_ptr<KeyFrameDatabase> dbptr;
    typedef boost::shared_ptr<CentralControl> ccptr;
    typedef boost::shared_ptr<MapMatcher> matchptr;
    typedef boost::shared_ptr<LocalMapping> mappingptr;

    typedef pair<size_t,kfptr> AckPairKF;
    typedef pair<size_t,mpptr> AckPairMP;

    typedef pair<hmrcslam_msgs::KF,hmrcslam_msgs::KFred> msgKFPair;
    typedef pair<hmrcslam_msgs::MP,hmrcslam_msgs::MPred> msgMPPair;

    //---additional functions
    struct kfcmp{
        bool operator() (const kfptr pA, const kfptr pB) const;
    };

    struct mpcmp{
        bool operator() (const mpptr pA, const mpptr pB) const;
    };

public:
    //---constructor---
    Communicator(ccptr pCC, vocptr pVoc, mapptr pMap, dbptr pKFDB, bool bLoadedMap = false);

    //---main---
    void RunClient();
    void RunServer();

    //---getter/setter---
    void SetMapping(mappingptr pMapping) {mpMapping = pMapping;}
    void ChangeMap(mapptr pMap){mpMap = pMap;}
    void SetMapMatcher(matchptr pMatch) {mpMapMatcher = pMatch;}
    dbptr GetDbPtr(){return mpDatabase;}
    size_t GetClientId(){return mClientId;}
    idpair GetNearestKFid(){return mNearestKfId;}

    //---callbacks---
    void MapCbClient(hmrcslam_msgs::MapConstPtr pMsg);
    void MapCbServer(hmrcslam_msgs::MapConstPtr pMsg);
    void RequestReset();
    void RequestResetExternal();

    //--- data transfer---
    void PassKftoComm(kfptr pKf);
    void PassMptoComm(mpptr pMp);
    void DeleteMpFromBuffer(mpptr pMP);

protected:
    //---infrastructure---
    ccptr mpCC;
    mapptr mpMap;
    dbptr mpDatabase;
    vocptr mpVoc;
    mappingptr mpMapping;
    matchptr mpMapMatcher;
    size_t mClientId;
    const double mdPeriodicTime;
    bool mbLoadedMap = false; //indicates that map for this client was loaded from a file (only works for client 0)

    kfptr mpNearestKF; //client: store last nearest KF -- server: store current nearest KF
    idpair mNearestKfId;

    int mKfItBound;
    int mMpItBound;
    int mKfItBoundPub;
    int mMpItBoundPub;
    int mMaxMsgSentPerIt;

    ros::NodeHandle mNh;
    ros::NodeHandle mNhPrivate;

    ros::Publisher mPubMap;
    ros::Subscriber mSubMap;

    string mSubKfTopicName;
    string mSubMpTopicName;
    int mPubMapBufferSize, mSubMapBufferSize;

    //---buffer checks---
    bool CheckBufferKfIn();
    bool CheckBufferKfOut();
    bool CheckBufferMpIn();
    bool CheckBufferMpOut();

    //---publish/receive---
    void PublishMapServer();
    void PublishMapClient();
    double mdLastTimePub;
    void ProcessKfInServer();
    void ProcessKfInClient();
    void ProcessMpInServer();
    void ProcessMpInClient();
    size_t mMsgCountLastMapMsg;
    kfptr mpKFLastFront;
    size_t mnMaxKfIdSent;

    //---IO buffers---
    set<kfptr,kfcmp> mspBufferKfOut;
    set<mpptr,mpcmp> mspBufferMpOut;
    list<msgKFPair> mlBufKFin;
    list<msgMPPair> mlBufMPin;

    list<kfptr> mlpAddedKfs;

    set<size_t> msAcksKF;
    set<size_t> msAcksMP;
    void SetWeakAckKF(size_t id);
    void SetWeakAckMP(size_t id);
    size_t mnWeakAckKF;
    size_t mnWeakAckMP;
    list<AckPairKF> mlKfOpenAcks;
    list<AckPairMP> mlMpOpenAcks;

    //---Reset---
    void ResetIfRequested();
    void ResetCommunicator();
    void ResetDatabase();
    void ResetMap();
    void ResetMapping();
    void ResetMapMatcher();
    bool mbResetRequested;

    //--mutexes
    mutex mMutexBuffersOut;
    mutex mMutexBuffersIn;
    mutex mMutexMsgBuffer;
    mutex mMutexReset;
    mutex mMutexLastMsgId;
    mutex mMutexNearestKf;

    //--Final BA if interrupted
    size_t mnEmptyMsgs;

    //monitoring //CHECKHERE
    size_t mOutMapCount;
    size_t mServerMapCount;
    void CheckOrdering(hmrcslam_msgs::Map msgMap);
    void ShowBufferContent(bool bGetMutexBufferIn = false, bool bGetMutexBufferOut = false);
};

} //end ns

#endif
