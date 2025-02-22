

#ifndef CSLAM_MAPPING_H_
#define CSLAM_MAPPING_H_

//C++
#include <boost/shared_ptr.hpp>
#include <mutex>

//CSLAM
#include <hmrc_slam/config.h>
#include <hmrc_slam/estd.h>
#include <hmrc_slam/Datatypes.h>
#include <hmrc_slam/CentralControl.h>
#include <hmrc_slam/KeyFrame.h>
#include <hmrc_slam/Map.h>
#include <hmrc_slam/MapMatcher.h>
#include <hmrc_slam/LoopFinder.h>
#include <hmrc_slam/Database.h>
#include <hmrc_slam/ORBmatcher.h>
#include <hmrc_slam/Optimizer.h>
#include <hmrc_slam/Communicator.h>
#include <hmrc_slam/Viewer.h>

using namespace std;
using namespace estd;

namespace hmrc_slam{

//forward decs
class KeyFrame;
class MapPoint;
class Map;
class MapMatcher;
struct CentralControl;
class LoopFinder;
class KeyFrameDatabase;
class Viewer;
//--------------------

class LocalMapping
{
public:
    typedef boost::shared_ptr<LocalMapping> mappingptr;
    typedef boost::shared_ptr<KeyFrame> kfptr;
    typedef boost::shared_ptr<MapPoint> mpptr;
    typedef boost::shared_ptr<Map> mapptr;
    typedef boost::shared_ptr<CentralControl> ccptr;
    typedef boost::shared_ptr<MapMatcher> matchptr;
    typedef boost::shared_ptr<LoopFinder> lfptr;
    typedef boost::shared_ptr<KeyFrameDatabase> dbptr;
    typedef boost::shared_ptr<Communicator> commptr;
    typedef boost::shared_ptr<Viewer> viewptr;
public:
    //constructor
    LocalMapping(ccptr pCC, mapptr pMap, dbptr pDB, viewptr pViewer);

    //getter/setter
    void SetCommunicator(commptr pComm) {mpComm = pComm;}
    void SetLoopFinder(lfptr pLF) {mpLoopFinder = pLF;}
    void SetMapMatcher(matchptr pMatch) {mpMapMatcher = pMatch;}

    void ChangeMap(mapptr pMap){mpMap = pMap;}

    //Forwarding
    void ClearCovGraph(size_t MapId);

    // Main function
    void RunClient();
    void RunServer();

    void InsertKeyFrame(kfptr pKF);

    // Thread Synch
    void RequestStop();
    void RequestReset();
    bool Stop();
    bool isStopped();
    bool stopRequested();
    bool AcceptKeyFrames();
    void SetAcceptKeyFrames(bool flag);
    bool SetNotStop(bool flag);

    void InterruptBA();

    void RequestFinish();
    bool isFinished();

    int KeyframesInQueue(){
        unique_lock<std::mutex> lock(mMutexNewKFs);
        return mlNewKeyFrames.size();
    }

protected:

    bool CheckNewKeyFrames();
    void ProcessNewKeyFrame();
    void CreateNewMapPoints();

    void MapPointCullingClient();
    void MapPointCullingServer();
    size_t mCountKFs;
    void SearchInNeighbors();

    void KeyFrameCullingV3();

    cv::Mat ComputeF12(kfptr &pKF1, kfptr &pKF2);

    cv::Mat SkewSymmetricMatrix(const cv::Mat &v);

    void ResetIfRequested();
    bool mbResetRequested;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;

    //infrastructure
    ccptr mpCC;
    mapptr mpMap;
    dbptr mpKFDB;
    commptr mpComm;
    lfptr mpLoopFinder;
    matchptr mpMapMatcher;
    viewptr mpViewer;

    //data
    std::list<kfptr> mlNewKeyFrames;
    std::list<kfptr> mlKfsForLBA;
    kfptr mpCurrentKeyFrame;
    std::list<mpptr> mlpRecentAddedMapPoints;
    std::list<kfptr> mlpRecentAddedKFs;
    set<kfptr>mspKFsCheckedForCulling;

    //reset/interrupt
    bool mbAbortBA;

    bool mbStopped;
    bool mbStopRequested;
    bool mbNotStop;

    bool mbAcceptKeyFrames;

    //mutexes
    std::mutex mMutexAccept;
    std::mutex mMutexStop;
    std::mutex mMutexNewKFs;
    std::mutex mMutexReset;
    std::mutex mMutexFinish;

    size_t mClientId;

    //monitoring of added vs culled KFs
    size_t mAddedKfs, mCulledKfs;
};

} //end namespace

#endif
