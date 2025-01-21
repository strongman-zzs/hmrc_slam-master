
#ifndef CSLAM_MAPMERGER_H_
#define CSLAM_MAPMERGER_H_

//C++
#include <boost/shared_ptr.hpp>
#include <mutex>
#include <opencv2/opencv.hpp>

//ROS
#include <ros/ros.h>
#include <ros/publisher.h>

//CSLAM
#include <hmrc_slam/config.h>
#include <hmrc_slam/estd.h>
#include <hmrc_slam/Converter.h>
#include <hmrc_slam/Datatypes.h>
#include <hmrc_slam/CentralControl.h>
#include <hmrc_slam/Map.h>
#include <hmrc_slam/KeyFrame.h>
#include <hmrc_slam/Optimizer.h>
#include <hmrc_slam/MapMatcher.h>
#include <hmrc_slam/ClientHandler.h>
#include <hmrc_slam/MapPoint.h>

//THIRDPARTY
#include "thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

using namespace std;
using namespace estd;

namespace hmrc_slam{

//forward decs
class MapMatcher;
class ClientHandler;
class Map;
class CentralControl;
class KeyFrame;
class MapPoint;
struct MapMatchHit;
//-----------------

class MapMerger
{
public:
    typedef boost::shared_ptr<MapMerger> mergeptr;
    typedef boost::shared_ptr<MapMatcher> matchptr;
    typedef boost::shared_ptr<ClientHandler> chptr;
    typedef boost::shared_ptr<KeyFrame> kfptr;
    typedef boost::shared_ptr<MapPoint> mpptr;
    typedef boost::shared_ptr<Map> mapptr;
    typedef boost::shared_ptr<CentralControl> ccptr;
public:
    typedef pair<set<kfptr>,int> ConsistentGroup;
    typedef map<kfptr,g2o::Sim3,std::less<kfptr>,
        Eigen::aligned_allocator<std::pair<const kfptr, g2o::Sim3> > > KeyFrameAndPose;
public:
    MapMerger(matchptr pMatcher);
    mapptr MergeMaps(mapptr pMapCurr, mapptr pMapMatch, vector<MapMatchHit> vMatchHits);

    bool isBusy();
private:
    void SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap, std::vector<mpptr> vpLoopMapPoints);
    void SetBusy();
    void SetIdle();

    matchptr mpMatcher;

    bool bIsBusy;
    mutex mMutexBusy;

    std::vector<kfptr> mvpCurrentConnectedKFs;

    void RunGBA(idpair nLoopKf, mapptr pFusedMap);
};

} //end namespace

#endif
