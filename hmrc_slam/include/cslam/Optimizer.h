
#ifndef CSLAM_OPTIMIZER_H_
#define CSLAM_OPTIMIZER_H_

//C++
#include<Eigen/StdVector>
#include<mutex>

//CSLAM
#include <hmrc_slam/config.h>
#include <hmrc_slam/estd.h>
#include <hmrc_slam/Converter.h>
#include <hmrc_slam/Frame.h>
#include <hmrc_slam/KeyFrame.h>
#include <hmrc_slam/MapPoint.h>
#include <hmrc_slam/Map.h>
#include <hmrc_slam/Communicator.h>
#include <hmrc_slam/Datatypes.h>

//Thirdparty
#include "thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "thirdparty/g2o/g2o/core/block_solver.h"
#include "thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

using namespace std;
using namespace estd;

namespace hmrc_slam{

#define IDRANGE 1000000
#define MAXAGENTS 4

//forward decs
class Communicator;
class Map;
class Frame;
class KeyFrame;
class MapPoint;
//------------

class Optimizer
{
public:
    typedef boost::shared_ptr<KeyFrame> kfptr;
    typedef boost::shared_ptr<MapPoint> mpptr;
    typedef boost::shared_ptr<Map> mapptr;
    typedef boost::shared_ptr<Communicator> commptr;

    typedef pair<set<kfptr>,int> ConsistentGroup;
    typedef map<kfptr,g2o::Sim3,std::less<kfptr>,
        Eigen::aligned_allocator<std::pair<const kfptr, g2o::Sim3> > > KeyFrameAndPose;
public:

    //-----------client-----------

    void static BundleAdjustmentClient(const std::vector<kfptr> &vpKF, const std::vector<mpptr> &vpMP, size_t ClientId,
                                 int nIterations = 5, bool *pbStopFlag=NULL, const idpair nLoopKF=make_pair(0,0),
                                 const bool bRobust = true);
    void static GlobalBundleAdjustemntClient(mapptr pMap, size_t ClientId, int nIterations=5, bool *pbStopFlag=NULL,
                                       const idpair nLoopKF=make_pair(0,0), const bool bRobust = true);
    void static LocalBundleAdjustmentClient(kfptr pKF, bool *pbStopFlag, mapptr pMap, size_t ClientId, eSystemState SysState = eSystemState::CLIENT);

    int static PoseOptimizationClient(Frame &Frame);


    //-----------server-----------

    void static MapFusionGBA(mapptr pMap, size_t ClientId, int nIterations=5, bool *pbStopFlag=NULL,
                             idpair nLoopKF = make_pair(0,0), const bool bRobust = true);

    // if bFixScale is true, optimize SE3 (stereo,rgbd), Sim3 otherwise (mono)
    static int OptimizeSim3(kfptr pKF1, kfptr pKF2, std::vector<mpptr> &vpMatches1,
                            g2o::Sim3 &g2oS12, const float th2,bool bFixScale);

    //does not acquire map mutex - done by merging method
    void static OptimizeEssentialGraphLoopClosure(mapptr pMap, kfptr pLoopKF, kfptr pCurKF,
                                       const KeyFrameAndPose &NonCorrectedSim3,
                                       const KeyFrameAndPose &CorrectedSim3,
                                       const map<kfptr, set<kfptr> > &LoopConnections,
                                       const bool &bFixScale);

    void static OptimizeEssentialGraphMapFusion(mapptr pMap, kfptr pLoopKF, kfptr pCurKF,
                                       const map<kfptr, set<kfptr> > &LoopConnections,
                                       const bool &bFixScale);

    size_t static GetID(idpair Idp, bool bIsKf)
    {
        if(bIsKf) return IDRANGE * Idp.second + Idp.first;
        else return IDRANGE * ( MAXAGENTS + Idp.second) + Idp.first;
    }

    idpair static GetPair(size_t Id)
    {
        size_t cid, id;
        cid = Id/IDRANGE;
        id = Id - cid * IDRANGE;
        if(cid >= MAXAGENTS) cid -= MAXAGENTS;
        return make_pair(id,cid);
    }
};

} //end namespace

#endif
