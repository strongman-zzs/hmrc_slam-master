
#ifndef CSLAM_MAP_H_
#define CSLAM_MAP_H_

//C++
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <set>
#include <mutex>
#include <sstream>
#include <queue>
#include <ctime>

//CSLAM
#include <hmrc_slam/config.h>
#include <hmrc_slam/estd.h>
#include <hmrc_slam/Datatypes.h>
#include <hmrc_slam/CentralControl.h>
#include <hmrc_slam/MapPoint.h>
#include <hmrc_slam/KeyFrame.h>
#include <hmrc_slam/Communicator.h>
#include <hmrc_slam/ORBVocabulary.h>

#ifdef DENSEMAP2
#include <dense_mapping_backend/Interfaces.h>
#endif

//Msgs
#include <hmrcslam_msgs/Map.h>

using namespace std;
using namespace estd;

namespace hmrc_slam{

//forward decs
class CentralControl;
class Communicator;
class KeyFrame;
class MapPoint;
class KeyFrameDatabase;
//------------

class Map : public boost::enable_shared_from_this<Map>
{
public:
    typedef boost::shared_ptr<Map> mapptr;
    typedef boost::shared_ptr<KeyFrame> kfptr;
    typedef boost::shared_ptr<MapPoint> mpptr;
    typedef boost::shared_ptr<CentralControl> ccptr;
    typedef boost::shared_ptr<Communicator> commptr;
    typedef boost::shared_ptr<KeyFrameDatabase> dbptr;

    struct kftimecmp{
        bool operator() (const kfptr pA, const kfptr pB) const;
    };

    struct kftimecmpsmaller{
        bool operator() (const kfptr pA, const kfptr pB) const;
    };

public:
    //---Constructor---
    Map(ros::NodeHandle Nh, ros::NodeHandle NhPrivate, size_t MapId, eSystemState SysState);
    Map(const mapptr &pMapA, const mapptr &pMapB); //merge constructor
    void UpdateAssociatedData(); //need to be called after merge constructor. Cannot be called by merge constructor because of usage of shared_from_this()
    Map& operator=(Map& rhs);

    //---infrastructure---
    set<size_t> msuAssClients;
    void AddCCPtr(ccptr pCC);
    set<ccptr> GetCCPtrs();
    ccptr GetCCPtr(size_t nClientId);
    void SetOutdated(){unique_lock<mutex> lock(mMutexOutdated); mbOutdated=true;}
    bool GetOutdated(){unique_lock<mutex> lock(mMutexOutdated); return mbOutdated;}
    string mOdomFrame;
    size_t mMapId;
    eSystemState mSysState;

    //---Add/Erase data---
    void AddKeyFrame(kfptr pKF);
    void AddMapPoint(mpptr pMP);
    void EraseMapPoint(mpptr pMP);
    void EraseKeyFrame(kfptr pKF);
    void SetReferenceMapPoints(const std::vector<mpptr> &vpMPs);
    void ClearBadMPs();

    //---MP/KF parent ptrs
    void HandleMissingParent(size_t QueryId, size_t QueryCId, cv::Mat &T_cref_cquery, kfptr pRefKf);

    //---Setter---
    void SetCommunicator(commptr pComm) {mspComm.insert(pComm);}

    //---Getter---
    kfptr GetKfPtr(size_t KfId, size_t ClientId, bool bIgnoreMutex = false);
    kfptr GetKfPtr(idpair id){return GetKfPtr(id.first,id.second);}
    kfptr GetRandKfPtr();
    mpptr GetMpPtr(size_t MpId, size_t ClientId);
    mpptr GetMpPtr(idpair id){return GetMpPtr(id.first,id.second);}
    kfptr GetErasedKfPtr(size_t KfId, size_t ClientId);
    kfptr GetErasedKfPtr(idpair id){return GetErasedKfPtr(id.first,id.second);}
    mpptr GetErasedMpPtr(size_t MpId, size_t ClientId);
    mpptr GetErasedMpPtr(idpair id){return GetErasedMpPtr(id.first,id.second);}
    bool IsKfDeleted(size_t KfId, size_t ClientId);
    bool IsKfDeleted(idpair id){return IsKfDeleted(id.first,id.second);}
    bool IsMpDeleted(size_t MpId, size_t ClientId);
    bool IsMpDeleted(idpair id){return IsMpDeleted(id.first,id.second);}

    vector<kfptr> GetAllKeyFrames();
    vector<mpptr> GetAllMapPoints();
    std::vector<mpptr> GetReferenceMapPoints();

    long unsigned int MapPointsInMap();
    long unsigned  KeyFramesInMap();

    long unsigned int GetMaxKFid();
    long unsigned int GetMaxMPid();
    long unsigned int GetMaxKFidUnique();
    long unsigned int GetMaxMPidUnique();
    long unsigned int GetLastKfIdUnique();

    kfptr GetPredecessor(kfptr pKF);

    kfptr GetFromKfBuffer(size_t KfId, size_t ClientId);
    bool IsInKfBuffer(size_t KfId, size_t ClientId);
    kfptr SearchAndReturnFromKfBuffer(size_t KfId, size_t ClientId);
    void DeleteFromKfBuffer(kfptr pKF);
    mpptr GetFromMpBuffer(size_t MpId, size_t ClientId);
    bool IsInMpBuffer(size_t MpId, size_t ClientId);
    mpptr SearchAndReturnFromMpBuffer(size_t MpId, size_t ClientId);
    void DeleteFromMpBuffer(mpptr pMP);

    std::vector<mpptr> GetMvpReferenceMapPoints() {return mvpReferenceMapPoints;}
    std::map<idpair,mpptr> GetMmpMapPoints() {return mmpMapPoints;}
    std::map<idpair,kfptr> GetMmpKeyFrames() {return mmpKeyFrames;}
    std::map<idpair,mpptr> GetMmpErasedMapPoints() {return mmpErasedMapPoints;}
    std::map<idpair,kfptr> GetMmpErasedKeyFrames() {return mmpErasedKeyFrames;}

    //---data---
    vector<kfptr> mvpKeyFrameOrigins;

    //---map management---
    void MapTrimming(kfptr pKFcur);
    void FindLocalKFsByTime(kfptr pKFcur,set<kfptr>& sKfVicinity,priority_queue<int>& pqNativeKFs,list<kfptr>& lForeignKFs, int nLocalKFs);
    void PackVicinityToMsg(kfptr pKFcur, hmrcslam_msgs::Map &msgMap, ccptr pCC);
    set<mpptr> mspMPsToErase;

    //---Reset---
    void clear();

    //---mutexes & sync---
    bool LockMapUpdate(){unique_lock<mutex> lock(mMutexMapUpdate); if(!mbLockMapUpdate){mbLockMapUpdate = true; return true;} else return false;}
    bool LockPointCreation(){unique_lock<mutex> lock(mMutexPointCreation); if(!mbLockPointCreation){mbLockPointCreation = true; return true;} else return false;}
    void UnLockMapUpdate(){unique_lock<mutex> lock(mMutexMapUpdate);if(mbLockMapUpdate){mbLockMapUpdate = false;} else{cout << "\033[1;31m!!! ERROR !!!\033[0m \"Map\": Attempt to UnLock MapUpdate -- was not locked" << endl; throw estd::infrastructure_ex();}}
    void UnLockPointCreation(){unique_lock<mutex> lock(mMutexPointCreation);if(mbLockPointCreation){mbLockPointCreation = false;} else{cout << "\033[1;31m!!! ERROR !!!\033[0m \"Map\": Attempt to UnLock PointCreation -- was not locked" << endl; throw estd::infrastructure_ex();}}

    //BA
    bool mbStopGBA;
    std::mutex mMutexGBA;
    std::thread* mpThreadGBA;

    bool isRunningGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbRunningGBA;
    }
    void setRunningGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        mbRunningGBA = true;
    }
    void unsetRunningGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        mbRunningGBA = false;
    }

    bool isFinishedGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbFinishedGBA;
    }
    void setFinishedGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        mbFinishedGBA = true;
    }
    void unsetFinishedGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        mbFinishedGBA = false;
    }

    bool isNoStartGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbNoStartGBA;
    }
    void setNoStartGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        mbNoStartGBA = true;
    }
    void unsetNoStartGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        mbNoStartGBA = false;
    }

    #ifdef DONOTINTERRUPTMERGE
    bool isMergeStepGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbMergeStepGBA;
    }
    void setMergeStepGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        mbMergeStepGBA = true;
    }
    void unsetMergeStepGBA(){
//        unique_lock<std::mutex> lock(mMutexGBA); //already locked when this is used
        mbMergeStepGBA = false;
    }
    #endif

    void StopGBA();

    #ifdef FINALBA
    bool isGBAinterrupted(){
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbGBAinterrupted;
    }
    void setGBAinterrupted(){
        unique_lock<std::mutex> lock(mMutexGBA);
        mbGBAinterrupted = true;
    }
    void unsetGBAinterrupted(){
//        unique_lock<std::mutex> lock(mMutexGBA); //already locked when this is used
        mbGBAinterrupted = false;
    }
    #endif

    void RequestBA(size_t nClientId);
    void RunGBA(idpair nLoopKF);
    set<size_t> msnFinishedAgents;

    void WriteStateToCsv(const std::string& filename,
                         const size_t clientId);

    //---Map Save/Load---
    void LoadMap(const string &path_name, vocptr voc, commptr comm, dbptr kfdb, uidptr uid);
    void SaveMap(const string &path_name);

    #ifdef LOGGING
    bool IsInUse(){return !mbOutdated;}
    #endif

    #ifdef DEBUGGING2
    void CheckStructure();
    #endif

protected:
    //---infrastructure---
    ros::NodeHandle mNh;
    ros::NodeHandle mNhPrivate;

    std::set<ccptr> mspCC;
    set<commptr> mspComm; //comm ptrs of associated clients -- necessary to pass to MPs/KFs

    //---data---
    std::vector<mpptr> mvpReferenceMapPoints;

    long unsigned int mnMaxKFid;
    long unsigned int mnMaxMPid;
    long unsigned int mnMaxKFidUnique;
    long unsigned int mnMaxMPidUnique;
    long unsigned int mnLastKfIdUnique;

    std::map<idpair,mpptr> mmpMapPoints; //Performance: second container is not nice, but fast... //CHECKHERE
    std::map<idpair,kfptr> mmpKeyFrames; //Performance: second container is not nice, but fast... //CHECKHERE
    std::map<idpair,mpptr> mmpErasedMapPoints;
    std::map<idpair,kfptr> mmpErasedKeyFrames;

//    map<idpair,kfptr> mmpKfBuffer;
//    map<idpair,mpptr> mmpMpBuffer;

    bool mbOutdated;

    //---mutexes---
    std::mutex mMutexCC;
    std::mutex mMutexErased;
    std::mutex mMutexMap;
//    mutex mMutexKfBuffer; //should not be necessary, buffers only accessed by comm thread
//    mutex mMutexMpBuffer; //should not be necessary, buffers only accessed by comm thread
    bool mbLockMapUpdate;
    bool mbLockPointCreation;
    std::mutex mMutexMapUpdate;
    std::mutex mMutexPointCreation; // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexOutdated;
    std::mutex mMutexStopGBAInProgess;

    bool mbRunningGBA;
    bool mbFinishedGBA;
    bool mbNoStartGBA;
    bool mbStopGBAInProgress;
    #ifdef DONOTINTERRUPTMERGE
    bool mbMergeStepGBA;
    #endif

    #ifdef FINALBA
    bool mbGBAinterrupted;
    #endif
};

} //end namespace

#endif
