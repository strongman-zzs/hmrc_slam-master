

#ifndef CSLAM_TRACKING_H_
#define CSLAM_TRACKING_H_

//C++
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <mutex>
#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>

//CSLAM
#include <hmrc_slam/config.h>
#include <hmrc_slam/estd.h>
#include <hmrc_slam/Converter.h>
#include <hmrc_slam/ORBextractor.h>
#include <hmrc_slam/ORBVocabulary.h>
#include <hmrc_slam/ORBmatcher.h>
#include <hmrc_slam/Frame.h>
#include <hmrc_slam/MapPoint.h>
#include <hmrc_slam/Map.h>
#include <hmrc_slam/KeyFrame.h>
#include <hmrc_slam/Mapping.h>
#include <hmrc_slam/PnPSolver.h>
#include <hmrc_slam/Optimizer.h>
#include <hmrc_slam/Viewer.h>
#include <hmrc_slam/Initializer.h>
#include <hmrc_slam/CentralControl.h>
#include <hmrc_slam/Viewer.h>
#include <hmrc_slam/Communicator.h>

using namespace std;
using namespace estd;

namespace hmrc_slam{

//forward decs
class Viewer;
class Initializer;
class Frame;
class LocalMapping;
class KeyFrameDatabase;
class CentralControl;
class KeyFrame;
class ORBmatcher;
//------------

class Tracking : public boost::enable_shared_from_this<Tracking>
{
public:
    typedef boost::shared_ptr<Tracking> trackptr;
    typedef boost::shared_ptr<Viewer> viewptr;
    typedef boost::shared_ptr<CentralControl> ccptr;
    typedef boost::shared_ptr<Map> mapptr;
    typedef boost::shared_ptr<KeyFrameDatabase> dbptr;
    typedef boost::shared_ptr<Communicator> commptr;
    typedef boost::shared_ptr<LocalMapping> mappingptr;
    typedef boost::shared_ptr<KeyFrame> kfptr;
    typedef boost::shared_ptr<MapPoint> mpptr;
    typedef boost::shared_ptr<Initializer> initptr;
    typedef boost::shared_ptr<Frame> frameptr;

public:
    Tracking(ccptr pCC, vocptr pVoc, viewptr pFrameViewer, mapptr pMap,
             dbptr pKFDB, const string &strCamPath, size_t ClientId);

    // Preprocess the input and call Track(). Extract features and performs stereo matching.
    cv::Mat GrabImageStereo(cv::Mat &imLeft, cv::Mat &imRight);

    // Pointer Setters
    void SetLocalMapper(mappingptr pLocalMapper) {mpLocalMapper = pLocalMapper;}
    void SetCommunicator(commptr pComm) {mpComm = pComm;}

    // Tracking states
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        OK=2,
        LOST=3
    };

    eTrackingState mState;
    eTrackingState mLastProcessedState;

    // Current Frame
    frameptr mCurrentFrame;
    cv::Mat mImGray;

    kfptr GetReferenceKF();
    std::vector<mpptr> GetLocalMPs(){return mvpLocalMapPoints;}

    // Initialization Variables (Monocular)
    std::vector<int> mvIniLastMatches;
    std::vector<int> mvIniMatches;
    std::vector<cv::Point2f> mvbPrevMatched;
    std::vector<cv::Point3f> mvIniP3D;
    frameptr mInitialFrame;

    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    list<cv::Mat> mlRelativeFramePoses;
    list<kfptr> mlpReferences;
    list<double> mlFrameTimes;
    list<bool> mlbLost;

    void Reset();

protected:

    // Main tracking function. It is independent of the input sensor.
    void Track();

    // Map initialization for monocular
    void MonocularInitialization();
    void CreateInitialMapMonocular();

    void CheckReplacedInLastFrame();
    bool TrackReferenceKeyFrame();
    void UpdateLastFrame();
    bool TrackWithMotionModel();

    void UpdateLocalMap();
    void UpdateLocalPoints();
    void UpdateLocalKeyFrames();

    bool TrackLocalMap();
    void SearchLocalPoints();

    bool NeedNewKeyFrame();
    void CreateNewKeyFrame();

    //infrastructure
    size_t mClientId;
    vocptr mpORBVocabulary;
    ccptr mpCC;
    viewptr mpViewer;
    mapptr mpMap;
    commptr mpComm;
    dbptr mpKeyFrameDB;

    //Other Thread Pointers
    mappingptr mpLocalMapper;

    //ORB
    extractorptr mpORBextractor;
    extractorptr mpIniORBextractor;

    // Initalization (only for monocular)
    initptr mpInitializer;

    //Local Map
    kfptr mpReferenceKF;
    std::vector<kfptr> mvpLocalKeyFrames;
    std::vector<mpptr> mvpLocalMapPoints;

    //Calibration matrix
    cv::Mat mK;
    cv::Mat mDistCoef;

    //Current matches in frame
    int mnMatchesInliers;

    //Last Frame, KeyFrame and Relocalisation Info
    kfptr mpLastKeyFrame;
    frameptr mLastFrame;
    idpair mLastKeyFrameId;
    idpair mLastRelocFrameId;

    //Motion Model
    cv::Mat mVelocity;

    //Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB;
};

} //end namespace

#endif
