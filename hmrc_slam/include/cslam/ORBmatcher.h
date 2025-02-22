
#ifndef CSLAM_ORBMATCHER_H_
#define CSLAM_ORBMATCHER_H_

//C++
#include <boost/shared_ptr.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <stdint-gcc.h>
#include <limits.h>

//CSLAM
#include <hmrc_slam/config.h>
#include <hmrc_slam/Frame.h>
#include <hmrc_slam/KeyFrame.h>
#include <hmrc_slam/MapPoint.h>

//Thirdparty
#include <thirdparty/DBoW2/DBoW2/FeatureVector.h>

using namespace std;
using namespace estd;

namespace hmrc_slam{

//forward decs
class KeyFrame;
class MapPoint;
class Frame;
//-------------------------

class ORBmatcher
{
public:
    typedef boost::shared_ptr<Frame> frameptr;
    typedef boost::shared_ptr<KeyFrame> kfptr;
    typedef boost::shared_ptr<MapPoint> mpptr;
public:

    ORBmatcher(float nnratio=0.6, bool checkOri=true);

    // Computes the Hamming distance between two ORB descriptors
    static int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);

    // Search matches between Frame keypoints and projected MapPoints. Returns number of matches
    // Used to track the local map (Tracking)
    int SearchByProjection(Frame &F, const std::vector<mpptr> &vpMapPoints, const float th=3);

    // Project MapPoints tracked in last frame into the current frame and search matches.
    // Used to track from previous frame (Tracking)
    int SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float th);

    // Project MapPoints seen in KeyFrame into the Frame and search matches.
    // Used in relocalisation (Tracking)
    int SearchByProjection(Frame &CurrentFrame, kfptr pKF, const std::set<mpptr> &sAlreadyFound, const float th, const int ORBdist);

    // Project MapPoints using a Similarity Transformation and search matches.
    // Used in loop detection (Loop Closing)
     int SearchByProjection(kfptr pKF, cv::Mat Scw, const std::vector<mpptr> &vpPoints, std::vector<mpptr> &vpMatched, int th);

    // Search matches between MapPoints in a KeyFrame and ORB in a Frame.
    // Brute force constrained to ORB that belong to the same vocabulary node (at a certain level)
    // Used in Relocalisation and Loop Detection
    int SearchByBoW(kfptr pKF, Frame &F, std::vector<mpptr> &vpMapPointMatches);
    int SearchByBoW(kfptr pKF1, kfptr pKF2, std::vector<mpptr> &vpMatches12);

    // Matching for the Map Initialization (only used in the monocular case)
    int SearchForInitialization(Frame &F1, Frame &F2, std::vector<cv::Point2f> &vbPrevMatched, std::vector<int> &vnMatches12, int windowSize=10);

    // Matching to triangulate new MapPoints. Check Epipolar Constraint.
    int SearchForTriangulation(kfptr pKF1, kfptr pKF2, cv::Mat F12,
                               std::vector<pair<size_t, size_t> > &vMatchedPairs);

    // Search matches between MapPoints seen in KF1 and KF2 transforming by a Sim3 [s12*R12|t12]
    // In the stereo and RGB-D case, s12=1
    int SearchBySim3(kfptr pKF1, kfptr pKF2, std::vector<mpptr> &vpMatches12, const float &s12, const cv::Mat &R12, const cv::Mat &t12, const float th);

    // Project MapPoints into KeyFrame and search for duplicated MapPoints.
    int Fuse(kfptr pKF, const vector<mpptr> &vpMapPoints, const float th=3.0);

    // Project MapPoints into KeyFrame using a given Sim3 and search for duplicated MapPoints.
    int Fuse(kfptr pKF, cv::Mat Scw, const std::vector<mpptr> &vpPoints, float th, vector<mpptr> &vpReplacePoint);

public:

    static const int TH_LOW;
    static const int TH_HIGH;
    static const int HISTO_LENGTH;


protected:

    bool CheckDistEpipolarLine(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &F12, const kfptr pKF);

    float RadiusByViewingCos(const float &viewCos);

    void ComputeThreeMaxima(std::vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3);

    float mfNNratio;
    bool mbCheckOrientation;
};

} //end namespace

#endif
