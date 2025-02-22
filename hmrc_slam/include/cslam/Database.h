
#ifndef CSLAM_DATABASE_H_
#define CSLAM_DATABASE_H_

//C++
#include <boost/shared_ptr.hpp>
#include <vector>
#include <list>
#include <set>
#include <mutex>

//CSLAM
#include <hmrc_slam/config.h>
#include <hmrc_slam/estd.h>
#include <hmrc_slam/Datatypes.h>
#include <hmrc_slam/ORBVocabulary.h>
#include <hmrc_slam/KeyFrame.h>
#include <hmrc_slam/Map.h>
#include <hmrc_slam/Frame.h>

#include <hmrc_slam/MapPoint.h>

//Thirdparty
#include <thirdparty/DBoW2/DBoW2/BowVector.h>

using namespace std;
using namespace estd;

namespace hmrc_slam {

//forward decs
class Map;
class Frame; //new
//------------

class KeyFrameDatabase
{
public:
    typedef boost::shared_ptr<KeyFrameDatabase> dbptr;
    typedef boost::shared_ptr<KeyFrame> kfptr;
    typedef boost::shared_ptr<Map> mapptr;
public:

   KeyFrameDatabase(const vocptr pVoc);

    void add(kfptr pKF);

    void erase(kfptr pKF);

    void clear();

   // Loop Detection
    vector<kfptr> DetectLoopCandidates(kfptr pKF, float minScore);
    vector<kfptr> DetectMapMatchCandidates(kfptr pKF, float minScore, mapptr pMap);

   // Relocalization
   std::vector<kfptr> DetectRelocalizationCandidates(Frame& F);

   //Debug
   typedef boost::shared_ptr<MapPoint> mpptr;
   map<idpair,mpptr> mmpMPs;
   map<idpair,bool> mmbDirectBad;
   void AddMP(mpptr pMP);
   void AddDirectBad(size_t id, size_t cid);
   bool FindMP(size_t id, size_t cid);
   bool FindDirectBad(size_t id, size_t cid);
   mutex mMutexMPs;
   void ResetMPs();

protected:

  // Associated vocabulary
  const vocptr mpVoc;

  // Inverted file
  std::vector<list<kfptr> > mvInvertedFile;

  // Mutex
  std::mutex mMutex;
};

} //end namespace

#endif
