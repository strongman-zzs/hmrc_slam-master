
#ifndef __D_T_FCLASS__
#define __D_T_FCLASS__

#include <opencv2/core/core.hpp>
#include <vector>
#include <string>

namespace DBoW2 {

/// Generic class to encapsulate functions to manage descriptors.
class FClass
{
  class TDescriptor;
  typedef const TDescriptor *pDescriptor;
  
    virtual void meanValue(const std::vector<pDescriptor> &descriptors, 
    TDescriptor &mean) = 0;
  
    static double distance(const TDescriptor &a, const TDescriptor &b);
  
    static std::string toString(const TDescriptor &a);
  
    static void fromString(TDescriptor &a, const std::string &s);

    static void toMat32F(const std::vector<TDescriptor> &descriptors, 
    cv::Mat &mat);
};

} // namespace DBoW2

#endif
