
#ifndef __D_T_F_ORB__
#define __D_T_F_ORB__

#include <opencv2/core/core.hpp>
#include <vector>
#include <string>

#include "FClass.h"

namespace DBoW2 {

/// Functions to manipulate ORB descriptors
class FORB: protected FClass
{
public:

  /// Descriptor type
  typedef cv::Mat TDescriptor; // CV_8U
  /// Pointer to a single descriptor
  typedef const TDescriptor *pDescriptor;
  /// Descriptor length (in bytes)
  static const int L;

    static void meanValue(const std::vector<pDescriptor> &descriptors,
    TDescriptor &mean);

    static int distance(const TDescriptor &a, const TDescriptor &b);

    static std::string toString(const TDescriptor &a);

    static void fromString(TDescriptor &a, const std::string &s);

    static void toMat32F(const std::vector<TDescriptor> &descriptors,
    cv::Mat &mat);

  static void toMat8U(const std::vector<TDescriptor> &descriptors,
    cv::Mat &mat);

};

} // namespace DBoW2

#endif

