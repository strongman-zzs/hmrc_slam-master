#ifndef CSLAM_DATATYPES_H_
#define CSLAM_DATATYPES_H_

//C++
#include <boost/shared_ptr.hpp>
#include <mutex>
#include <opencv2/opencv.hpp>

//ROS
#include <ros/ros.h>

//CSLAM
#include <hmrc_slam/config.h>
#include <hmrc_slam/estd.h>

//Thirdparty
#include "thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

using namespace std;
//using namespace estd;

namespace hmrc_slam{

enum eSystemState{
    NOTYPE=-1,
    CLIENT=0,
    SERVER=1
};

} //end namespace

#endif
