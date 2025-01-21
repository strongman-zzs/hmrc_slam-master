

#include <hmrc_slam/client/ClientSystem.h>

#include <hmrc_slam/config.h>

int main(int argc, char **argv) {

    ros::init(argc, argv, "CSLAM client node");

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun hmrc_slam clientnode path_to_vocabulary path_to_cam_params" << endl;
        ros::shutdown();
        return 1;
    }

    ros::NodeHandle Nh;
    ros::NodeHandle NhPrivate("~");

    boost::shared_ptr<hmrc_slam::ClientSystem> pCSys{new hmrc_slam::ClientSystem(Nh,NhPrivate,argv[1],argv[2])};

    ROS_INFO("Started CSLAM client node...");

    ros::Rate r(params::timings::client::miRosRate);
    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
