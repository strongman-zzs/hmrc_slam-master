

#include <hmrc_slam/server/ServerSystem.h>

int main(int argc, char **argv) {

    ros::init(argc, argv, "CSLAM server node");

    if(argc != 2)
    {
        cerr << endl << "Usage: rosrun hmrc_slam clientnode path_to_vocabulary" << endl;
        ros::shutdown();
        return 1;
    }

    ros::NodeHandle Nh;
    ros::NodeHandle NhPrivate("~");

    boost::shared_ptr<hmrc_slam::ServerSystem> pSSys{new hmrc_slam::ServerSystem(Nh,NhPrivate,argv[1])};
    pSSys->InitializeClients();


    ROS_INFO("started CSLAM server node...");

    ros::MultiThreadedSpinner MSpin(2);

        MSpin.spin();

    ros::waitForShutdown();

    return 0;
}
