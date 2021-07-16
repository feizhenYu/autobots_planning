//
// Created by fz on 2021/7/13.
//

#include "trajectory_gen_core.h"

/// @brief main function of the map module
int main(int argc, char **argv) {
    ros::init(argc, argv, "autobots_planning");
    //ros::console::shutdown();

    autobots::lightning::planning::TrajectoryCore& trajectory_core = autobots::lightning::planning::TrajectoryCore::Instance();

    ros::spin();

    return 0;
}