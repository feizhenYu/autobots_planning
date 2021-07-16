//
// Created by fz on 2021/7/13.
//

#ifndef TRAJECTORY_MODULE_TRAJECTORY_GEN_CORE_H
#define TRAJECTORY_MODULE_TRAJECTORY_GEN_CORE_H

#include <ros/ros.h>
#include <cmath>
#include <std_msgs/String.h>
#include <std_msgs/Int64.h>
#include <std_srvs/Empty.h>
#include <iostream>
#include <sys/time.h>
#include <visualization_msgs/Marker.h>
#include "autobots_msgs/AutobotsStatus.h"
#include "autobots_msgs/AutobotsWaypoint.h"
#include "autobots_msgs/AutobotsTrajectory.h"
#include "autobots_msgs/AutobotsTrajArray.h"

#include "autoware_msgs/Lane.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include <string>
#include <fstream>
#include <boost/filesystem.hpp>

#include "cubic_spline_fitter.h"

namespace autobots {
    namespace lightning {
        namespace planning{
            class TrajectoryCore {
            public:

                static TrajectoryCore& Instance() {
                    static TrajectoryCore instance;
                    return instance;
                }

                /// @brief the global path callback function
                void global_path_callback(const visualization_msgs::MarkerArray& marker_array);

                void status_callback(const autobots_msgs::AutobotsStatus& status);

                void behavior_callback(const std_msgs::Int64 behavior_flag);

                void autoware_traj_callback(const autoware_msgs::Lane& traj);

                void generate_local_trajectory(float goal_vel_s, int total_num , float delta_time);

                void generate_quintic_params(float init_d, float init_v, float init_a, float goal_d, float t);

                void generate_quartic_params(float init_s, float init_v, float init_a, float goal_v, float t);

                float calc_quintic_val(float t);

                float calc_quintic_first_derivative(float t);

                float calc_quintic_second_derivative(float t);

                float calc_quintic_third_derivative(float t);

                float calc_quartic_val(float t);

                float calc_quartic_first_derivative(float t);

                float calc_quartic_second_derivative(float t);

                float calc_quartic_third_derivative(float t);

                std::vector<float> meanfilter(std::vector<float> data, int meanwindow);

                void calculate_kappa(std::vector<autobots_msgs::AutobotsWaypoint> waypoints);

                float unify_angle(float angle);

                float distance(const geometry_msgs::Point& pt1, const geometry_msgs::Point& pt2);

                void record_traj(autobots_msgs::AutobotsTrajectory* traj);

                /// @brief the trajectory puh timer function
                void trajectory_pub_timer(const ros::TimerEvent& event);

            private:

                TrajectoryCore();
                ~TrajectoryCore();

                ros::NodeHandle nh_; ///< ros handle
                ros::Subscriber global_path_sub_; ///< global path Subscriber
                ros::Subscriber autoware_traj_sub_; ///< autoware local trajectory Subscriber
                ros::Subscriber status_sub_; ///< status Subscriber
                ros::Publisher trajectory_pub_; ///< trajectory publisher
                ros::Publisher trajectory_visual_pub_; ///< trajectory publisher
                ros::Subscriber behavior_sub_; ///< behavior Subscriber
                ros::Timer trajectory_timer_; ///< the timer to publish

                autobots_msgs::AutobotsTrajectory* global_path_ = new autobots_msgs::AutobotsTrajectory(); ///< the global path
                autobots_msgs::AutobotsTrajectory* local_trajectory_ = new autobots_msgs::AutobotsTrajectory(); ///< the local trajectory
                autobots_msgs::AutobotsStatus* status_ = new autobots_msgs::AutobotsStatus(); ///< the car status
                autobots_msgs::AutobotsWaypoint* current_waypoint_ = new autobots_msgs::AutobotsWaypoint();
                CubicSplineFitter* spline_fitter_; ///< the spline fitter

                std::vector<float> quintic_params_;
                std::vector<float> quartic_params_;
                std::vector<autobots_msgs::AutobotsWaypoint> paths_;

                std::vector<float> theta_array_;

                int local_traj_counter_ = 0; ///< local trajectory counter
                int log_file_counter_ = 0; ///< local trajectory counter
                float dt_;
                bool is_global_path_fitted_;
                int g_path_min_counter_;
                int behavior_;
                int lane_id_;
                float road_width_;

                FILE* log_file_;
            };
        }
    }
}



#endif //TRAJECTORY_MODULE_TRAJECTORY_GEN_CORE_H
