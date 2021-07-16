//
// Created by fz on 2021/7/14.
//

#ifndef TRAJECTORY_MODULE_CUBIC_SPLINE_FITTER_H
#define TRAJECTORY_MODULE_CUBIC_SPLINE_FITTER_H

#include <geometry_msgs/Point.h>
#include <string>
#include <fstream>
#include <boost/filesystem.hpp>
#include "cubic_spline.h"
#include "autobots_msgs/AutobotsStatus.h"
#include "autobots_msgs/AutobotsWaypoint.h"
#include "autobots_msgs/AutobotsTrajectory.h"
#include "autobots_msgs/AutobotsTrajArray.h"

namespace autobots {
    namespace lightning {
        namespace planning {

            /// @brief the class
            class CubicSplineFitter{
            public:
                /// @brief construct function
                explicit CubicSplineFitter(autobots_msgs::AutobotsTrajectory* global_path_);

                /// @brief destruct function
                ~CubicSplineFitter();

                /// @brief map function
                autobots_msgs::AutobotsWaypoint* calculate(const autobots_msgs::AutobotsStatus* status);

                geometry_msgs::Vector3 get_euler_zyx(const geometry_msgs::Quaternion& quat);

                std::vector<float> calculate_fit_delta(float curr_x, float curr_y, float station);

                float calculate_fit_theta(float station);

                CubicSpline* SX_spline_;
                CubicSpline* SY_spline_;


            private:

                autobots_msgs::AutobotsTrajectory* global_path_ = new autobots_msgs::AutobotsTrajectory(); ///< global path

                std::vector<int> closest_idx_array_;
                std::vector<float> theta_array_;
                std::vector<float> station_array_;
                std::vector<float> x_array_;
                std::vector<float> y_array_;
                std::vector<float> diff_station_;
                std::vector<float> diff_x_;
                std::vector<float> diff_y_;
            };

        } // namespace planning
    } // namespace lightning
} // namespace autobots


#endif //TRAJECTORY_MODULE_CUBIC_SPLINE_FITTER_H
