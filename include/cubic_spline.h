//
// Created by fz on 2021/7/14.
//

#ifndef TRAJECTORY_MODULE_CUBIC_SPLINE_H
#define TRAJECTORY_MODULE_CUBIC_SPLINE_H

#include <ros/ros.h>
#include "Eigen/Dense"
#include <vector>
#include <iostream>

namespace autobots {
    namespace lightning {
        namespace planning {

            class CubicSpline {
            public:
                CubicSpline(std::vector<float> x_array, std::vector<float> diff_x, std::vector<float> y_array, std::vector<float> diff_y);

                ~CubicSpline();

                int get_idx(float x);

                float get_y(int idx);

                float calculate(float x);

                float calculate_derivative(float x);

                float calculate_second_derivative(float x);

            private:
                int waypoints_num_;
                std::vector<float> a_matrix_;
                std::vector<float> b_matrix_;
                std::vector<float> c_matrix_;
                std::vector<float> d_matrix_;

                std::vector<float> x_array_;
                std::vector<float> y_array_;
                std::vector<float> diff_x_;
                std::vector<float> diff_y_;
            };

        } // namespace planning
    } // namespace lightning
} // namespace autobots

#endif //TRAJECTORY_MODULE_CUBIC_SPLINE_H
