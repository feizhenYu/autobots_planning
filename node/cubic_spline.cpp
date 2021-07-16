//
// Created by fz on 2021/7/14.
//

#include "cubic_spline.h"

using namespace Eigen;

namespace autobots {
    namespace lightning {
        namespace planning {

            CubicSpline::CubicSpline(std::vector<float> x_array, std::vector<float> diff_x, std::vector<float> y_array,
                                     std::vector<float> diff_y) : x_array_(x_array), diff_x_(diff_x),
                                                                   y_array_(y_array), diff_y_(diff_y) {
                std::cout << "CubicSpline Constructor" << std::endl;
                waypoints_num_ = x_array.size();

                Eigen::MatrixXf H = Eigen::MatrixXf::Zero(waypoints_num_, waypoints_num_);
                Eigen::VectorXf Y = Eigen::VectorXf::Zero(waypoints_num_);
                Eigen::VectorXf M = Eigen::VectorXf::Zero(waypoints_num_);

                H(0, 0) = 1;
                H(waypoints_num_ - 1, waypoints_num_ - 1) = 1;
                for (int i = 1; i < waypoints_num_ - 1; i++) {
                    H(i, i - 1) = diff_x_[i - 1];
                    H(i, i) = 2 * (diff_x_[i - 1] + diff_x_[i]);
                    H(i, i + 1) = diff_x_[i];
                    Y(i) = 3 * (diff_y_[i] / diff_x_[i] - diff_y_[i - 1] / diff_x_[i - 1]);
                }

                M = H.colPivHouseholderQr().solve(Y);

                for (int i = 0; i < waypoints_num_ - 1; i++) {
                    a_matrix_.push_back(y_array_[i]);
                    b_matrix_.push_back((M(i + 1) - M(i)) / (3 * diff_x_[i]));
                    c_matrix_.push_back(diff_y_[i] / diff_x_[i] - diff_x_[i] * (2 * M(i) + M(i + 1)) / 3);
                    d_matrix_.push_back(M(i));
                }
            }

            CubicSpline::~CubicSpline() {}

            int CubicSpline::get_idx(float x) {
                int idx = 0;
                if (x < 0.0) {
                    idx = 0;
                    return idx;
                }
                for (int i = 0; i < waypoints_num_ - 1; i++) {
                    if (x >= x_array_[waypoints_num_ - 1]) {
                        idx = waypoints_num_ - 1;
                        break;
                    } else if (x >= x_array_[i] && x < x_array_[i + 1]) {
                        idx = i;
                        break;
                    }
                }
                return idx;
            }

            float CubicSpline::get_y(int idx) {
                return y_array_[idx];
            }

            float CubicSpline::calculate(float x) {
                int idx = get_idx(x);
                float dx = x - x_array_[idx];
                float res = a_matrix_[idx] + b_matrix_[idx] * dx + c_matrix_[idx] * pow(dx, 2) +
                             d_matrix_[idx] * pow(dx, 3);
                return res;
            }

            float CubicSpline::calculate_derivative(float x) {
                int idx = get_idx(x);
                float dx = x - x_array_[idx];
                float res = b_matrix_[idx] + 2 * c_matrix_[idx] * dx + 3 * d_matrix_[idx] * pow(dx, 2);
                return res;
            }

            float CubicSpline::calculate_second_derivative(float x) {
                int idx = get_idx(x);
                float dx = x - x_array_[idx];
                float res = 2 * c_matrix_[idx] + 6 * d_matrix_[idx] * dx;
                return res;
            }
        } // namespace planning
    } // namespace lightning
} // namespace autobots
