//
// Created by fz on 2021/7/14.
//

#include "cubic_spline_fitter.h"

namespace autobots {
    namespace lightning {
        namespace planning {

            CubicSplineFitter::CubicSplineFitter(autobots_msgs::AutobotsTrajectory* global_path){
                global_path_ = global_path;

                int waypoints_num = global_path_->waypoints.size();
                station_array_.resize(waypoints_num);
                x_array_.resize(waypoints_num);
                y_array_.resize(waypoints_num);
                station_array_[0] = 0;
                diff_station_.resize(waypoints_num - 1);
                diff_x_.resize(waypoints_num - 1);
                diff_y_.resize(waypoints_num - 1);

                for (int i = 0; i < waypoints_num; ++i) {
                    x_array_[i] = global_path_->waypoints.at(i).point.x;
                    y_array_[i] = global_path_->waypoints.at(i).point.y;
                    if (i >= 1) {
                        float dx = x_array_[i] - x_array_[i - 1];
                        float dy = y_array_[i] - y_array_[i - 1];
                        diff_x_[i - 1] = dx;
                        diff_y_[i - 1] = dy;
                        diff_station_[i - 1] = sqrt(pow(dx, 2) + pow(dy, 2));
                        station_array_[i] = station_array_[i - 1] + diff_station_[i - 1];
                    }
                }
                SX_spline_ = new CubicSpline(station_array_, diff_station_, x_array_, diff_x_);
                SY_spline_ = new CubicSpline(station_array_, diff_station_, y_array_, diff_y_);

                for(int i = 0; i < waypoints_num; i++) {
                    global_path_->waypoints.at(i).station = station_array_[i];
                    global_path_->waypoints.at(i).lateral = 0.0;
                }
            }

            CubicSplineFitter::~CubicSplineFitter() {}

            autobots_msgs::AutobotsWaypoint* CubicSplineFitter::calculate(const autobots_msgs::AutobotsStatus* status){
                autobots_msgs::AutobotsWaypoint* waypoint = new autobots_msgs::AutobotsWaypoint();
                waypoint->point = status->pose.position;
                geometry_msgs::Vector3 vel_vec3 = status->linear_velocity;
                waypoint->speed = sqrt(pow(vel_vec3.x, 2) + pow(vel_vec3.y, 2));
                geometry_msgs::Vector3 acc_vec3 = status->linear_acceleration;
                waypoint->accel = sqrt(pow(acc_vec3.x, 2) + pow(acc_vec3.y, 2));
                waypoint->theta = get_euler_zyx(status->pose.orientation).z;

                float min_dist = 300.0;
                int closest_idx = 0;
                float curr_x = status->pose.position.x;
                float curr_y = status->pose.position.y;
                // find the closest point idx
                for (int i = global_path_->waypoints.size() - 1; i >= 0; --i) {
                    float wp_x = global_path_->waypoints.at(i).point.x;
                    float wp_y = global_path_->waypoints.at(i).point.y;
                    float dis = sqrt(pow(curr_x - wp_x, 2) + pow(curr_y - wp_y, 2));
                    if (dis < min_dist) {
                        closest_idx = i;
                        min_dist = dis;
                    }
                }

                // closest_idx should be on the behind of current point
                int nxt_idx = closest_idx + 1;
                float closest_wp_x = global_path_->waypoints.at(closest_idx).point.x;
                float closest_wp_y = global_path_->waypoints.at(closest_idx).point.y;
                float nxt_wp_x = global_path_->waypoints.at(nxt_idx).point.x;
                float nxt_wp_y = global_path_->waypoints.at(nxt_idx).point.y;
                float dot = (curr_x - closest_wp_x) * (nxt_wp_x - closest_wp_x) + (curr_y - closest_wp_y) * (nxt_wp_y - closest_wp_y);
                if (dot < 0 && closest_idx > 0) {
                    closest_idx--;
                    nxt_idx--;
                }
                float closest_station = station_array_[closest_idx];
                float vec1_x = global_path_->waypoints[nxt_idx].point.x - global_path_->waypoints[closest_idx].point.x;
                float vec1_y = global_path_->waypoints[nxt_idx].point.y - global_path_->waypoints[closest_idx].point.y;
                float vec2_x = curr_x - global_path_->waypoints[closest_idx].point.x;
                float vec2_y = curr_y - global_path_->waypoints[closest_idx].point.y;
                float distance = sqrt(pow(vec1_x, 2) + pow(vec1_y, 2));
                float delta_s = (vec1_x * vec2_x + vec1_y * vec2_y) / distance;
                float s_condition = closest_station + delta_s;

//                std::vector<float> delta_vec = calculate_fit_delta(curr_x, curr_y, s_condition);
//                float delta_angle = delta_vec[0];
//                float delta_x = delta_vec[1];
//                float delta_y = delta_vec[2];
//                s_condition += sin(delta_angle) * (sqrt(pow(delta_x, 2) + pow(delta_y, 2)));
                waypoint->station = s_condition;
                std::vector<float> delta_vec = calculate_fit_delta(curr_x, curr_y, s_condition);
                float delta_angle = delta_vec[0];
                float delta_x = delta_vec[1];
                float delta_y = delta_vec[2];
                float d_condition = cos(delta_angle) * (sqrt(pow(delta_x, 2) + pow(delta_y, 2)));
                waypoint->lateral = d_condition;
                return waypoint;
            }


            std::vector<float> CubicSplineFitter::calculate_fit_delta(float curr_x, float curr_y, float station) {
                float r_theta = calculate_fit_theta(station);
                float dis_x = curr_x - SX_spline_->calculate(station);
                float dis_y = curr_y - SY_spline_->calculate(station);
                float norm_dis_x = dis_x / sqrt(pow(dis_x, 2) + pow(dis_y, 2));
                float norm_dis_y = dis_y / sqrt(pow(dis_x, 2) + pow(dis_y, 2));

                float r_theta_sin = -sin(r_theta);
                float r_theta_cos = cos(r_theta);
                float norm_r_theta_sin = r_theta_sin / sqrt(pow(r_theta_sin, 2) + pow(r_theta_cos, 2));
                float norm_r_theta_cos = r_theta_cos / sqrt(pow(r_theta_sin, 2) + pow(r_theta_cos, 2));
                float arccos = norm_dis_x * norm_r_theta_sin + norm_dis_y * norm_r_theta_cos;
                if (arccos > 1) {
                    arccos = 1;
                } else if (arccos < -1) {
                    arccos = -1;
                }
                float angle = acos(arccos);
                return std::vector<float>{angle, dis_x, dis_y};
            }

            float CubicSplineFitter::calculate_fit_theta(float station) {
                float dx = SX_spline_->calculate_derivative(station);
                float dy = SY_spline_->calculate_derivative(station);
                float theta = atan2(dy, dx);
                return theta;
            }

            geometry_msgs::Vector3 CubicSplineFitter::get_euler_zyx(const geometry_msgs::Quaternion& quat) {
                float x = quat.x;
                float y = quat.y;
                float z = quat.z;
                float w = quat.w;

                geometry_msgs::Vector3 vect3;
                vect3.x = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
                vect3.y = asin(2 * (w * y - z * x));
                vect3.z = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));

                float absr1 = fabs(vect3.x) + fabs(vect3.y) + fabs(vect3.z);

                //see if any less rotation
                if (absr1 >= M_PI * 2) {
                    geometry_msgs::Vector3 vect4;
                    vect4.x = vect3.x + M_PI;
                    vect4.y = vect3.y + M_PI;
                    vect4.z = vect3.z + M_PI;
                    if (vect4.x >= M_PI) {
                        vect4.x -= M_PI * 2;
                    }
                    if (vect4.y >= M_PI) {
                        vect4.y -= M_PI * 2;
                    }
                    if (vect4.z >= M_PI) {
                        vect4.z -= M_PI * 2;
                    }
                    float absr2 = fabs(vect4.x) + fabs(vect4.y) + fabs(vect4.z);
                    if (absr2 < absr1) {
                        vect3 = vect4;
                    }
                }
                return vect3;
            }


        } // namespace map
    } // namespace lightning
} // namespace autobots
