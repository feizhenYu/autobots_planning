//
// Created by fz on 2021/7/13.
//

#include "trajectory_gen_core.h"

using namespace Eigen;
namespace autobots {
    namespace lightning {
        namespace planning {
            TrajectoryCore::TrajectoryCore():dt_(0.05), g_path_min_counter_(0), is_global_path_fitted_(false), behavior_(0), road_width_(3.0), lane_id_(0){
                quintic_params_.resize(6);
                quartic_params_.resize(5);
                std::vector<autobots_msgs::AutobotsWaypoint>().swap(paths_);
                global_path_sub_ = nh_.subscribe("/global_waypoints_rviz", 1, &TrajectoryCore::global_path_callback, this);
                status_sub_ = nh_.subscribe("/autobots/gateway/status", 1, &TrajectoryCore::status_callback, this);
                behavior_sub_ = nh_.subscribe("/behavior", 1, &TrajectoryCore::behavior_callback, this);
                autoware_traj_sub_ = nh_.subscribe("/final_waypoints", 1, &TrajectoryCore::autoware_traj_callback, this);
                trajectory_pub_ = nh_.advertise<autobots_msgs::AutobotsTrajectory>("/autobots/planning/trajectory", 1);
                trajectory_timer_ = nh_.createTimer(ros::Duration(dt_), &TrajectoryCore::trajectory_pub_timer, this);
            }

            TrajectoryCore::~TrajectoryCore() {
                status_sub_.shutdown();
                behavior_sub_.shutdown();
                autoware_traj_sub_.shutdown();
                global_path_sub_.shutdown();
                trajectory_pub_.shutdown();
            }

            void TrajectoryCore::global_path_callback(const visualization_msgs::MarkerArray& marker_array) {
                ROS_WARN("receive global path!, points: %ld", marker_array.markers[0].points.size() - 1);
                struct timeval begin_tm;
                gettimeofday(&begin_tm, NULL);
                float tem_dis = 0.0;
                int len = 0;

                delete global_path_;
                global_path_ = new autobots_msgs::AutobotsTrajectory();
                autobots_msgs::AutobotsWaypoint temp_global_waypoint;

                // from i = 1, because the first point is (0, 0, 0)
                for (int i = 1; i < marker_array.markers[0].points.size(); i++) {
                    temp_global_waypoint.point = marker_array.markers[i].pose.position;
                    len = global_path_->waypoints.size();
                    if (len > 0) {
                        tem_dis = sqrt(pow((temp_global_waypoint.point.x - global_path_->waypoints.back().point.x), 2) +\
                    pow((temp_global_waypoint.point.y - global_path_->waypoints.back().point.y), 2));
                        if (tem_dis < 0.1) {
                            g_path_min_counter_++;
                            continue;
                        }
                        if (tem_dis > 5.0) {
                            ROS_ERROR("Global path error: two point distance more than 5 meters !!!!");
                            return;
                        }
                    }
                    global_path_->waypoints.push_back(temp_global_waypoint);
                }

                ROS_WARN("Global path processed: min: %d, output points: %ld", g_path_min_counter_, global_path_->waypoints.size());
                if (!is_global_path_fitted_) {
                    spline_fitter_ = new CubicSplineFitter(global_path_);
                    is_global_path_fitted_ = true;
                }

                struct timeval tm;
                gettimeofday(&tm, NULL);
                float fitted_global_path_time = (tm.tv_sec - begin_tm.tv_sec) * 1000 + (tm.tv_usec - begin_tm.tv_usec) * 0.001;
                ROS_WARN("fitted global path! time cost: %.3f ms", fitted_global_path_time);
            }

            void TrajectoryCore::status_callback(const autobots_msgs::AutobotsStatus& status) {
                status_ = new autobots_msgs::AutobotsStatus(status);
            }

            void TrajectoryCore::behavior_callback(const std_msgs::Int64 behavior_flag){
                behavior_ = behavior_flag.data;
            }

            void TrajectoryCore::autoware_traj_callback(const autoware_msgs::Lane& traj){
                ROS_WARN("autoware_traj_callback");
                ROS_ERROR("status_x is : %f, status_y is : %f", status_->pose.position.x, status_->pose.position.y);
                if(status_->pose.position.x == 0){
                    return;
                }
                current_waypoint_ = spline_fitter_->calculate(status_);
                float goal_vel_s = 4.0;
                int total_num = 50;
                float delta_time = 0.2;

                generate_local_trajectory(goal_vel_s, total_num, delta_time);
            }

            void TrajectoryCore::trajectory_pub_timer(const ros::TimerEvent& event) {

                local_trajectory_->header.stamp = ros::Time::now();
                local_trajectory_->header.frame_id = "map";
                local_trajectory_->direction = true;
                local_trajectory_->auto_mode = 0x95;
                local_trajectory_->estop = false;
                ROS_INFO("local trajectory pub: points: %ld", local_trajectory_->waypoints.size());
                trajectory_pub_.publish(*local_trajectory_);
            }

            void TrajectoryCore::generate_local_trajectory(float goal_vel_s, int total_num , float delta_time){
                ROS_WARN("generate_local_trajectory");
                local_traj_counter_++;
                std::vector<float>().swap(theta_array_);
                std::vector<autobots_msgs::AutobotsWaypoint> waypoints;
                waypoints.push_back(*current_waypoint_);
                theta_array_.push_back(current_waypoint_->theta);

                autobots_msgs::AutobotsWaypoint temp_waypoint;
                float curr_s = current_waypoint_->station;
                ROS_WARN("current s is : %f,current d is: %f", curr_s, current_waypoint_->lateral);
                if(paths_.size() > 0){
                    float min_dist = 300.0;
                    int closest_idx = 0;
                    for(int i = 0; i < paths_.size(); ++i){
                        float dis = fabs(paths_.at(i).station - curr_s);
                        if (dis < min_dist) {
                            closest_idx = i;
                            min_dist = dis;
                        }
                    }
                    if(paths_.at(closest_idx).station <= curr_s && closest_idx < paths_.size() - 1){
                        ++closest_idx;
                    }
                    ROS_WARN("closest idx is : %d", closest_idx);
                    for(int i = closest_idx, cnt = 0; i < paths_.size() && cnt < 10; ++i, ++cnt){
                        waypoints.push_back(paths_.at(i));
                        theta_array_.push_back(paths_.at(i).theta);
                    }
                }
                ROS_ERROR("current lateral is: %f", waypoints.back().lateral);
                ROS_WARN("waypoints size is : %d", waypoints.size());
                float init_s = waypoints.back().station;
                float init_d = waypoints.back().lateral;
                float yaw = waypoints.back().theta;
                float r_yaw = spline_fitter_->calculate_fit_theta(init_s);
                float vel_x = waypoints.back().speed * cos(yaw);
                float vel_y = waypoints.back().speed * sin(yaw);
                float init_vel_s = vel_x * cos(r_yaw) + vel_y * sin(r_yaw);
                float init_vel_d = vel_x * sin(r_yaw) + vel_y * cos(r_yaw);
                float acc_x = waypoints.back().accel * cos(yaw);
                float acc_y = waypoints.back().accel * sin(yaw);
                float init_acc_s = acc_x * cos(r_yaw) + acc_y * sin(r_yaw);
                float init_acc_d = acc_x * sin(r_yaw) + acc_y * cos(r_yaw);
                ROS_ERROR("init_s: %f, init_d: %f, init_vel_s: %f, init_vel_d: %f, init_acc_s: %f, init_acc_d: %f", init_s, init_d,
                          init_vel_s, init_vel_d, init_acc_s, init_acc_d);
                ROS_ERROR("r_yaw: %f, vel_x: %f, vel_y: %f, acc_x: %f, acc_y: %f", r_yaw, vel_x,
                          vel_y, acc_x, acc_y);

                if(behavior_ == 1){
                    lane_id_ = 0;
                } else if(behavior_ == 2){
                    lane_id_ = 1;
                } else if(behavior_ == 0){
                    goal_vel_s = 0.1;
                }
                float goal_d = lane_id_ * road_width_;
                ROS_ERROR("target d: %f, target v: %f", goal_d, goal_vel_s);
                int num_new_path = total_num - waypoints.size();
                generate_quintic_params(init_d, init_vel_d, init_acc_d, goal_d, num_new_path * delta_time);
                generate_quartic_params(init_s, init_vel_s, init_acc_s, goal_vel_s, num_new_path * delta_time);

                for(int i = 1; i < num_new_path; ++i){
                    temp_waypoint.station = calc_quartic_val(i * delta_time);
                    temp_waypoint.lateral = calc_quintic_val(i * delta_time);
                    temp_waypoint.point.x = spline_fitter_->SX_spline_->calculate(temp_waypoint.station);
                    temp_waypoint.point.y = spline_fitter_->SY_spline_->calculate(temp_waypoint.station);
                    temp_waypoint.speed = sqrt(pow(calc_quartic_first_derivative(i * delta_time), 2) + pow(calc_quintic_first_derivative(i * delta_time), 2));
                    temp_waypoint.accel = sqrt(pow(calc_quartic_second_derivative(i * delta_time), 2) + pow(calc_quintic_second_derivative(i * delta_time), 2));
                    if(i != 1){
                        theta_array_.push_back(atan2(temp_waypoint.point.y - waypoints.back().point.y, temp_waypoint.point.x - waypoints.back().point.x));
                    }
                    waypoints.push_back(temp_waypoint);
                }
                theta_array_.push_back(theta_array_.back());
                std::vector<float> filter_theta_array = meanfilter(theta_array_, 10);
                ROS_WARN("waypoints num: %d", int(waypoints.size()));
                ROS_WARN("theta num: %d", int(filter_theta_array.size()));
                for (int i = 0; i < waypoints.size(); i++) {
                    waypoints.at(i).theta = filter_theta_array.at(i);
                }
                calculate_kappa(waypoints);
                local_trajectory_->waypoints = waypoints;
                ROS_WARN("generated points: %ld", waypoints.size());
                paths_ = waypoints;

                if (fmod(local_traj_counter_, 100) == 0) {
                    record_traj(local_trajectory_);
                }
            }

            void TrajectoryCore::generate_quintic_params(float init_d, float init_v, float init_a, float goal_d, float t){
                float goal_v = 0;
                float goal_a = 0;
                quintic_params_[0] = init_d;
                quintic_params_[1] = init_v;
                quintic_params_[2] = init_a / 2.0;
                Eigen::Matrix3f H;
                ROS_INFO("init d is : %f, v is : %f, a is: %f, goal d is: %f, goal v is: %f, goal a is: %f, t is: %f",
                         init_d, init_v, init_a, goal_d, goal_v, goal_a, t);
                H << pow(t, 3), pow(t, 4), pow(t, 5),
                     3 * pow(t, 2), 4 * pow(t, 3), 5 * pow(t, 4),
                     6 * t, 12 * pow(t, 2), 20 * pow(t, 3);
                Eigen::Vector3f Y;
                Y << goal_d - (init_d  + init_v * t + init_a * pow(t, 2) / 2),
                     goal_v - (init_v + init_a * t),
                     goal_a - init_a;
                Eigen::Vector3f M = H.colPivHouseholderQr().solve(Y);
                for(int i = 0; i < 3; ++i){
                    quintic_params_[i + 3] = M(i);
                }
                ROS_ERROR("quintic params are: %f, %f, %f, %f, %f, %f", quintic_params_[0], quintic_params_[1],
                        quintic_params_[2], quintic_params_[3], quintic_params_[4], quintic_params_[5]);
            }

            void TrajectoryCore::generate_quartic_params(float init_s, float init_v, float init_a, float goal_v, float t){
                float goal_a = 0;
                quartic_params_[0] = init_s;
                quartic_params_[1] = init_v;
                quartic_params_[2] = init_a / 2.0;
                ROS_INFO("init s is : %f, v is : %f, a is: %f, goal v is: %f, goal a is: %f, t is: %f",
                         init_s, init_v, init_a, goal_v, goal_a, t);
                Eigen::Matrix2f H;
                H << 3 * pow(t, 2), 4 * pow(t, 3),
                     6 * t, 12 * pow(t, 2);
                Eigen::Vector2f Y;
                Y << goal_v - (init_v + init_a * t),
                     goal_a - init_a;
                Eigen::Vector2f M = H.colPivHouseholderQr().solve(Y);
                quartic_params_[3] = M(0);
                quartic_params_[4] = M(1);
                ROS_ERROR("quartic params are: %f, %f, %f, %f, %f, %f", quartic_params_[0], quartic_params_[1],
                          quartic_params_[2], quartic_params_[3], quartic_params_[4], quartic_params_[5]);
            }

            float TrajectoryCore::calc_quintic_val(float t){
                float res = quintic_params_[0] + quintic_params_[1] * t + quintic_params_[2] * pow(t, 2) +
                        quintic_params_[3] * pow(t, 3) +  quintic_params_[4] * pow(t, 4) +  quintic_params_[5] * pow(t, 5);
                return res;
            }

            float TrajectoryCore::calc_quintic_first_derivative(float t){
                float res = quintic_params_[1] + 2 * quintic_params_[2] * t + 3 * quintic_params_[3] * pow(t, 2) +
                        4 * quintic_params_[4] * pow(t, 3) +  5 * quintic_params_[5] * pow(t, 4);
                return res;
            }

            float TrajectoryCore::calc_quintic_second_derivative(float t){
                float res = 2 * quintic_params_[2] + 6 * quintic_params_[3] * t +
                            12 * quintic_params_[4] * pow(t, 2) +  20 * quintic_params_[5] * pow(t, 3);
                return res;
            }

            float TrajectoryCore::calc_quintic_third_derivative(float t){
                float res = 6 * quintic_params_[3] + 24 * quintic_params_[4] * t +  60 * quintic_params_[5] * pow(t, 2);
                return res;
            }


            float TrajectoryCore::calc_quartic_val(float t){
                float res = quintic_params_[0] + quintic_params_[1] * t + quintic_params_[2] * pow(t, 2) +
                            quintic_params_[3] * pow(t, 3) +  quintic_params_[4] * pow(t, 4);
                return res;
            }

            float TrajectoryCore::calc_quartic_first_derivative(float t){
                float res = quintic_params_[1] + 2 * quintic_params_[2] * t + 3 * quintic_params_[3] * pow(t, 2) +
                            4 * quintic_params_[4] * pow(t, 3);
                return res;
            }

            float TrajectoryCore::calc_quartic_second_derivative(float t){
                float res = 2 * quintic_params_[2] + 6 * quintic_params_[3] * t + 12 * quintic_params_[4] * pow(t, 2);
                return res;
            }

            float TrajectoryCore::calc_quartic_third_derivative(float t){
                float res = 6 * quintic_params_[3] + 24 * quintic_params_[4] * t;
                return res;
            }

            std::vector<float> TrajectoryCore::meanfilter(std::vector<float> data, int meanwindow) {
                int data_length = data.size();
                int half_window = meanwindow / 2;
                int left_index = 0;
                int right_index = 0;
                float temp_sum = 0.0;
                float temp_average = 0.0;
                std::vector<float> filter_data;

                for (int i = 0; i < data_length; i++) {
                    left_index = i - half_window;
                    right_index = i + half_window;
                    if (left_index < 0) {
                        left_index = 0;
                    }
                    if (right_index >= data_length) {
                        right_index = data_length - 1;
                    }
                    for (int j = left_index; j < right_index; j++) {
                        temp_sum += data.at(j);
                    }
                    temp_average = temp_sum / float(right_index - left_index);
                    //ROS_WARN("left: %d, right:%d, sum: %.3f, ave:%.3f", left_index, right_index, temp_sum, temp_average);
                    filter_data.push_back(temp_average);
                    temp_sum = 0.0;
                }
                return filter_data;
            }

            void TrajectoryCore::calculate_kappa(std::vector<autobots_msgs::AutobotsWaypoint> waypoints) {
                float temp_dis = 0.0;
                float kappa = 0.0;

                for (int i = 0; i < waypoints.size() - 1; i++) {
                    if (i <= waypoints.size() - 2) {
                        temp_dis = distance(waypoints.at(i + 1).point, waypoints.at(i).point);
                        kappa = unify_angle(waypoints.at(i + 1).theta - waypoints.at(i).theta) / temp_dis;
                        waypoints.at(i).kappa = kappa;
                    } else {
                        waypoints.at(i).kappa = waypoints.at(i - 1).kappa;
                    }
                }
            }

            float TrajectoryCore::unify_angle(float angle) {
                float angle_mod = fmod(angle, M_PI * 2);
                while (angle_mod < - M_PI) {
                    angle_mod = angle_mod + 2 * M_PI;
                }
                while (angle_mod > M_PI) {
                    angle_mod = angle_mod - 2 * M_PI;
                }
                return angle_mod;
            }

            float TrajectoryCore::distance(const geometry_msgs::Point& pt1, const geometry_msgs::Point& pt2) {
                float dis = sqrt(pow((pt2.y - pt1.y), 2) + pow((pt2.x - pt1.x), 2));
                return dis;
            }

            void TrajectoryCore::record_traj(autobots_msgs::AutobotsTrajectory* traj) {
            #ifndef BUILD_TRIAL_VERSION
                time_t rawtime;
                time(&rawtime);
                if(!boost::filesystem::exists("autobots_planning")) {
                    boost::filesystem::create_directories("autobots_planning");
                }
                std::string file_name = "autobots_planning/planning_log_" + std::to_string(log_file_counter_++) + ".csv";
                log_file_ = fopen(file_name.data(), "w");

                fprintf(log_file_,
                        "status_x,"
                        "status_y,"
                        "local_x,"
                        "local_y,"
                        "station,"
                        "lateral,"
                        "raw_theta,"
                        "filter_theta,"
                        "kappa,"
                        "speed,"
                        "accel,"
                        " \n");

                //log the data
                for (int i = 0; i < traj->waypoints.size(); i++) {
                    fprintf(log_file_, "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\r\n",
                            status_->pose.position.x,
                            status_->pose.position.y,
                            traj->waypoints[i].point.x,
                            traj->waypoints[i].point.y,
                            traj->waypoints[i].station,
                            traj->waypoints[i].lateral,
                            theta_array_[i],
                            traj->waypoints[i].theta,
                            traj->waypoints[i].kappa,
                            traj->waypoints[i].speed,
                            traj->waypoints[i].accel
                    );
                    fflush(log_file_);
                }
                fclose(log_file_);
            #endif
            }

        }
    }
}
