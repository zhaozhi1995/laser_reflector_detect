/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/ros_qt5/qnode.h"
#include "visualization_msgs/MarkerArray.h"
#include "math.h"

#include <iostream>
#include <fstream>

#include <nav_msgs/Odometry.h>

//#define VISUAL_RVIZ

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qt_test
{

/*****************************************************************************
** Implementation
*****************************************************************************/

    QNode::QNode(ros::NodeHandle node_handle, tf2_ros::Buffer *buffer)
        : tf_buffer_node_(buffer)
    {
        n = node_handle;
        tf_ = new tf::TransformListener(ros::Duration(10.0));
    }

    QNode::~QNode()
    {
        if (ros::isStarted())
        {
            ros::shutdown(); // explicitly needed since we use ros::start();
            ros::waitForShutdown();
        }
        wait();
        if (tf_ != NULL)
            delete tf_;
    }

    int QNode::round_int(double val)
    {
        return (val > 0.0) ? floor(val + 0.5) : ceil(val - 0.5);
    }

    bool QNode::init()
    {
        if (!ros::master::check())
        {
            ROS_INFO("ros master fail");
            return false;
        }
        else
        {
            ROS_INFO("ros master success");
        }

        //ros::param::get("/laser_reflector_detect_node/angle_error_scale", angle_error_scale);
        //ros::param::get("/laser_reflector_detect_node/distance_error_scale", distance_error_scale);
        ros::param::get("/laser_reflector_detect_node/reflector_combined_length", reflector_combined_length);
        //反光柱的半径
        ros::param::get("/laser_reflector_detect_node/reflector_radius", reflector_radius);
        //检测反光柱的最小点数
        ros::param::get("/laser_reflector_detect_node/min_reflector_sample_count", min_reflector_sample_count);
        //检测反光柱的最小强度
        ros::param::get("/laser_reflector_detect_node/min_reflector_intensity", min_reflector_intensity);
        //导航时定位的反光柱的大小
        ros::param::get("/laser_reflector_detect_node/kLandmarkMarkerScale", kLandmarkMarkerScale);
        //使用反光柱的模式
        ros::param::get("/laser_reflector_detect_node/reflector_combination_mode", reflector_combination_mode);
        //激光坐标系的名称
        ros::param::get("/laser_reflector_detect_node/lidar_frame", lidar_frame_);
        //地图坐标系的名称
        ros::param::get("/laser_reflector_detect_node/map_frame", map_frame_);
        //导航时可允许的反光柱检测误差
        ros::param::get("/laser_reflector_detect_node/match_distance_accepted", match_distance_acceped);

        std::cout << "lidar_frame_: " << lidar_frame_ << std::endl;
        std::cout << "map_frame_: " << map_frame_ << std::endl;
        std::cout << "match_distance_acceped: " << match_distance_acceped << std::endl;

        //订阅激光话题
        scan_sub_ = n.subscribe("/scan", 1, &QNode::laserProcess, this);

        //订阅全局反光板话题
        global_reflector_pos_sub_ = n.subscribe("/landmark_poses_list", 1, &QNode::getGlobalReflector, this);

        //订阅机器人当前位姿话题
        robot_pose_sub_ = n.subscribe("/base_link_odom", 1, &QNode::getRobotPose, this);

        //订阅当前建图与导航系统状态
        system_working_state_sub_ = n.subscribe("working_state", 1, &QNode::getSystemWoringState, this);

        //检测到的反光板
        reflector_points_ = n.advertise<geometry_msgs::PointStamped>("reflector_points", 1);

        //发送给cartographer的landmark
        reflector_landmark_ = n.advertise<cartographer_ros_msgs::LandmarkList>("Laserlandmark", 1);

        //定位时检测到的反光板
        current_landmark_list_pub_ = n.advertise<::visualization_msgs::MarkerArray>("current_landmark_list", 1);

        //使用反光板定位发布的机器人位姿
        //reflector_localization_pos_pub_ = n.advertise<::visualization_msgs::MarkerArray>("reflector_localization_pos", 1);

        //机器人到雷达的外参
        base_link_to_lidar = tf::Transform(tf::createQuaternionFromRPY(0, 0, -0.005406), tf::Vector3(0.169713, 0.003589, 0));
        
        //雷达未矫正的移动路线
        lidar_uncorrected_pub_ = n.advertise<nav_msgs::Path>("lidar/uncorrcted_path", 1);

        //雷达矫正后的移动路线
        lidar_corrected_pub_ = n.advertise<nav_msgs::Path>("lidar/corrcted_path", 1);
        
        //雷达矫正与未矫正之间的连线
        error_edge_pub_ = n.advertise<visualization_msgs::MarkerArray>("/lidar/error_edge", 1);

        //反光柱矫正后的机器人位姿
        robot_pose_pub_ = n.advertise<cartographer_ros_msgs::RobotPose>("/robot_pose", 1);

        return true;
    }

    void QNode::log(const LogLevel &level, const std::string &msg)
    {
        /*
        logging_model.insertRows(logging_model.rowCount(), 1);
        std::stringstream logging_model_msg;
        switch (level)
        {
        case (Debug):
        {
            ROS_DEBUG_STREAM(msg);
            logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
            break;
        }
        case (Info):
        {
            ROS_INFO_STREAM(msg);
            logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
            break;
        }
        case (Warn):
        {
            ROS_WARN_STREAM(msg);
            logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
            break;
        }
        case (Error):
        {
            ROS_ERROR_STREAM(msg);
            logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
            break;
        }
        case (Fatal):
        {
            ROS_FATAL_STREAM(msg);
            logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
            break;
        }
        }
        QVariant new_row(QString(logging_model_msg.str().c_str()));
        logging_model.setData(logging_model.index(logging_model.rowCount() - 1), new_row);
        Q_EMIT loggingUpdated(); // used to readjust the scrollbar
        */
    }

    void QNode::laserProcess(const sensor_msgs::LaserScanConstPtr &scan)
    {
        //tf::Stamped<tf::Pose> laser_pose; // later is used for localization
        tf::Pose laser_pose;
        std::vector<laser_processor::Sample *> scan_filter;

        for (uint32_t i = 0; i < scan->ranges.size(); i++)
        {
            laser_processor::Sample *s = laser_processor::Sample::Extract(i, min_reflector_intensity, *scan);
            if (s != NULL) scan_filter.push_back(s);
        }

        if (scan_filter.size() == 0) return;

        //get the center of reflector
        double center_x, center_y, center_yaw, center_count, center_distance;
        std::vector<Reflector_pos> reflectors_;
        //double last_item_x, last_item_y, last_item_range;
        bool is_mark_start = true;

        // store the temp data of reflector
        std::vector<std::pair<double, double>> reflector_data; //first is angle, second is distance;
        // store the intensity data of reflector
        std::vector<int> intensity_data;

        //std::cout << "peak.ding scan_filter size " << scan_filter.size() << std::endl;
        //检测有多少个反光柱
        for (int i = 0; i < scan_filter.size(); i++)
        {
            is_mark_start = true;

            for (int j = i; j < scan_filter.size(); j++)
            {
                laser_processor::Sample *p = scan_filter[j];

                //std::cout << "peak.ding last index " << p->index << std::endl;

                //double item_x = p->x;
                //double item_y = p->y;
                double item_r = p->range;
                double item_i = p->intensity;
                //origin angle range is -180~180,transfer to 0~360
                double item_a = scan->angle_min + p->index * scan->angle_increment + M_PI;

                if (is_mark_start == true)
                {
                    //center_x = p->x;
                    //center_y = p->y;
                    //center_yaw = item_angle;
                    center_count = 1;
                    //开始记录单个反光柱的角度和距离
                    reflector_data.clear();
                    reflector_data.push_back(std::pair<double, double>(item_a, item_r));
                    //std::cout << "Peak.ding index " << p->index << " item_angle " << item_a << std::endl;
                    //开始记录单个反光柱的强度
                    intensity_data.clear();
                    intensity_data.push_back(item_i);

                    is_mark_start = false;
                }
                else
                {
                    //double d_x = item_x - last_item_x;
                    //double d_y = item_y - last_item_y;
                    //double d_xy = sqrt(pow(d_x, 2) + pow(d_y, 2));
                    //std::cout << "Peak.ding d_xy " << d_xy << std::endl;

                    //double distance_internal = tan(scan->angle_increment) * last_item_range;

                    //double angle_internal =  atan2(d_xy,last_item_range); //d_xy / item_range;

                    //angle_error = scan->angle_increment * angle_error_scale;
                    //distance_error = 2 * reflector_radius;

                    //check the scan point not the last one
                    //if ((angle_internal < angle_error || d_xy < distance_error) && (j != scan_filter.size() - 1))
                    //检测强度点是否连续来进行分割
                    if ((p->index - last_index) < 3 && (j != scan_filter.size() - 1))
                    {
                        //center_x += p->x;
                        //center_y += p->y;
                        //center_yaw += center_yaw;
                       
                        reflector_data.push_back(std::pair<double, double>(item_a, item_r));
                        //std::cout << "Peak.ding index " << p->index << " item_angle " << item_a << std::endl;
                        //std::cout << "Peak.ding reflector_data size " << reflector_data.size() << std::endl;

                        intensity_data.push_back(item_i);
                        center_count++;
                    }
                    else
                    {
                        //记录当前的反光柱点平均距离
                        double sum_range_of_target = 0;
                        for (auto it : reflector_data)
                        {
                            sum_range_of_target += it.second;
                        }
                        double avg_range = sum_range_of_target / reflector_data.size();

                        //记录当前反光柱子的平均强度
                        double sum_intensity_of_target = 0;
                        for (auto it : intensity_data)
                        {
                            sum_intensity_of_target += it;
                        }
                        double avg_intensity = sum_intensity_of_target / intensity_data.size();

                        //check the points number of reflecor size is < 5cm/D/scan->angle_increment
                        //计算反光柱的有效点数量 放宽至理想值的0.6~1.2。
                        int max_size = std::floor((2.0 * reflector_radius / avg_range) / scan->angle_increment * 1.2);
                        int min_size = max_size * 0.6 > min_reflector_sample_count ? max_size * 0.6 : min_reflector_sample_count;

                        /*
                        std::cout << "Peak.ding max_size " << max_size << std::endl;
                        std::cout << "Peak.ding min_size " << min_size << std::endl;
                        std::cout << "Peak.ding avg_range " << avg_range << std::endl;
                        std::cout << "Peak.ding avg_intensity " << avg_intensity << std::endl;
                        std::cout << "Peak.ding reflector_data.size() " << reflector_data.size() << std::endl;
                        */

                        //if (center_count > min_reflector_sample_count)
                        if (center_count <= max_size &&
                            center_count >= min_size &&
                            avg_intensity > 850 &&
                            avg_range > 1 &&
                            avg_range < 6)
                        {
                            //std::cout << "Peak.ding center_count " << center_count << std::endl;

                            //std::cout << "Peak.ding min_reflector_sample_count " << min_reflector_sample_count << std::endl;

                            //center_x /= center_count;
                            //center_y /= center_count;
                            //center_yaw /= center_count; //atan2(center_y, center_x);

                            //center_x += cos(center_yaw) * reflector_radius;
                            //center_y += sin(center_yaw) * reflector_radius;

                            //calculate the angle
                            for (int i = 0; i < reflector_data.size(); i++)
                            {
                                center_yaw += (reflector_data[i].first);
                                //std::cout << "Peak.ding " << i << " fisrt-> " << reflector_data[i].first << std::endl;
                                //std::cout << "Peak.ding " << i << " second-> " << reflector_data[i].second << std::endl;
                            }
                            center_yaw /= center_count; //average
                                                        //std::cout << "Peak.ding center_count " << center_count << std::endl;
                                                        //std::cout << "Peak.ding center_yaw " << center_yaw << std::endl;

                            //calculate the distance
                            for (int i = 0; i < reflector_data.size(); i++)
                            {
                                //part_1
                                double theta = fabs(reflector_data[i].first - center_yaw);
                                double ds_1 = reflector_data[i].second * cos(theta);
                                //std::cout << "Peak.ding theta " << theta << std::endl;
                                //std::cout << "Peak.ding ds_1 " << ds_1 << std::endl;

                                //part_2
                                double a = reflector_data[i].second * sin(theta);
                                double b = reflector_radius;
                                double angle = (a / b) > 1 ? 1.0 : a / b; // Bug fixed!!!
                                double theta_2 = asin(angle);
                                double ds_2 = reflector_radius * cos(theta_2);
                                //std::cout << "Peak.ding ds_2 " << ds_2 << std::endl;

                                //std::cout << "Peak.ding reflector_data[i].second " << reflector_data[i].second << std::endl;
                                //std::cout << "Peak.ding theta " << theta << std::endl;
                                //std::cout << "Peak.ding a " << a << std::endl;
                                //std::cout << "Peak.ding b " << b << std::endl;
                                //std::cout << "Peak.ding angle " << angle << std::endl;
                                //std::cout << "Peak.ding theta_2 " << theta_2 << std::endl;
                                //std::cout << "Peak.ding ds " << i << " " << ds_1+ds_2 << std::endl;

                                center_distance += (ds_1 + ds_2);
                                /*
                                if (std::isnan(center_distance))
                                {
                                    std::cout << "Peak.ding origin ds_1 " << ds_1 << std::endl;
                                    std::cout << "Peak.ding origin ds_2 " << ds_2 << std::endl;
                                    std::cout << "Peak.ding origin reflector_radius " << reflector_radius << std::endl;
                                    std::cout << "Peak.ding origin theta_2 " << theta_2 << std::endl;
                                    std::cout << "Peak.ding origin angle " << angle << std::endl;
                                    std::cout << "Peak.ding origin a " << a << std::endl;
                                    std::cout << "Peak.ding origin b " << b << std::endl;
                                    std::cout << "Peak.ding origin center_distance " << center_distance << std::endl;
                                }
                                */
                            }
                            center_distance /= center_count; //average
                            center_yaw -= M_PI;              //transfer to -180~180
                            //std::cout << "Peak.ding center_count " << center_count << std::endl;
                            //std::cout << "Peak.ding center_distance " << center_distance << std::endl;

                            Reflector_pos item;
                            item.center_x = center_distance * cos(center_yaw);
                            item.center_y = center_distance * sin(center_yaw);
                            item.center_yaw = center_yaw;
                            reflectors_.push_back(item);

                            /*
                            if(std::isnan(item.center_x))
                            {
                                std::cout << "Peak.ding center_distance " << center_distance << std::endl;
                                std::cout << "Peak.ding center_yaw " << center_yaw << std::endl;
                                std::cout << "Peak.ding item_r " << item_r << std::endl;
                                std::cout << "Peak.ding ds_1 " << ds_1 << std::endl;
                                std::cout << "Peak.ding ds_2 " << ds_2 << std::endl;
                                std::cout << "Peak.ding center_count " << center_count << std::endl;
                            }
                            */

                            //std::cout << "Peak.ding item.center_yaw " << item.center_yaw << std::endl;
                            //std::cout << "Peak.ding item.center_x " << item.center_x << std::endl;
                            //std::cout << "Peak.ding item.center_y " << item.center_y << std::endl;
                        }

                        center_x = 0.0;
                        center_y = 0.0;
                        center_count = 0;
                        center_yaw = 0;
                        center_distance = 0;
                        i = j - 1;
                        break;
                    }
                }

                //last_item_x = item_x;
                //last_item_y = item_y;
                //last_item_range = item_r;
                last_index = p->index;
            }
        }

        //peak add
        //检测到的反光柱的数量
        if (reflectors_.size() < 3)
        {
            std::cout << "Peak.ding reflectors_.size() " << reflectors_.size() << std::endl;
            for (auto it : reflectors_)
            {
                std::cout << "reflector x y " << it.center_x << " " << it.center_y << std::endl;
            }
        }

        for (int i = 0; i < reflectors_.size(); i++)
        {
            /*
            geometry_msgs::PointStamped point;
            point.header.stamp = scan->header.stamp;

            point.header.frame_id = "car_laser";

            point.point.x = reflectors_[i].center_x;
            point.point.y = reflectors_[i].center_y;
            point.point.z = 0;

            reflector_points_.publish(point);
            */
        }

        cartographer_ros_msgs::LandmarkList reflector_LandMarkList;

        //use two reflector as a landmark
        //log(Info, "reflectors_ size " + std::to_string(reflectors_.size() ));

        double origin_x, origin_y, origin_theta;
        origin_x = 0;
        origin_y = 0;
        origin_theta = 0;

        // start to classify reflector

        if (reflector_combination_mode == 1)
        {
            ros::Time dt = scan->header.stamp;

            //laser_pose = base_link_to_map * base_link_to_lidar;
            //reallocated match state;
            global_match_data.clear();
            global_match_sorted_data.clear();

            for (int i = 0; i < reflectors_.size(); i++)
            {
                bool is_match = false;
                int match_id = 0;

                //bool get_laser_pose = getLaserPose(laser_pose, dt, tf_);
                bool get_laser_pose = getLaserPose2(laser_pose, dt);

                if (get_laser_pose && global_reflectors.size() != 0)
                {
                    tf::Vector3 local_reflector_point;
                    local_reflector_point.setValue(reflectors_[i].center_x, reflectors_[i].center_y, 0);

                    tf::Vector3 global_reflector_point = laser_pose * local_reflector_point;

                    // loop match map reflectors
                    for (auto item : global_reflectors)
                    {
                        tf::Vector3 map_reflectors;
                        map_reflectors.setValue(item.second.x, item.second.y, 0);
                        //
                        double distance = global_reflector_point.distance(map_reflectors);

                        if (distance < match_distance_acceped)
                        {
                            match_id = item.first;
                            is_match = true;
                            //std::cout << "Peak.ding match id " << match_id << std::endl;

                            //double range = hypot(reflectors_[i].center_x, reflectors_[i].center_y);
                            //save match outcome
                            global_match_data.insert(std::pair<Reflector_pos, int>(reflectors_[i], match_id));
                            global_match_sorted_data.insert(std::pair<double, Reflector_pos>(reflectors_[i].center_yaw, reflectors_[i]));
                            break;
                        }
                    }
                }

                if (system_working_state_ == "slam")
                {
                    cartographer_ros_msgs::LandmarkEntry reflector_LandMarkEntry;
                    reflector_LandMarkEntry.id = is_match ? std::to_string(match_id) : "unknown";
                    //std::cout << "reflector_LandMarkEntry.id " << reflector_LandMarkEntry.id << std::endl;

                    reflector_LandMarkEntry.tracking_from_landmark_transform.position.x = reflectors_[i].center_x;
                    reflector_LandMarkEntry.tracking_from_landmark_transform.position.y = reflectors_[i].center_y;
                    //std::cout << "Peak.ding reflectors_[i].center_x " << reflectors_[i].center_x <<std::endl;
                    reflector_LandMarkEntry.tracking_from_landmark_transform.position.z = 0;
                    tf::Quaternion quat = tf::createQuaternionFromYaw(-1.0 * origin_theta);
                    reflector_LandMarkEntry.tracking_from_landmark_transform.orientation.x = 0;
                    reflector_LandMarkEntry.tracking_from_landmark_transform.orientation.y = 0;
                    reflector_LandMarkEntry.tracking_from_landmark_transform.orientation.z = 0;
                    reflector_LandMarkEntry.tracking_from_landmark_transform.orientation.w = 1;
                    reflector_LandMarkEntry.translation_weight = 20;
                    reflector_LandMarkEntry.rotation_weight = 0;
                    reflector_LandMarkEntry.type = "reflector";

                    reflector_LandMarkList.header.frame_id = "car_laser";
                    reflector_LandMarkList.header.stamp = scan->header.stamp; //ros::Time::now();
                    reflector_LandMarkList.landmarks.push_back(reflector_LandMarkEntry);
                }
            }
        }

        if (reflector_combination_mode == 2 && reflectors_.size() == 2)
        {
            for (int i = 0; i < reflectors_.size(); i++)
            {
                Reflector_pos item_i = reflectors_[i];
                //log(Info, "item_i.center_yaw : " + std::to_string(item_i.center_yaw));

                for (int j = i + 1; j < reflectors_.size(); j++)
                {
                    Reflector_pos item_j = reflectors_[j];
                    //log(Info, "item_j.center_yaw : " + std::to_string(item_j.center_yaw));

                    //filter the refelectors

                    double x_ij = fabs(item_i.center_x - item_j.center_x);
                    double y_ij = fabs(item_i.center_y - item_j.center_y);
                    double distance_ij = sqrt(pow(x_ij, 2) + pow(y_ij, 2));
                    //log(Info, "item_i.center_x : " + std::to_string(item_i.center_x));
                    //log(Info, "item_i.center_y : " + std::to_string(item_i.center_y));
                    //log(Info, "item_j.center_x : " + std::to_string(item_j.center_x));
                    //log(Info, "item_j.center_y : " + std::to_string(item_j.center_y));
                    //log(Info, "x_ij : " + std::to_string(x_ij));
                    //log(Info, "y_ij : " + std::to_string(y_ij));
                    //log(Info, "distance_ij : " + std::to_string(distance_ij));

                    //if two reflectors not arrange by special mode, jump out
                    //reflector_combined_length是两个反光柱的距离（定值）
                    if (fabs(distance_ij - reflector_combined_length) > 0.2)
                        continue;

                    //okagv
                    std::string outcomeAds = "/home/peak/okagv/reflector_distance_measure.txt";
                    std::ofstream outfile;
                    outfile.open(outcomeAds, std::ios::app);

                    outfile << " " << distance_ij << std::endl;
                    //std::cout << " " << distance_ij << std::endl;

                    //
                    double distance_i = sqrt(pow(item_i.center_x, 2) + pow(item_i.center_y, 2));
                    double distance_j = sqrt(pow(item_j.center_x, 2) + pow(item_j.center_y, 2));

                    double coord_x, coord_y, coord_theta;

                    //第一种情况，两个反光柱在激光扫描的同一侧
                    if (fabs(item_i.center_yaw - item_j.center_yaw) < M_PI)
                    {
                        //这时候一定有一个反光柱的角度是大于另一个的
                        if (item_i.center_yaw > item_j.center_yaw)
                        {
                            //item_i is origin
                            double up_value = pow(distance_i, 2) + pow(distance_ij, 2) - pow(distance_j, 2);
                            double down_value = 2 * distance_i * distance_ij;
                            double cos_theta = acos(up_value / down_value);
                            coord_x = cos(cos_theta) * distance_i;
                            coord_y = sin(cos_theta) * distance_i;
                            origin_x = item_i.center_x;
                            origin_y = item_i.center_y;
                            origin_theta = atan2(1, 0) - atan2(item_j.center_y - item_i.center_y, item_j.center_x - item_i.center_x);
                            if (origin_theta > M_PI)
                            {
                                origin_theta -= 2 * M_PI;
                            }
                            if (origin_theta < -M_PI)
                            {
                                origin_theta += 2 * M_PI;
                            }
                        }
                        else
                        {
                            //item_j is origin
                            double up_value = pow(distance_j, 2) + pow(distance_ij, 2) - pow(distance_i, 2);
                            double down_value = 2 * distance_j * distance_ij;
                            double cos_theta = acos(up_value / down_value);
                            coord_x = cos(cos_theta) * distance_j;
                            coord_y = sin(cos_theta) * distance_j;
                            coord_theta = atan2(coord_y, coord_x);
                            origin_x = item_j.center_x;
                            origin_y = item_j.center_y;
                            origin_theta = atan2(1, 0) - atan2(item_i.center_y - item_j.center_y, item_i.center_x - item_j.center_x);
                            if (origin_theta > M_PI)
                            {
                                origin_theta -= 2 * M_PI;
                            }
                            if (origin_theta < -M_PI)
                            {
                                origin_theta += 2 * M_PI;
                            }
                        }
                    }
                    else
                    {
                        //两个反光柱在激光扫描0/360的两侧，这时两个角度之差一定大于180
                        if (item_i.center_yaw < item_j.center_yaw)
                        {
                            //item_i is origin
                            //下面这些好像没用上
                            double up_value = pow(distance_i, 2) + pow(distance_ij, 2) - pow(distance_j, 2);
                            double down_value = 2 * distance_i * distance_ij;
                            double cos_theta = acos(up_value / down_value);
                            coord_x = cos(cos_theta) * distance_i;
                            coord_y = sin(cos_theta) * distance_i;
                            coord_theta = atan2(coord_y, coord_x);

                            origin_x = item_i.center_x;
                            origin_y = item_i.center_y;
                            origin_theta = atan2(1, 0) - atan2(item_j.center_y - item_i.center_y, item_j.center_x - item_i.center_x);
                            if (origin_theta > M_PI)
                            {
                                origin_theta -= 2 * M_PI;
                            }
                            if (origin_theta < -M_PI)
                            {
                                origin_theta += 2 * M_PI;
                            }
                        }
                        else
                        {
                            //item_j is origin
                            double up_value = pow(distance_j, 2) + pow(distance_ij, 2) - pow(distance_i, 2);
                            double down_value = 2 * distance_j * distance_ij;
                            double cos_theta = acos(up_value / down_value);
                            coord_x = cos(cos_theta) * distance_j;
                            coord_y = sin(cos_theta) * distance_j;
                            coord_theta = atan2(coord_y, coord_x);

                            origin_x = item_j.center_x;
                            origin_y = item_j.center_y;
                            origin_theta = atan2(1, 0) - atan2(item_i.center_y - item_j.center_y, item_i.center_x - item_j.center_x);
                            if (origin_theta > M_PI)
                            {
                                origin_theta -= 2 * M_PI;
                            }
                            if (origin_theta < -M_PI)
                            {
                                origin_theta += 2 * M_PI;
                            }
                        }
                    }

                    //log(Info, "origin_x : " + std::to_string(origin_x));
                    //log(Info, "origin_y : " + std::to_string(origin_y));
                    //log(Info, "origin_theta : " + std::to_string(origin_theta));

                    /*
            Reflector_pos item;
            item.center_x = (item_i.center_x + item_j.center_x) / 2;
            item.center_y = (item_i.center_y + item_j.center_y) / 2;
            item.center_yaw = atan2(item.center_y , item.center_x);// + M_PI_2;
            log(Info, "item_i.center_yaw : " + std::to_string(item_i.center_yaw));
            log(Info, "item_j.center_yaw : " + std::to_string(item_j.center_yaw));
            //log(Info, "item.center_yaw : " + std::to_string(item.center_yaw));


            //publish the center points
            geometry_msgs::PointStamped point;
            point.header.stamp = scan->header.stamp;
            point.header.frame_id = "car_laser";

            point.point.x = item.center_x;
            point.point.y = item.center_y;
            point.point.z = 0;

            reflector_points_.publish(point);
            */
                }
            }
        }

        /*
        if (reflector_combination_mode == 3 && reflectors_.size() == 3)
        {
            //log(Info, "reflectors_.size() = " + std::to_string(reflectors_.size()));
            Reflector_pos item_j, item_k;

            for (int i = 0; i < reflectors_.size(); i++)
            {
                Reflector_pos item_i = reflectors_[i];
                
                //log(Info, "i = " + std::to_string(i));
                switch (i)
                {
                case 0 :
                    //log(Info, "-------------------------vetex is 0 --------------------------");
                    item_j = reflectors_[1];
                    item_k = reflectors_[2];
                    break;
                case 1 :
                    //log(Info, "-------------------------vetex is 1 --------------------------");
                    item_j = reflectors_[0];
                    item_k = reflectors_[2];
                    break;
                case 2 :
                    //log(Info, "-------------------------vetex is 2 --------------------------");
                    item_j = reflectors_[0];
                    item_k = reflectors_[1];
                    break;
                }

                double theta_1 = atan2(item_i.center_y - item_j.center_y, item_i.center_x - item_j.center_x);
                //log(Info, "theta_1 is " + std::to_string(theta_1));
                double theta_2 = atan2(item_i.center_y - item_k.center_y, item_i.center_x - item_k.center_x);
                //log(Info, "theta_2 is " + std::to_string(theta_2));

                //degree detect error is not less 20
                double f_theta = fabs(theta_1 - theta_2);
                //log(Info, "f_theta is " + std::to_string(f_theta));
                //log(Info, "fabs(f_theta - M_PI/2) = " + std::to_string(fabs(f_theta - M_PI/2)) );
                if (fabs(f_theta - M_PI/2) > M_PI/9) continue;

                //log(Info, "i = " + std::to_string(i) + " satisfy condition");
                //if satisfy condition (less 20) then
                double x_ij = fabs(item_i.center_x - item_j.center_x);
                double y_ij = fabs(item_i.center_y - item_j.center_y);
                double distance_ij = sqrt(pow(x_ij, 2) + pow(y_ij, 2));
                //log(Info, "distance_ij is = " + std::to_string(distance_ij));
                double x_ik = fabs(item_i.center_x - item_k.center_x);
                double y_ik = fabs(item_i.center_y - item_k.center_y);
                double distance_ik = sqrt(pow(x_ik, 2) + pow(y_ik, 2));
                //log(Info, "distance_ik is = " + std::to_string(distance_ik));

                double long_distance = 0;
                if (distance_ij > distance_ik)
                {
                    //ij is the axis x
                    long_distance = distance_ij;
                    origin_theta = atan2(1, 0) - atan2(item_i.center_y - item_j.center_y, item_i.center_x - item_j.center_x);
                    //log(Info, "origin_theta = " + std::to_string(origin_theta));
                }
                else
                {
                    //ik is the axis_x
                    long_distance = distance_ik;
                    origin_theta = atan2(1, 0) - atan2(item_i.center_y - item_k.center_y, item_i.center_x - item_k.center_x);
                    //log(Info, "origin_theta = " + std::to_string(origin_theta));
                }

                //the long side of right angle is even number;
                if(round_int(long_distance) % 2 != 0) return;
                //log(Info, "long_distance = " + std::to_string(long_distance));
                origin_x = item_i.center_x;
                origin_y = item_i.center_y;
                //log(Info, "origin_x = " + std::to_string(origin_x));
                //log(Info, "origin_y = " + std::to_string(origin_y));
            }
        }
        */

        if (reflector_combination_mode == 3 && reflectors_.size() == 3)
        {
            //log(Info, "reflectors_.size() = " + std::to_string(reflectors_.size()));
            Reflector_pos item_j, item_k;

            for (int i = 0; i < reflectors_.size(); i++)
            {
                Reflector_pos item_i = reflectors_[i];

                //log(Info, "i = " + std::to_string(i));
                switch (i)
                {
                case 0:
                    //log(Info, "-------------------------vetex is 0 --------------------------");
                    item_j = reflectors_[1];
                    item_k = reflectors_[2];
                    break;
                case 1:
                    //log(Info, "-------------------------vetex is 1 --------------------------");
                    item_j = reflectors_[0];
                    item_k = reflectors_[2];
                    break;
                case 2:
                    //log(Info, "-------------------------vetex is 2 --------------------------");
                    item_j = reflectors_[0];
                    item_k = reflectors_[1];
                    break;
                }

                double theta_1 = atan2(item_i.center_y - item_j.center_y, item_i.center_x - item_j.center_x);
                //log(Info, "theta_1 is " + std::to_string(theta_1));
                double theta_2 = atan2(item_i.center_y - item_k.center_y, item_i.center_x - item_k.center_x);
                //log(Info, "theta_2 is " + std::to_string(theta_2));

                //degree detect error is not less 20
                double f_theta = fabs(theta_1 - theta_2);
                //log(Info, "f_theta is " + std::to_string(f_theta));
                //log(Info, "fabs(f_theta - M_PI/2) = " + std::to_string(fabs(f_theta - M_PI/2)) );

                if (fabs(f_theta - M_PI) < M_PI / 13 || fabs(f_theta) < M_PI / 13)
                {
                    if (fabs(f_theta) < M_PI / 13)
                        continue;
                }
                else
                {
                    return;
                }

                //log(Info, "i = " + std::to_string(i) + " satisfy condition");
                //if satisfy condition (less 20) then
                double x_ij = fabs(item_i.center_x - item_j.center_x);
                double y_ij = fabs(item_i.center_y - item_j.center_y);
                double distance_ij = sqrt(pow(x_ij, 2) + pow(y_ij, 2));
                //log(Info, "distance_ij is = " + std::to_string(distance_ij));
                double x_ik = fabs(item_i.center_x - item_k.center_x);
                double y_ik = fabs(item_i.center_y - item_k.center_y);
                double distance_ik = sqrt(pow(x_ik, 2) + pow(y_ik, 2));
                //log(Info, "distance_ik is = " + std::to_string(distance_ik));

                double long_distance = 0;
                double short_distance = 0;
                if (distance_ij > distance_ik)
                {
                    //ij is the axis x
                    long_distance = distance_ij;
                    short_distance = distance_ik;
                    origin_theta = atan2(1, 0) - atan2(item_i.center_y - item_j.center_y, item_i.center_x - item_j.center_x);
                    //log(Info, "origin_theta = " + std::to_string(origin_theta));
                }
                else
                {
                    //ik is the axis_x
                    long_distance = distance_ik;
                    short_distance = distance_ij;
                    origin_theta = atan2(1, 0) - atan2(item_i.center_y - item_k.center_y, item_i.center_x - item_k.center_x);
                    //log(Info, "origin_theta = " + std::to_string(origin_theta));
                }

                //the long side of right angle is even number;
                //if(round_int(long_distance) % 2 != 0) return;

                //log(Info, "ratio = " + std::to_string(round_int(long_distance/short_distance)));

                //below is set for test, chang what you need reflector mode
                if (fabs(long_distance - 1.25) > 0.1)
                    return;
                if (fabs(short_distance - 0.45) > 0.1)
                    return;
                if (round_int(long_distance / short_distance) != 3)
                    return;

                //log(Info, "long_distance = " + std::to_string(long_distance));
                //log(Info, "distance_ij is = " + std::to_string(distance_ij));
                //log(Info, "distance_ik is = " + std::to_string(distance_ik));
                //log(Info, "theta_1 is " + std::to_string(theta_1));
                //log(Info, "theta_2 is " + std::to_string(theta_2));
                origin_x = item_i.center_x;
                origin_y = item_i.center_y;
                //log(Info, "origin_x = " + std::to_string(origin_x));
                //log(Info, "origin_y = " + std::to_string(origin_y));
                //log(Info, "f_theta is " + std::to_string(f_theta));
            }
        }

        if (reflector_combination_mode != 1)
        {
            double distance = sqrt(pow(origin_x, 2) + pow(origin_y, 2));
            if (distance < 1.0)
                return;
            //
            cartographer_ros_msgs::LandmarkEntry reflector_LandMarkEntry;
            reflector_LandMarkEntry.id = "landmark_1";
            reflector_LandMarkEntry.tracking_from_landmark_transform.position.x = origin_x;
            reflector_LandMarkEntry.tracking_from_landmark_transform.position.y = origin_y;
            reflector_LandMarkEntry.tracking_from_landmark_transform.position.z = 0;
            tf::Quaternion quat = tf::createQuaternionFromYaw(-1.0 * origin_theta);
            reflector_LandMarkEntry.tracking_from_landmark_transform.orientation.x = quat.x();
            reflector_LandMarkEntry.tracking_from_landmark_transform.orientation.y = quat.y();
            reflector_LandMarkEntry.tracking_from_landmark_transform.orientation.z = quat.z();
            reflector_LandMarkEntry.tracking_from_landmark_transform.orientation.w = quat.w();
            reflector_LandMarkEntry.translation_weight = 20;
            reflector_LandMarkEntry.rotation_weight = 20;
            reflector_LandMarkEntry.type = "reflector_combined";

            reflector_LandMarkList.header.frame_id = "car_laser";
            reflector_LandMarkList.header.stamp = scan->header.stamp; //ros::Time::now();
            reflector_LandMarkList.landmarks.push_back(reflector_LandMarkEntry);
        }

#ifdef VISUAL_RVIZ
        visualization_msgs::Marker landmark_item;
        landmark_item.pose.position.x = origin_x;
        landmark_item.pose.position.y = origin_y;
        landmark_item.pose.orientation.x = 0.0;
        landmark_item.pose.orientation.y = 0.0;
        landmark_item.pose.orientation.z = 0.0;
        landmark_item.pose.orientation.w = 1.0;
        landmark_item.header.frame_id = "car_laser";
        landmark_item.header.stamp = scan->header.stamp; //ros::Time::now();
        landmark_item.scale.x = kLandmarkMarkerScale;
        landmark_item.scale.y = kLandmarkMarkerScale;
        landmark_item.scale.z = kLandmarkMarkerScale;
        landmark_item.type = visualization_msgs::Marker::SPHERE;
        landmark_item.ns = "Landmarks";
        landmark_item.id = reflector_LandMarkList.landmarks.size();
        landmark_item.color.a = 1.0;
        landmark_item.color.r = 0;
        landmark_item.color.g = 255;
        landmark_item.color.b = 0;
        landmark_item.lifetime = ros::Duration(0.02);

        landmark_poses_list.markers.push_back(landmark_item);
        //log(Info, "landmark publish!");
#endif

        if (reflector_LandMarkList.landmarks.size() != 0)
        {
            reflector_landmark_.publish(reflector_LandMarkList);

#ifdef VISUAL_RVIZ
            //log(Info, "landmark_poses_list size " + std::to_string(landmark_poses_list.markers.size()));
            landmark_poses_list_publisher_.publish(landmark_poses_list);
#endif
        }

        std::array<double, 2> param = {laser_pose.getOrigin().getX(),
                                       laser_pose.getOrigin().getY()};
        std::array<double, 2> origin = param;

        //check find landmark can be use?
        Point_pos current_robot_pos(0.0, 0.0);

        std::vector<Point_pos> Polygon_pos;
        //std::cout << "---------------------------------------" <<std::endl;
        for (auto it : global_match_sorted_data)
        {
            Point_pos landmark_pos(it.second.center_x, it.second.center_y);
            Polygon_pos.push_back(landmark_pos);
            /*
            std::cout << "match data yaw " << it.first << std::endl;
            std::cout << "match data x y " << it.second.center_x << " "
                                           << it.second.center_y << std::endl;
                                           */
        }

        /*
        std::cout << "current_robot_pos data x y " << current_robot_pos.x << " "
                  << current_robot_pos.y << std::endl;
                  */

        int nCount = Polygon_pos.size();
        bool isInside = PtInPolygon(current_robot_pos, Polygon_pos, nCount);

        if (can_use_reflector_localization == true &&
            isInside == true &&
            global_match_data.size() >= 3)
        {

            //std::cout << "can_use_reflector_localization is true" << std::endl;
            //calculate the global robot pose;
            ceres::Problem::Options problem_options;
            ceres::Problem problem(problem_options);

            //std::cout << "param origin xy " << laser_pose.getOrigin().getX() << " "
            //                                << laser_pose.getOrigin().getY() << "----> ";

            //problem.AddParameterBlock(param.data(), 2);
            //std::cout << "Peak.ding match " << global_match_data.size() << " id -> ";
            int use_count = 0;
            visualization_msgs::MarkerArray current_landmark_list;
            for (auto item : global_match_data)
            {
                //if (use_count >= 4)
                //    break;

                //   CostFunction* cost_function
                //       = new AutoDiffCostFunction<MyScalarCostFunctor, 1, 2, 2>(
                //            new MyScalarCostFunctor(1.0));             ^  ^  ^
                //                                                       |  |  |
                //                            Dimension of residual -----+  |  |
                //                            Dimension of x ---------------+  |
                //                            Dimension of y ------------------+
                //std::cout << item.second << " ";
                //range
                double range = hypot(item.first.center_x, item.first.center_y);
                landmark_relfector object = global_reflectors[item.second];

                /*
                std::cout << "Peak.ding id: " << item.first << " "
                          << "range: " << range << std::endl;
                          */

                ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<ReflectorCostFunctor, 1, 2>(
                    new ReflectorCostFunctor(range, object));

                problem.AddResidualBlock(cost_function, new ceres::HuberLoss(10), param.data());

                use_count++;

                //show landmark pose

                visualization_msgs::Marker landmark_item;
                landmark_item.pose.position.x = object.x;
                landmark_item.pose.position.y = object.y;
                landmark_item.pose.orientation.x = 0.0;
                landmark_item.pose.orientation.y = 0.0;
                landmark_item.pose.orientation.z = 0.0;
                landmark_item.pose.orientation.w = 1.0;
                landmark_item.header.frame_id = "map";
                landmark_item.header.stamp = scan->header.stamp; //ros::Time::now();
                landmark_item.scale.x = 0.15;                    //kLandmarkMarkerScale * 1.5;
                landmark_item.scale.y = 0.15;                    //kLandmarkMarkerScale * 1.5;
                landmark_item.scale.z = 0.15;                    //kLandmarkMarkerScale * 1.5;
                landmark_item.type = visualization_msgs::Marker::SPHERE;
                landmark_item.ns = "Landmarks";
                landmark_item.id = item.second; //bug fixed
                landmark_item.color.a = 1;
                landmark_item.color.r = 255;
                landmark_item.color.g = 255;
                landmark_item.color.b = 0;
                landmark_item.lifetime = ros::Duration(0.05);

                current_landmark_list.markers.push_back(landmark_item);
            }
            //std::cout << std::endl;

            /*
            for(auto item : current_landmark_list.markers)
            {
                std::cout << "Peak.ding id x y" << item.id << " "
                                                << item.pose.position.x << " "
                                                << item.pose.position.y << std::endl; 
            }
            */
            //publish current detect landmarks
            current_landmark_list_pub_.publish(current_landmark_list);

            ceres::Solver::Options options;
            //options.linear_solver_type = ceres::DENSE_QR;
            //options.minimizer_progress_to_stdout = true;
            options.use_nonmonotonic_steps = false;
            options.max_num_iterations = 50;
            options.num_threads = 2;

            ceres::Solver::Summary summary;
            //std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
            ceres::Solve(options, &problem, &summary);
            //std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
            //std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
            //std::cout << "solve time cost = " << time_used.count() << " seconds. " << std::endl;

            //std::cout << summary.BriefReport() << std::endl;
            //std::cout << "estimated xy = " << param[0] << " " << param[1] << std::endl;

            //calculate the yaw of reflector localization
            //std::cout << "------------------------------------" << std::endl;
            double yaw_x_sum = 0;
            double yaw_y_sum = 0;
            int yaw_count = 0;
            for (auto item : global_match_data)
            {
                double lidar_yaw = item.first.center_yaw;
                double map_yaw = atan2(param[1] - global_reflectors[item.second].y,
                                       param[0] - global_reflectors[item.second].x);

                /*
                std::cout << "lidar_yaw " << lidar_yaw * 180.0 / M_PI << " "
                          << "map_yaw " << map_yaw * 180.0 / M_PI << std::endl;
                */

                double value = (map_yaw - lidar_yaw);

                //std::cout << "value " << value * 180.0/M_PI << std::endl;
                if (value > M_PI)
                    value -= 2 * M_PI;
                if (value < -M_PI)
                    value += 2 * M_PI;

                //transfer to x、y because -180/180 problem
                double x = 1 * cos(value);
                double y = 1 * sin(value);

                //std::cout << "x, y " << x << " " << y << std::endl;

                yaw_x_sum += x;
                yaw_y_sum += y;
                //std::cout << "yaw_sum " << yaw_sum << std::endl;
                yaw_count++;
            }

            double yaw_x_average = yaw_x_sum / yaw_count;
            double yaw_y_average = yaw_y_sum / yaw_count;

            //std::cout << "yaw_y_sum " << yaw_y_sum << std::endl;
            //std::cout << "yaw_x_sum " << yaw_x_sum << std::endl;
            //std::cout << "yaw_x_average " << yaw_x_average << std::endl;
            //std::cout << "yaw_y_average " << yaw_y_average << std::endl;

            double lidar_yaw_in_map = atan2(yaw_y_average, yaw_x_average) - M_PI;

            if (lidar_yaw_in_map > M_PI)
                lidar_yaw_in_map -= 2 * M_PI;
            if (lidar_yaw_in_map < -M_PI)
                lidar_yaw_in_map += 2 * M_PI;
            //std::cout << "yaw_count " << yaw_count << std::endl;
            //std::cout << "Peak.ding lidar_yaw_in_map " << lidar_yaw_in_map * 180.0/M_PI   << std::endl;
            //std::cout << "robot_pose_yaw " << tf::getYaw(laser_pose.getRotation()) * 180.0/M_PI << std::endl;

            //检测与全局位姿的角度差异
            /*
            double angle_change_value = (lidar_yaw_in_map - tf::getYaw(laser_pose.getRotation())) * 180.0 / M_PI;
            if (fabs(angle_change_value) > max_change_angle)
            {
                double yaw_x_sum_test, yaw_y_sum_test;
                int yaw_count_test = 0;

                max_change_angle = fabs(angle_change_value);
                std::cout << "max_change_angle " << max_change_angle << std::endl;
                std::cout << "Peak.ding lidar_yaw_in_map " << lidar_yaw_in_map * 180.0 / M_PI << std::endl;
                std::cout << "robot_pose_yaw " << tf::getYaw(laser_pose.getRotation()) * 180.0 / M_PI << std::endl;
            }
            */

            //show lidar pose
            //显示激光雷达的位置
            geometry_msgs::PointStamped point;
            point.header.stamp = scan->header.stamp;

            point.header.frame_id = "map";

            point.point.x = param[0];
            point.point.y = param[1];
            point.point.z = 0;

            reflector_points_.publish(point);

            //发布激光里程计
            /*
            nav_msgs::Odometry laserOdometryROS;
            laserOdometryROS.header.stamp = scan->header.stamp;
            laserOdometryROS.header.frame_id = "map";
            laserOdometryROS.child_frame_id = "odom_mapping";
            laserOdometryROS.pose.pose.position.x = param[0];
            laserOdometryROS.pose.pose.position.y = param[1];
            laserOdometryROS.pose.pose.position.z = 0;
            laserOdometryROS.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
            pubLaserOdometryGlobal.publish(laserOdometryROS);
            */

            //发布矫正和未矫正路径
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.stamp = scan->header.stamp;
            pose_stamped.header.frame_id = "map";
            pose_stamped.pose.position.x = origin[0];
            pose_stamped.pose.position.y = origin[1];
            pose_stamped.pose.position.z = 0;
            tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, 0);
            pose_stamped.pose.orientation.x = q.x();
            pose_stamped.pose.orientation.y = q.y();
            pose_stamped.pose.orientation.z = q.z();
            pose_stamped.pose.orientation.w = q.w();

            lidar_uncorrected_path_.poses.push_back(pose_stamped);
            pose_stamped.pose.position.x = param[0];
            pose_stamped.pose.position.y = param[1];
            lidar_corrected_path_.poses.push_back(pose_stamped);

            lidar_uncorrected_path_.header.stamp = scan->header.stamp;
            lidar_uncorrected_path_.header.frame_id = "map";
            lidar_uncorrected_pub_.publish(lidar_uncorrected_path_);

            lidar_corrected_path_.header.stamp = scan->header.stamp;
            lidar_corrected_path_.header.frame_id = "map";
            lidar_corrected_pub_.publish(lidar_corrected_path_);

            //检测与全局位姿的xy差异
            if (fabs(param[0] - origin[0]) > max_change_x)
            {
                max_change_x = fabs(param[0] - origin[0]);

                /*
                std::cout << "Peak.ding test max_change_x: " << max_change_x << " "
                          << "origin_x: " << origin[0] << " "
                          << "optimize_x: " << param[0] << std::endl; 
                          */
            }

            if (fabs(param[1] - origin[1]) > max_change_y)
            {
                max_change_y = fabs(param[1] - origin[1]);

                /*
                std::cout << "Peak.ding test max_change_y: " << max_change_y << " "
                          << "origin_y: " << origin[1] << " "
                          << "optimize_y: " << param[1] << std::endl;
                          */
            }

            /*
            std::cout << "Peak.ding test max_change_x: " << max_change_x << " "
                      << "max_change_y: " << max_change_y << std::endl;
                      */

            /*
            if (param[0] > max_x) max_x = param[0];
            if (param[0] < min_x) min_x = param[0];
            if (param[1] > max_y) max_y = param[1];
            if (param[1] < min_y) min_y = param[1];

            std::cout << "Peak.ding test param max_change_x: " << max_x-min_x << " "
                      << "param max_change_y: " << max_y-min_y << std::endl;
                      */

            //绘制差异大于0.02的标记
            double dx = fabs(origin[0] - param[0]);
            double dy = fabs(origin[1] - param[1]);
            double distance = sqrt(std::pow(dx, 2) + std::pow(dy, 2));
            //if (sqrt(std::pow(max_change_x, 2) + std::pow(max_change_y, 2)) > max_change)
            if (distance > 0.02)
            {
                max_change = sqrt(std::pow(max_change_x, 2) + std::pow(max_change_y, 2));

                // loop nodes
                //visualization_msgs::Marker markerNode;
                markerNode.header.frame_id = "map";
                markerNode.header.stamp = scan->header.stamp;
                markerNode.action = visualization_msgs::Marker::ADD;
                markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
                markerNode.ns = "error_nodes";
                markerNode.id = 0;
                markerNode.pose.orientation.w = 1;
                markerNode.scale.x = 0.01;
                markerNode.scale.y = 0.01;
                markerNode.scale.z = 0.01;
                markerNode.color.r = 1; //0
                markerNode.color.g = 0; //0.8
                markerNode.color.b = 0; //1
                markerNode.color.a = 1;
                // loop edges
                //visualization_msgs::Marker markerEdge;
                markerEdge.header.frame_id = "map";
                markerEdge.header.stamp = scan->header.stamp;
                markerEdge.action = visualization_msgs::Marker::ADD;
                markerEdge.type = visualization_msgs::Marker::LINE_LIST;
                markerEdge.ns = "loop_edges";
                markerEdge.id = 1;
                markerEdge.pose.orientation.w = 1;
                markerEdge.scale.x = 0.005;
                markerEdge.color.r = 0.9;
                markerEdge.color.g = 0.9;
                markerEdge.color.b = 0;
                markerEdge.color.a = 1;

                geometry_msgs::Point p;
                p.x = origin[0];
                p.y = origin[1];
                p.z = 0;
                markerNode.points.push_back(p);
                markerEdge.points.push_back(p);
                p.x = param[0];
                p.y = param[1];
                p.z = 0;
                markerNode.points.push_back(p);
                markerEdge.points.push_back(p);

                error_edge_markers.markers.push_back(markerNode);
                error_edge_markers.markers.push_back(markerEdge);

                /*
                if(error_edge_markers.markers.size() > 10)
                {
                   error_edge_markers.markers.clear(); 
                }
                */

                error_edge_pub_.publish(error_edge_markers);
            }

            tf::Pose current_laser_pose;
            //getLaserPose(current_laser_pose, ros::Time(0.), tf_);
            getLaserPose2(current_laser_pose, ros::Time(0.));
            current_laser_pose = base_link_to_map * base_link_to_lidar;

            tf::Quaternion correct_lidar_quaternion = tf::createQuaternionFromYaw(lidar_yaw_in_map);
            tf::Vector3 correct_lidar_point(param[0], param[1], 0);

            tf::Pose current_correct_lidar_pose =
                tf::Pose(correct_lidar_quaternion, correct_lidar_point) *
                (laser_pose.inverse() * current_laser_pose);

            tf::Pose current_correct_base_link = current_correct_lidar_pose *
                                                 base_link_to_lidar.inverse();

            geometry_msgs::TransformStamped stamped_transform;

            stamped_transform.header.stamp = ros::Time::now();
            stamped_transform.header.frame_id = "map";
            stamped_transform.child_frame_id = "correct_base_link";

            geometry_msgs::Transform transform;
            transform.translation.x = current_correct_base_link.getOrigin().getX();
            transform.translation.y = current_correct_base_link.getOrigin().getY();
            transform.translation.z = 0;
            tf::Quaternion temp = current_correct_base_link.getRotation();
            transform.rotation.w = temp.getW();
            transform.rotation.x = temp.getX();
            transform.rotation.y = temp.getY();
            transform.rotation.z = temp.getZ();

            stamped_transform.transform = transform;

            tf_broadcaster_.sendTransform(stamped_transform);

            // publish robot pose
            cartographer_ros_msgs::RobotPose current_pose;

            current_pose.robot_pose.position.x = current_correct_base_link.getOrigin().getX();
            current_pose.robot_pose.position.y = current_correct_base_link.getOrigin().getY();
            current_pose.robot_pose.position.z = 0;
            current_pose.robot_pose.orientation.w = temp.getW();
            current_pose.robot_pose.orientation.x = temp.getX();
            current_pose.robot_pose.orientation.y = temp.getY();
            current_pose.robot_pose.orientation.z = temp.getZ();
            current_pose.covariance_score = 0.6;
            current_pose.current_trajectory = trajectory_id;
            current_pose.last_update_duration = 0.1;
            current_pose.last_update_pose = current_pose.robot_pose;

            robot_pose_pub_.publish(current_pose);
        }
    }

    void QNode::getGlobalReflector(const visualization_msgs::MarkerArrayConstPtr &data)
    {
        for (int i = 0; i < data->markers.size(); i++)
        {
            landmark_relfector it;
            int id = data->markers[i].id;
            it.x = data->markers[i].pose.position.x;
            it.y = data->markers[i].pose.position.y;
            it.z = data->markers[i].pose.position.z;

            global_reflectors[id] = it;

            /*
            std::cout << "peak.ding id " << id << " "
                      << "it.x " << it.x << " "
                      << "it.y " << it.y << " "
                      << "it.z " << it.z << " " << std::endl;
            */
        }

        //std::cout << "Peak.ding global_reflectors size " << global_reflectors.size() << std::endl;
    }

    /**
     * @name getLaserPose()
     * @brief 得到机器人在里程计坐标系中的位姿tf::Pose
     *        得到dt时刻激光雷达在odom坐标系的位姿
     * @param odom_pos  机器人的位姿
     * @param dt        dt时刻
     * @param tf_
    */
    bool QNode::getLaserPose(tf::Stamped<tf::Pose> &laser_pose,
                             ros::Time dt,
                             tf::TransformListener *tf_)
    {
        //below is not used
        laser_pose.setIdentity();

        tf::Stamped<tf::Pose> robot_pose;
        robot_pose.setIdentity();
        robot_pose.frame_id_ = lidar_frame_;
        robot_pose.stamp_ = dt; //设置为ros::Time()表示返回最近的转换关系

        /*
        // get the global pose of the robot
        try
        {

            if (!tf_->waitForTransform(map_frame_, lidar_frame_, dt, ros::Duration(0.5))) // 0.15s 的时间可以修改
            {
                ROS_ERROR("LidarMotion-Can not Wait Transform()");
                return false;
            }

            tf_->transformPose(map_frame_, robot_pose, laser_pose);
        }
        catch (tf::LookupException &ex)
        {
            ROS_ERROR("LidarMotion: No Transform available Error looking up robot pose: %s\n", ex.what());
            return false;
        }
        catch (tf::ConnectivityException &ex)
        {
            ROS_ERROR("LidarMotion: Connectivity Error looking up looking up robot pose: %s\n", ex.what());
            return false;
        }
        catch (tf::ExtrapolationException &ex)
        {
            ROS_ERROR("LidarMotion: Extrapolation Error looking up looking up robot pose: %s\n", ex.what());
            return false;
        }

        return true;
        */

        try
        {
            geometry_msgs::TransformStamped lidar_to_map =
                tf_buffer_node_->lookupTransform(map_frame_, lidar_frame_,
                                                 ros::Time(0.), ros::Duration(0.5));

            //laser_pose = tf::StampedTransform

            tf::Quaternion quaternion_(lidar_to_map.transform.rotation.x,
                                       lidar_to_map.transform.rotation.y,
                                       lidar_to_map.transform.rotation.z,
                                       lidar_to_map.transform.rotation.w);
            tf::Vector3 origin_(lidar_to_map.transform.translation.x,
                                lidar_to_map.transform.translation.y,
                                lidar_to_map.transform.translation.z);

            tf::Pose pose(quaternion_, origin_);
        }
        catch (const tf2::TransformException &ex)
        {
            LOG(WARNING) << ex.what();
            return false;
        }

        return true;
    }

    /**
     * @name getLaserPose()
     * @brief 得到机器人在里程计坐标系中的位姿tf::Pose
     *        得到dt时刻激光雷达在odom坐标系的位姿
     * @param laser_pose  输出机器人的位姿
     * @param dt dt时刻
    */

    bool QNode::getLaserPose2(tf::Pose &laser_pose, ros::Time dt)
    {
        try
        {
            geometry_msgs::TransformStamped lidar_to_map =
                tf_buffer_node_->lookupTransform(map_frame_, lidar_frame_,
                                                 dt, ros::Duration(0.5));

            //laser_pose = tf::StampedTransform

            tf::Quaternion quaternion_(lidar_to_map.transform.rotation.x,
                                       lidar_to_map.transform.rotation.y,
                                       lidar_to_map.transform.rotation.z,
                                       lidar_to_map.transform.rotation.w);
            tf::Vector3 origin_(lidar_to_map.transform.translation.x,
                                lidar_to_map.transform.translation.y,
                                lidar_to_map.transform.translation.z);
            //set data
            laser_pose.setRotation(quaternion_);
            laser_pose.setOrigin(origin_);
        }
        catch (const tf2::TransformException &ex)
        {
            LOG(WARNING) << ex.what();
            return false;
        }

        return true;
    }

    void QNode::getRobotPose(const cartographer_ros_msgs::RobotPoseConstPtr &data)
    {
        //std::cout << "Peak.ding getRobotPose" << std::endl;

        tf::Quaternion q_tem;
        q_tem.setX(data->robot_pose.orientation.x);
        q_tem.setY(data->robot_pose.orientation.y);
        q_tem.setZ(data->robot_pose.orientation.z);
        q_tem.setW(data->robot_pose.orientation.w);

        robot_pose_yaw = tf::getYaw(q_tem);
        //std::cout << "robot_yaw " << robot_yaw*180.0/M_PI << std::endl;

        tf::Vector3 p_tem(data->robot_pose.position.x,
                          data->robot_pose.position.y,
                          data->robot_pose.position.z);

        base_link_to_map = tf::Transform(q_tem, p_tem);

        if (data->covariance_score > 0.0)
        {
            can_use_reflector_localization = true;
        }
        else
        {
            can_use_reflector_localization = false;
        }

        /*
        std::cout << "Peak.ding base_link_to_map " 
                  << base_link_to_map.getOrigin().getX() << " "
                  << base_link_to_map.getOrigin().getY() << " "
                  << base_link_to_map.getOrigin().getZ() << " " << std::endl;
                  */

        trajectory_id = data->current_trajectory;
    }

    bool QNode::selectProperData(std::map<Reflector_pos, int /*id*/> &global_match_data,
                                 std::vector<int /*id*/> proper_data)
    {
        /*
        int count = 0;
        for (auto f : global_match_data)
        {
            double base_yaw = f.first.center_yaw;
            double sum_center_yaw = 0.0;
            proper_data.push_back(f.second);

            for (auto s : global_match_data)
            {
                if (s.second != f.second) //id is different
                {
                    double d_value = s.first.center_yaw - base_yaw;
                    if (fabs(d_value) > M_PI)
                    {
                        d_value = fabs(d_value) - M_PI;
                    }

                    sum_center_yaw += d_value;
                    count++;

                    if (sum_center_yaw >)
                }
            }
        }
        */

        return false;
    }

    //作用：判断点是否在多边形内
    //p指目标点， ptPolygon指多边形的点集合， nCount指多边形的边数
    bool QNode::PtInPolygon(Point_pos p, std::vector<Point_pos> &ptPolygon, int nCount)
    {
        // 交点个数
        int nCross = 0;
        for (int i = 0; i < nCount; i++)
        {
            Point_pos p1 = ptPolygon[i];
            Point_pos p2 = ptPolygon[(i + 1) % nCount]; // 点P1与P2形成连线

            if (p1.y == p2.y)
                continue;
            if (p.y < std::min(p1.y, p2.y))
                continue;
            if (p.y >= std::max(p1.y, p2.y))
                continue;
            // 求交点的x坐标（由直线两点式方程转化而来）

            double x = (double)(p.y - p1.y) * (double)(p2.x - p1.x) / (double)(p2.y - p1.y) + p1.x;

            // 只统计p1p2与p向右射线的交点
            if (x > p.x)
            {
                nCross++;
            }
        }

        // 交点为偶数，点在多边形之外
        // 交点为奇数，点在多边形之内
        if ((nCross % 2) == 1)
        {
            //g_proj_log.ShowInfo("点在区域内");
            return true;
        }
        else
        {
            //g_proj_log.ShowInfo("点在区域外");
            return false;
        }
    }

    void QNode::getSystemWoringState(const cartographer_ros_msgs::WorkingStateConstPtr &data)
    {
        system_working_state_ = data->state;
    }

} // namespace qt_test
