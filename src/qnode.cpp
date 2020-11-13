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


#define VISUAL_RVIZ


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qt_test {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
    ROS_INFO("ros master start init");
	ros::init(init_argc,init_argv,"qt_test");
    ROS_INFO("ros master finish init");
	if ( ! ros::master::check() ) {
        ROS_INFO("ros master fail");
		return false;
	}
    else
    {
        ROS_INFO("ros master success");
    }
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);

    scan_sub_ = n.subscribe("scan",10, &QNode::callback, this);

    debug_points_ = n.advertise<sensor_msgs::PointCloud2>("dock_points",10);

    //reflector_points_ = n.advertise<geometry_msgs::PointStamped>("reflector_points",10);

    reflector_landmark_ = n.advertise<cartographer_ros_msgs::LandmarkList>("landmark",1);

    landmark_poses_list_publisher_ = n.advertise<::visualization_msgs::MarkerArray>("laser_landmark_list_test", 1);

	start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"qt_test");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
	start();
	return true;
}

void QNode::run() {
    ros::Rate loop_rate(30);
	int count = 0;
	while ( ros::ok() ) {
        /*
		std_msgs::String msg;
		std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();
		chatter_publisher.publish(msg);
		log(Info,std::string("I sent: ")+msg.data);
        */
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

    void QNode::callback(const sensor_msgs::LaserScanConstPtr& scan)
    {
        //scan_ = scan;

        //laser_processor::SampleSet* cluster = new laser_processor::SampleSet;

        std::vector<laser_processor::Sample*> scan_filter;

        for(uint32_t i = 0; i < scan->ranges.size(); i++)
        {
            laser_processor::Sample* s = laser_processor::Sample::Extract(i,*scan);

            if(s != NULL)
            {
                //cluster->insert(s);
                scan_filter.push_back(s);
            }
        }

        if(scan_filter.size() == 0) return;

        //get the center of reflector
        double center_x , center_y, center_yaw , center_count;
        std::vector<Reflector_pos> reflectors_;
        double last_item_x, last_item_y;
        bool is_mark_start = true;

        //for(laser_processor::SampleSet::iterator p = cluster->begin(); p != cluster->end(); p++)
        for(int i=0 ; i < scan_filter.size(); i++)
        {
            is_mark_start = true;

            for(int j=i; j < scan_filter.size(); j++)
            {
                laser_processor::Sample* p = scan_filter[j];

                double item_x = p->x;
                double item_y = p->y;
                double item_range = p->range;

                if(is_mark_start == true)
                {
                    center_x = p->x;
                    center_y = p->y;
                    center_count = 1;
                    is_mark_start = false;
                }
                else
                {
                    double d_x = item_x - last_item_x;
                    double d_y = item_y - last_item_y;
                    double d_xy = sqrt(pow(d_x,2) + pow(d_y,2));
                    double angle_internal = d_xy/item_range;
                    double distance_internal = scan->angle_increment * item_range;

                    angle_error = scan->angle_increment * angle_error_scale;
                    distance_error = distance_internal * distance_error_scale;

                    //check the scan point not the last one
                    if((angle_internal < angle_error || d_xy < distance_error)
                            && (j != scan_filter.size() - 1))
                    {
                        center_x += p->x;
                        center_y += p->y;
                        center_count++;
                    }
                    else
                    {
                        if(center_count > min_reflector_sample_count)
                        {
                            center_x /= center_count;
                            center_y /= center_count;
                            center_yaw = atan(center_y / center_x);

                            center_x += cos(center_yaw) * reflector_radius;
                            center_y += sin(center_yaw) * reflector_radius;

                            Reflector_pos item;
                            item.center_x = center_x;
                            item.center_y = center_y;
                            item.center_yaw = center_yaw;
                            reflectors_.push_back(item);
                        }

                        center_x = 0.0;
                        center_y = 0.0;
                        center_count = 0;
                        i = j-1;
                        break;

                    }

                }

                last_item_x = item_x;
                last_item_y = item_y;

            }

        }

        visualization_msgs::MarkerArray landmark_poses_list;
        cartographer_ros_msgs::LandmarkList reflector_LandMarkList;

        //log(Info,"peak check the size " + std::to_string((reflectors_.size())));

        for(int i = 0 ; i < reflectors_.size(); i++)
        {
            Reflector_pos item = reflectors_[i];

            //publish the center points
            /*
            geometry_msgs::PointStamped point;
            point.header.stamp = scan->header.stamp;
            point.header.frame_id = "car_laser";

            point.point.x = item.center_x;
            point.point.y = item.center_y;
            point.point.z = 0;

            reflector_points_.publish(point);
            */

            double distance = sqrt(pow(item.center_x,2) + pow(item.center_y,2));
            if(distance < 1.5) continue;
            //
            cartographer_ros_msgs::LandmarkEntry reflector_LandMarkEntry;
            reflector_LandMarkEntry.id = "unknown";
            reflector_LandMarkEntry.tracking_from_landmark_transform.position.x = item.center_x;
            reflector_LandMarkEntry.tracking_from_landmark_transform.position.y = item.center_y;
            reflector_LandMarkEntry.tracking_from_landmark_transform.position.z = 0.0;
            tf::Quaternion quat = tf::createQuaternionFromYaw(item.center_yaw);
            reflector_LandMarkEntry.tracking_from_landmark_transform.orientation.x = quat.x();
            reflector_LandMarkEntry.tracking_from_landmark_transform.orientation.y = quat.y();
            reflector_LandMarkEntry.tracking_from_landmark_transform.orientation.z = quat.z();
            reflector_LandMarkEntry.tracking_from_landmark_transform.orientation.w = quat.w();
            reflector_LandMarkEntry.translation_weight = 1.0;
            reflector_LandMarkEntry.rotation_weight = 1.0;


            reflector_LandMarkList.header.frame_id = "car_laser";
            reflector_LandMarkList.header.stamp = ros::Time::now();
            reflector_LandMarkList.landmarks.push_back(reflector_LandMarkEntry);


            #ifdef VISUAL_RVIZ
            visualization_msgs::Marker landmark_item;
            landmark_item.pose.position.x = item.center_x;
            landmark_item.pose.position.y = item.center_y;
            landmark_item.pose.orientation.x = quat.x();
            landmark_item.pose.orientation.y = quat.y();
            landmark_item.pose.orientation.z = quat.z();
            landmark_item.pose.orientation.w = quat.w();
            landmark_item.header.frame_id = "car_laser";
            landmark_item.header.stamp = ros::Time::now();
            landmark_item.scale.x = kLandmarkMarkerScale;
            landmark_item.scale.y = kLandmarkMarkerScale;
            landmark_item.scale.z = kLandmarkMarkerScale;
            landmark_item.type = visualization_msgs::Marker::SPHERE;
            landmark_item.ns = "Landmarks";
            landmark_item.id = i;
            landmark_item.color.a = 1.0;
            landmark_item.color.r = 0;
            landmark_item.color.g = 255;
            landmark_item.color.b = 0;
            landmark_item.lifetime = ros::Duration(0.05);

            landmark_poses_list.markers.push_back(landmark_item);
           #endif

           break;

        }
            if(reflector_LandMarkList.landmarks.size() != 0)
            {
                reflector_landmark_.publish(reflector_LandMarkList);

                #ifdef VISUAL_RVIZ
                landmark_poses_list_publisher_.publish(landmark_poses_list);
                #endif
            }


    }

}
