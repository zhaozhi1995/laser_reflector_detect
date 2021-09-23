/**
 * @file /include/qt_test/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef qt_test_QNODE_HPP_
#define qt_test_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/package.h>
#endif
#include <string>
//#include <QThread>
#include <QStringListModel>

#include <sensor_msgs/LaserScan.h>
#include <cartographer_ros_msgs/LandmarkEntry.h>
#include <cartographer_ros_msgs/LandmarkList.h>
#include <geometry_msgs/PoseStamped.h>

#include <../include/ros_qt5/laser_processor.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <visualization_msgs/MarkerArray.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <ceres/ceres.h>

#include <chrono>

#include <cartographer_ros_msgs/RobotPose.h>
#include <nav_msgs/Path.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <cartographer_ros_msgs/WorkingState.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qt_test {


struct Point_pos {
    double x;
    double y;

    Point_pos(double x_, double y_)
    {
        this->x = x_;
        this->y = y_;
    }
};


struct Reflector_pos {

    double center_x;
    double center_y;
    double center_yaw;

    bool operator <(const Reflector_pos& other) const {
        double this_distance = hypot(center_x,center_y);
        double other_distance = hypot(other.center_x,other.center_y);

        return this_distance < other_distance;
    }
};

struct landmark_relfector
{
    double x;
    double y;
    double z;
};

/*****************************************************************************
** Class
*****************************************************************************/

class ReflectorCostFunctor{

   public:
   ReflectorCostFunctor(double range, landmark_relfector object)
   : measure_ramge(range),
     reflector(object){}

   template <typename T>
   bool operator()(const T* x, T* e) const{
       T d_x = x[0]-T(reflector.x);
       T d_y = x[1]-T(reflector.y);
       //e[0] = T(measure_ramge) - T(ceres::sqrt(ceres::pow(T(d_x),2)+ceres::pow(T(d_y),2)));
       e[0] = T(measure_ramge) - sqrt(T(d_x)*T(d_x) + T(d_y)*T(d_y));
       return true;
   }

   private:
   double measure_ramge;
   landmark_relfector reflector;

};


class QNode {
    //Q_OBJECT
public:
	QNode(ros::NodeHandle node_handle,tf2_ros::Buffer* buffer);
	virtual ~QNode();
	bool init();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

    bool get_IK(float pos[6], float joints[6]);

    int round_int(double val);

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();

private:
	int init_argc;
	char** init_argv;
    QStringListModel logging_model;

    ros::Subscriber scan_sub_;
    ros::Subscriber global_reflector_pos_sub_;
    ros::Subscriber robot_pose_sub_;
    ros::Subscriber system_working_state_sub_;

    void laserProcess(const sensor_msgs::LaserScanConstPtr& scan);
    void getGlobalReflector(const visualization_msgs::MarkerArrayConstPtr& data);
    void getRobotPose(const cartographer_ros_msgs::RobotPoseConstPtr& data);
    bool selectProperData(std::map<Reflector_pos,int/*id*/> &global_match_data,std::vector<int/*id*/> proper_data);
    bool PtInPolygon (Point_pos p, std::vector<Point_pos>& ptPolygon, int nCount);  
    void getSystemWoringState(const cartographer_ros_msgs::WorkingStateConstPtr& data);

    sensor_msgs::LaserScan scan_;

    ros::Publisher reflector_points_;
    ros::Publisher reflector_landmark_;
    ros::Publisher current_landmark_list_pub_;
    ros::Publisher reflector_localization_pos_pub_;
    ros::Publisher lidar_uncorrected_pub_;
    ros::Publisher lidar_corrected_pub_;
    ros::Publisher error_edge_pub_;
    ros::Publisher robot_pose_pub_;

    //double angle_error_scale = 15;
    //double distance_error_scale = 15;
    double reflector_radius = 0.0 / 2.0;
    int min_reflector_sample_count = 8;
    int min_reflector_intensity = 1000;

    double kLandmarkMarkerScale = 0.05;

    double reflector_combined_length = 0.5;
    int reflector_combination_mode = 0;

    double angle_error;
    double distance_error;
    int last_index = 0;

    std::map<int, landmark_relfector> global_reflectors;
    std::string lidar_frame_;
    std::string map_frame_;

    bool getLaserPose(tf::Stamped<tf::Pose> &laser_pose,
                      ros::Time dt,
                      tf::TransformListener * tf_);

    bool getLaserPose2(tf::Pose &laser_pose, ros::Time dt);

    tf::TransformListener * tf_;

    double match_distance_acceped;
    //rearrange by first
    //std::map<double/*range*/,int/*id*/> global_match_data;
    std::map<Reflector_pos,int/*id*/> global_match_data;
    std::map<double/*yaw*/,Reflector_pos> global_match_sorted_data;

    tf::Transform base_link_to_map;
    tf::Transform base_link_to_lidar;

    bool can_use_reflector_localization = false;

    //test localization error
    double max_change_y = 0;
    double max_change_x = 0;
    double max_change_angle = 0;
    double max_change = 0;

    double max_x = -10;
    double min_x = 10;
    double max_y = -10;
    double min_y = 10;

    ros::NodeHandle n;

    nav_msgs::Path lidar_uncorrected_path_;
    nav_msgs::Path lidar_corrected_path_;
    visualization_msgs::MarkerArray error_edge_markers;
    visualization_msgs::Marker markerNode;
    visualization_msgs::Marker markerEdge;

    double robot_pose_yaw;

    tf2_ros::TransformBroadcaster tf_broadcaster_;

    std::string trajectory_id;

      //test
    tf2_ros::Buffer* tf_buffer_node_;

    //
    std::string system_working_state_;
    
};

}  // namespace qt_test

#endif /* qt_test_QNODE_HPP_ */
