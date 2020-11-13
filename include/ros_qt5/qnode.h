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
#endif
#include <string>
#include <QThread>
#include <QStringListModel>

#include <sensor_msgs/LaserScan.h>
#include <cartographer_ros_msgs/LandmarkEntry.h>
#include <cartographer_ros_msgs/LandmarkList.h>
#include <geometry_msgs/PoseStamped.h>

#include <../include/ros_qt5/laser_processor.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qt_test {


struct Reflector_pos {

    double center_x;
    double center_y;
    double center_yaw;
};


/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();

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

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();

private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
    QStringListModel logging_model;

    ros::Subscriber scan_sub_;

    void callback(const sensor_msgs::LaserScanConstPtr& scan);

    sensor_msgs::LaserScan scan_;

    ros::Publisher debug_points_;
    ros::Publisher reflector_points_;
    ros::Publisher reflector_landmark_;
    ros::Publisher landmark_poses_list_publisher_;

    double angle_error_scale = 15;
    double distance_error_scale = 15;
    double reflector_radius = 0.0 / 2.0;
    int min_reflector_sample_count = 8;

    double kLandmarkMarkerScale = 0.05;

    double angle_error;
    double distance_error;

};

}  // namespace qt_test

#endif /* qt_test_QNODE_HPP_ */
