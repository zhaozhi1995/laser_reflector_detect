//#include <QApplication>
//#include <../include/ros_qt5/mainwindow.h>

#include <../include/ros_qt5/qnode.h>

int main(int argc, char *argv[])
{
    //QApplication a(argc,argv);
    //MainWindow w(argc,argv);
    //w.show();
    //return a.exec();
    ROS_INFO("ros master start init");
    ros::init(argc, argv, "LaserReflectorDetect");
    ROS_INFO("ros master finish init");

    constexpr double kTfBufferCacheTimeInSeconds = 10.;
    tf2_ros::Buffer tf_buffer{::ros::Duration(kTfBufferCacheTimeInSeconds)};
    tf2_ros::TransformListener tf(tf_buffer);
    
    qt_test::QNode qnode(ros::NodeHandle(""),&tf_buffer);
    qnode.init();
    
    ros::spin();
    return 0;
}
