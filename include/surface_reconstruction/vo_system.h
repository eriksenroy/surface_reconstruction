#ifndef __VO_SYSTEM_H
#define __VO_SYSTEM_H


#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <cstdio>
#include <message_filters/subscriber.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
using namespace std;

// TIC - TOC
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>
#include <math.h>


#include <boost/thread/thread.hpp>
#include <iostream>
#include <stdio.h>
#include <boost/filesystem.hpp>


#include <ros/ros.h>
//#include <pcl_ros/point_cloud.h>
//#include <pcl/point_types.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include <visualization_msgs/Marker.h>

/////ROS IMAGE SUBSCRIBER
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <surface_reconstruction/superpixel.h>
#include <surface_reconstruction/DenseMapping.h>
#include <surface_reconstruction/tracking.h>
#include <surface_reconstruction/mapping.h>


class SemiDenseTracking;

class vo_system
{
public:
    vo_system();


    void imgcb(const sensor_msgs::Image::ConstPtr& msg,const nav_msgs::Odometry::ConstPtr& msg2); // ,const geometry_msgs::PoseStamped::ConstPtr& msg2


    DenseMapping dense_mapper;
    SemiDenseTracking semidense_tracker;
    SemiDenseMapping semidense_mapper;
    MapShared Map;
    Imagenes images,images_previous_keyframe;

    int cont_frames;
    double stamps;
    cv::Mat image_frame,image_frame_aux;
    double depth_stamps;
    ros::Time current_time,stamps_ros;

    ros::NodeHandle nh;
    image_transport::Subscriber sub1;
    image_transport::Publisher pub_image;




    ros::Publisher odom_pub;
    ros::Subscriber chat;
//    ros::Subscriber ground_pose;

    ros::Publisher pub_cloud;
    ros::Publisher pub_poses;
    ros::Publisher vis_pub;


    /// inserted by roy


    //Roy inserting

//    message_filters::Subscriber<sensor_msgs::Image>* m_imageSubscriber;     //(nh,camera_path,1000);
//    message_filters::Subscriber<geometry_msgs::PoseStamped>* m_poseMsgFilterSubscriber;       //(nh,"/groundtruth",1000);
//    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped,sensor_msgs::Image> SyncImageWithPose;
//    message_filters::Synchronizer<SyncImageWithPose>* m_imagePoseSynchronizer;
//
//    typedef image_transport::SubscriberFilter ImageSubscriber;
//    typedef message_filters::Subscriber<geometry_msgs::PoseStamped> PoseSubscriber2;
//    ImageSubscriber subbing1;
//    PoseSubscriber2 groundpose;
//    subbing1(it_,"/camera/rgb/image_color",1);
//
//
//
//    sync(MySyncPolicy(10),sub1,ground_pose)
//    {
//        sync.registerCallback( boost::bind( &vo_system::imgcb, this, _1, _2 ) );
//    }

};
#endif