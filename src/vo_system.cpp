//
// Created by roy on 10/7/20.
//

#include <surface_reconstruction/vo_system.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <ros/package.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
/// sync libraries
#include <image_transport/subscriber_filter.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>

using namespace message_filters;




vo_system::vo_system(){


    ///vo_system launch the three threads, tracking, semidense mapping and dense mapping (3D superpixels)

    cv::FileStorage  fs2( (ros::package::getPath("surface_reconstruction")+"/src/data.yml").c_str(), cv::FileStorage::READ);

    std::string camera_path;
    fs2["camera_path"] >> camera_path;
    cont_frames = 0;
    cout << camera_path<<endl;
    int calculate_superpixels = (int)fs2["calculate_superpixels"];



    image_transport::ImageTransport it(nh);
//    sub1 = it.subscribe(camera_path,1, & vo_system::imgcb,this);
//    chat = nh.subscribe(camera_path, 1, &vo_system::imgcb,this);
///    image_tranzsport::SubscriberFilter sub1(it,camera_path,20);
////    image_transport::SubscriberFilter sub1;


    cout <<"before"<<endl;
//    message_filters::Subscriber<sensor_msgs::Image>* m_imageSubscriber;     //(nh,camera_path,1000);
    image_transport::SubscriberFilter* m_imageSubscriber;
    message_filters::Subscriber<geometry_msgs::PoseStamped>* m_poseMsgFilterSubscriber;       //(nh,"/groundtruth",1000);
    typedef sync_policies::ApproximateTime<sensor_msgs::Image,geometry_msgs::PoseStamped> SyncImageWithPose;
    message_filters::Synchronizer<SyncImageWithPose>* m_imagePoseSynchronizer;

    m_poseMsgFilterSubscriber = new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh,"/groundtruth",1000);
//    m_imageSubscriber = new message_filters::Subscriber<sensor_msgs::Image>(nh,camera_path,1000);
    m_imageSubscriber = new image_transport::SubscriberFilter(it,camera_path,1000);

    m_imagePoseSynchronizer = new message_filters::Synchronizer<SyncImageWithPose>(SyncImageWithPose(1000),*m_imageSubscriber,*m_poseMsgFilterSubscriber);
    //m_imagePoseSynchronizer->setAgePenalty(1.0);
    m_imagePoseSynchronizer->registerCallback(&vo_system::imgcb, this);

//    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh,camera_path,10000);
//    message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub(nh,"/groundtruth",10000);
//    typedef sync_policies::ApproximateTime<sensor_msgs::Image,geometry_msgs::PoseStamped> MySyncPolicy;
//    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10000),image_sub,pose_sub);
//    sync.registerCallback(boost::bind(&vo_system::imgcb,this,_1,_2));

//message_filters::Subscriber<sensor_msgs::Image> image_sub(nh,camera_path,1);
//message_filters::Subscriber<geometry_msgs::PoseStamped> groundtruth_sub(nh,"/groundtruth",1);
//typedef sync_policies::ApproximateTime<sensor_msgs::Image,geometry_msgs::PoseStamped> MySyncPolicy;
//Synchronizer<MySyncPolicy> sync(MySyncPolicy(100),image_sub,groundtruth_sub);
//sync.registerCallback(boost::bind(&vo_system::imgcb,_1,_2));


////    typedef Synchronizer<MySyncPolicy> Sync;
//     Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),subbing1,ground_pose);
////    boost::shared_ptr<Sync> sync;
    cout <<"before sync"<<endl;
//    sync.registerCallback(boost::bind(&vo_system::imgcb,this, _1, _2));
    odom_pub = nh.advertise<nav_msgs::Odometry>("odom1", 50);
    cout <<"after sync"<<endl;
////    ground_pose = nh.subscribe("/groundtruth", 1000);

    /// subcribe filter for ros image
//    image_transport::SubscriberFilter imagesubscriber1;
//    imagesubscriber1.subscribe(it,camera_path,32);
//
//    message_filters::Subscriber<sensor_msgs::Image> image1_sub(nh,camera_path,100);
////    /// subscriber for StampedPose message filters
//    message_filters::Subscriber<geometry_msgs::PointStamped> ground_pose(nh,"/groundtruth",100);
//    typedef sync_policies::ApproximateTime<sensor_msgs::Image,geometry_msgs::PoseStamped> MySyncPolicy;
//    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),image1_sub,ground_pose);
//    sync.registerCallback(boost::bind(&imgcbb, _1, _2));



    /// advertising 3D map and camera poses in rviz
    pub_cloud = nh.advertise<sensor_msgs::PointCloud2> ("surface_reconstruction/map", 1);
    pub_poses = nh.advertise<sensor_msgs::PointCloud2> ("points_poses", 1);
    vis_pub = nh.advertise<visualization_msgs::Marker>( "surface_reconstruction/visualization_marker", 0 );
    /// advertising 3D map and camera poses in rviz


    /// pubishing current frame and the reprojection of the 3D map
    pub_image = it.advertise("surface_reconstruction/camera/image",1);
    /// pubishing current frame and the reprojection of the 3D map


    semidense_tracker.cont_frames = &cont_frames;
    semidense_tracker.stamps = &stamps;
    semidense_tracker.image_frame = &image_frame_aux;
    semidense_tracker.stamps_ros = &stamps_ros ;

    ///Launch semidense tracker thread*/
    boost::thread thread_semidense_tracker(&ThreadSemiDenseTracker,&images,&semidense_mapper,&semidense_tracker,&dense_mapper,&Map,&odom_pub,&pub_poses,&vis_pub,&pub_image);

    ///Launch semidense mapper thread
    boost::thread thread_semidense_mapper(&ThreadSemiDenseMapper,&images,&images_previous_keyframe,&semidense_mapper,&semidense_tracker,&dense_mapper,&Map,&pub_cloud);



    if (calculate_superpixels > 0.5)
     {
         ///launch dense mapper thread
         boost::thread thread_dense_mapper(&ThreadDenseMapper,&dense_mapper,&pub_cloud);
     }

    cout << "***    Surface Reconstruction program is working     *** " <<  endl << endl;
    cout << "***    Launch the example sequences or use your own sequence / live camera and update the file 'data.yml' with the corresponding camera_path and calibration parameters    ***"  << endl;

}



void vo_system::imgcb(const sensor_msgs::Image::ConstPtr& msg,const geometry_msgs::PoseStamped::ConstPtr& msg2)//,const geometry_msgs::PoseStamped::ConstPtr& msg2
{
    ///read images
    try {
        cv_bridge::CvImageConstPtr cv_ptr;
        cv_bridge::toCvShare(msg);
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);

        stamps_ros =  cv_ptr->header.stamp;
        stamps = cv_ptr->header.stamp.toSec();
        current_time = cv_ptr->header.stamp;
        image_frame_aux =  cv_ptr->image.clone();
        cont_frames++;
        cout<<" image time:  "<<stamps_ros.nsec<<endl;
        cout<<"pose time "<<msg2->header.stamp.nsec<<endl;
    }
    catch (const cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}