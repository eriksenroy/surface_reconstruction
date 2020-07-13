//
// Created by roy on 10/7/20.
//

#include <surface_reconstruction/vo_system.h>
#include <fstream>
#include <iomanip>

#include <ros/package.h>

vo_system::vo_system(){


    ///vo_system launch the three threads, tracking, semidense mapping and dense mapping (3D superpixels)

    cv::FileStorage  fs2( (ros::package::getPath("surface_reconstruction")+"/src/data.yml").c_str(), cv::FileStorage::READ);

    std::string camera_path;
    fs2["camera_path"] >> camera_path;
    cont_frames = 0;

    //int calculate_superpixels = (int)fs2["calculate_superpixels"];



    image_transport::ImageTransport it(nh);
    sub1 = it.subscribe(camera_path,1, & vo_system::imgcb,this);


    odom_pub = nh.advertise<nav_msgs::Odometry>("odom1", 50);



    /// advertising 3D map and camera poses in rviz
    pub_cloud = nh.advertise<sensor_msgs::Image> ("surface_reconstruction/map", 1);
    pub_poses = nh.advertise<sensor_msgs::Image> ("points_poses", 1);
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
    //boost::thread thread_semidense_tracker(&ThreadSemiDenseTracker,&images,&semidense_mapper,&semidense_tracker,&Map,&odom_pub,&pub_poses,&vis_pub,&pub_image);

    ///Launch semidense mapper thread
    //boost::thread thread_semidense_mapper(&ThreadSemiDenseMapper,&images,&images_previous_keyframe,&semidense_mapper,&semidense_tracker,&Map,&pub_cloud);


   /* if (calculate_superpixels > 0.5)
    {
        ///launch dense mapper thread
        boost::thread thread_dense_mapper(&ThreadDenseMapper,&dense_mapper,&pub_cloud);
    }
        */
    cout << "***    Surface Reconstruction program is working     *** " <<  endl << endl;
    cout << "***    Launch the example sequences or use your own sequence / live camera and update the file 'data.yml' with the corresponding camera_path and calibration parameters    ***"  << endl;

}



void vo_system::imgcb(const sensor_msgs::Image::ConstPtr& msg)
{
    ///read images
    try {
        cv_bridge::CvImageConstPtr cv_ptr;

        cv_bridge::toCvShare(msg);
        cv_ptr = cv_bridge::toCvShare(msg);

        stamps_ros =  cv_ptr->header.stamp;
        stamps = cv_ptr->header.stamp.toSec();
        current_time = cv_ptr->header.stamp;
        image_frame_aux =  cv_ptr->image.clone();
        cont_frames++;
    }
    catch (const cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}