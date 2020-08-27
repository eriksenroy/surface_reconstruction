//
// Created by roy on 25/8/20.
//

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using namespace std;

std::vector<std::vector<double>> v;
int i = 0;
int initiate = 0;

vector<vector<double>> readData()
{
    std::vector<std::vector<double> > data;

    std::ifstream          file("rgbd_dataset_freiburg1_xyz-groundtruth.txt");

    std::string   line;
    while(std::getline(file, line))
    {
        std::vector<double>   lineData;
        std::stringstream  lineStream(line);
        double value;
        // Read an integer at a time from the line
        while(lineStream >> value)
        {
            // Add the integers from a line to a 1D array (vector)
            lineData.push_back(value);
        }
        // When all the integers have been read, add the 1D array
        // into a 2D array (as one line in the 2D array)
        data.push_back(lineData);
    }
    return data;
}
void chatterCallback(const sensor_msgs::ImageConstPtr& msg)
{
    initiate = 1;
    cout << "roy"<<endl;
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "file_reader_poses");
    ros::NodeHandle nh;
    ros::Publisher dataPublisher = nh.advertise<geometry_msgs::PoseStamped>("/groundtruth", 1000);
//    ros::Subscriber sub = nh.subscribe("/camera/rgb/image_color", 1, chatterCallback);
    v = readData();
    cout << v[0][1]<<endl;
    ros::Rate rate(100);
    while (ros::ok())
    {
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp.sec = floor(v[i][0]);
        float n_secs = v[i][0]-floor(v[i][0]);
        pose_msg.header.stamp.nsec = n_secs*10000;
        pose_msg.pose.position.x = v[i][1];
        pose_msg.pose.position.y = v[i][2];
        pose_msg.pose.position.z = v[i][3];
        pose_msg.pose.orientation.x = v[i][4];
        pose_msg.pose.orientation.y = v[i][5];
        pose_msg.pose.orientation.z = v[i][6];
        pose_msg.pose.orientation.w = v[i][7];
        while (dataPublisher.getNumSubscribers()==0)
        {
            rate.sleep();
        }
        dataPublisher.publish(pose_msg);
        rate.sleep();
        i++;
        cout << i<<endl;
    }
    ros::spin();
    return 0;
}