#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <assert.h>
#include <string>
#include <iostream>

class RosbagToPointCloudNode {

    private:
        std::string input_topic_;
        std::string point_type_;
        std::string save_dir_;
        std::string save_type_;
        std::stringstream ss;
        std::fstream fs;
        int number_;

    public:
        RosbagToPointCloudNode(ros::NodeHandle nh, std::string input_topic, std::string point_type,
                               std::string save_dir, std::string save_type);
        void get_paramters(ros::NodeHandle nh);
        void run(const sensor_msgs::PointCloud2 &msg);
        std::string get_input_topic();
};