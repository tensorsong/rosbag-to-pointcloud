/* ==================================================================
* Copyright (c) 2021.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the
* distribution.
* 3. All advertising materials mentioning features or use of this software
* must display the following acknowledgement:
* This product includes software developed by its contributors.
* 4. Neither the name of the Group nor the names of its contributors may
* be used to endorse or promote products derived from this software
* without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY Ding Song
* ===================================================================
* Author: Ding Song.
*/

#include "bag_to_pointcloud/rosbag_to_pointcloud_node.hpp"

RosbagToPointCloudNode::RosbagToPointCloudNode(ros::NodeHandle nh,
                                               std::string input_topic,
                                               std::string point_type,
                                               std::string save_dir,
                                               std::string save_type) {
    get_paramters(nh);
    number_ = 0;
}

void RosbagToPointCloudNode::get_paramters(ros::NodeHandle nh) {
    nh.getParam("input_topic", input_topic_);
    nh.getParam("point_type", point_type_);
    nh.getParam("save_dir", save_dir_);
    nh.getParam("save_type", save_type_);
}

void RosbagToPointCloudNode::run(const sensor_msgs::PointCloud2 &msg) {
    assert(point_type_ == "XYZI" || point_type_ == "XYZRGB" ||
           save_type_ == "pcd" || save_type_ == "txt");
    std::string name = std::to_string(number_), save_path;
    if (point_type_ == "XYZI") {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(msg, *cloud);
        if (save_type_ == "pcd") {
            save_path = save_dir_ + "/" + name + "_i.pcd";
            std::cout << "save_path: " << save_path << std::endl;
            pcl::io::savePCDFileASCII(save_path, *cloud);
        } else {
            save_path = save_dir_ + "/" + name + "_i.txt";
            fs.open(save_path, std::fstream::out);
                fs << "x\ty\tz\tintensity\n";
                for (size_t i = 0; i < cloud->points.size(); ++i) {
                    fs << cloud->points[i].x << "\t" << cloud->points[i].y << "\t"
                    << cloud->points[i].z << "\t" << cloud->points[i].intensity << "\n";
                }
            fs.close();
        }
    } else {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(msg, *cloud);
        if (save_type_ == "pcd") {
            save_path = save_dir_ + "/" + name + "_rgb.pcd";
            pcl::io::savePCDFileASCII(save_path, *cloud);
        } else {
            save_path = save_dir_ + "/" + name + "_rgb.txt";
            fs.open(save_path, std::fstream::out);
                fs << "x\ty\tz\tr\tg\tb\n";
                for (size_t i = 0; i < cloud->points.size(); ++i) {
                    fs << cloud->points[i].x << "\t" << cloud->points[i].y << "\t" << cloud->points[i].z 
                       << "\t" << cloud->points[i].r << "\t" << cloud->points[i].g << "\t" << cloud->points[i].b << "\n";
                }
            fs.close();
        }
    }
    if (number_ != 0 && number_ % 10 == 0) {
        ROS_INFO(" %s has been saved.", save_path.c_str());
    }
    number_ += 1;
}

std::string RosbagToPointCloudNode::get_input_topic() {
    return input_topic_;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "rosbag_to_pointcloud");
    ros::NodeHandle nh("~");
    ros::Subscriber cloud_sub;
    std::string input_topic, point_type, save_dir, save_type;
    RosbagToPointCloudNode node(nh, input_topic, point_type, save_dir, save_type);
    cloud_sub = nh.subscribe(node.get_input_topic(), 1, &RosbagToPointCloudNode::run, &node);
    ros::spin();
    return 0;
}