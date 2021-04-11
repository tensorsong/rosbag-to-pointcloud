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