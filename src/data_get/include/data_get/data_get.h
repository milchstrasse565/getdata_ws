/*******************************************************************************

    BSD 3-Clause License
    Copyright (c) 2024, Mingkai
    All rights reserved.
    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
        Redistributions of source code must retain the above copyright notice, this
    list of conditions and the following disclaimer.
        Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.
        Neither the name of the copyright holder nor the names of its
    contributors may be used to endorse or promote products derived from
    this software without specific prior written permission.
    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
    ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
    FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
    DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
    SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
    *******************************************************************************/

//
// Created by Mingkai on 2024/03/26.
//
#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <cstdlib>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <realtime_tools/realtime_buffer.h>
#include <iostream>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <fstream>
using namespace message_filters;
using namespace std;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image,sensor_msgs::Imu,sensor_msgs::Image> mysync;

class DatasetGet
{
public:
    DatasetGet(ros::NodeHandle& nh);
    ~DatasetGet() = default;
    void callback(const sensor_msgs::Image::ConstPtr& depth_msg,
                          const sensor_msgs::Image::ConstPtr& rgb_msg,
                          const sensor_msgs::Imu::ConstPtr& imu_msg,
                          const sensor_msgs::Image::ConstPtr& red_img_msg);
    void run();
    void saveDataToLocal(cv::Mat &depth_image,cv::Mat &rgb_image, sensor_msgs::Imu &imu_, cv::Mat &red_image);
private:
    ros::NodeHandle nh_;
    message_filters::Subscriber<sensor_msgs::Image> *depth_sub_;
    message_filters::Subscriber<sensor_msgs::Image> *image_sub_;
    message_filters::Subscriber<sensor_msgs::Imu> *imu_sub_;
    message_filters::Subscriber<sensor_msgs::Image> *red_img_sub_;
    Synchronizer<mysync> *sync_;
    realtime_tools::RealtimeBuffer <cv::Mat> depth_image_buffer;
    realtime_tools::RealtimeBuffer <cv::Mat> rgb_image_buffer;
    realtime_tools::RealtimeBuffer <sensor_msgs::Imu>   imu_buffer;
    realtime_tools::RealtimeBuffer <cv::Mat> red_image_buffer;
    bool topic_update_ = false;
};