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
#include <data_get/data_get.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dataset_get_node");
    ros::NodeHandle n;
    DatasetGet dataset_get(n);
    ros::Rate loop_rate(10);
    ros::AsyncSpinner spinner(1);  //多线程处理回调函数
    spinner.start();
    while (ros::ok())
    {
    dataset_get.run();
    loop_rate.sleep();
    }
    return 0;
}

DatasetGet::DatasetGet(ros::NodeHandle& nh)
{
    nh_ =  nh;
    depth_sub_ = new message_filters::Subscriber<sensor_msgs::Image>(nh_,"/camera/aligned_depth_to_color/image_raw", 1000);
    image_sub_  = new message_filters::Subscriber<sensor_msgs::Image>(nh_,"/camera/color/image_raw", 1000);
    imu_sub_  = new message_filters::Subscriber<sensor_msgs::Imu>(nh_,"/imu_data", 1000);
    red_img_sub_  = new message_filters::Subscriber<sensor_msgs::Image>(nh_,"/redcamera/image", 1000);
    sync_ = new Synchronizer<mysync>(mysync(10),*depth_sub_,*image_sub_,*imu_sub_,*red_img_sub_);
    sync_->registerCallback(boost::bind(&DatasetGet::callback, this, _1, _2, _3, _4));
}

void DatasetGet::callback(const sensor_msgs::Image::ConstPtr& depth_msg,
                          const sensor_msgs::Image::ConstPtr& rgb_msg,
                          const sensor_msgs::Imu::ConstPtr& imu_msg,
                          const sensor_msgs::Image::ConstPtr& red_img_msg)
{

        // 将 ROS 消息转换为 OpenCV 格式
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
    cv::Mat temp_image = cv_ptr->image;
    depth_image_buffer.writeFromNonRT(temp_image.clone());

    cv_ptr = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
    temp_image = cv_ptr->image;
    rgb_image_buffer.writeFromNonRT(temp_image.clone());


    imu_buffer.writeFromNonRT(*imu_msg);

    cv_ptr = cv_bridge::toCvCopy(red_img_msg, sensor_msgs::image_encodings::MONO8);
    temp_image = cv_ptr->image;
    red_image_buffer.writeFromNonRT(temp_image.clone());

    topic_update_ = true;
}

void DatasetGet::run(){
 if (topic_update_) {
        cv::Mat* depth_image = depth_image_buffer.readFromRT();
        cv::Mat* rgb_image = rgb_image_buffer.readFromRT();
        sensor_msgs::Imu* imu_ = imu_buffer.readFromRT();
        cv::Mat* red_image = red_image_buffer.readFromRT();
        saveDataToLocal(*depth_image, *rgb_image, *imu_, *red_image);
        topic_update_ = false; // 重置标志位
            }
}
void DatasetGet::saveDataToLocal(cv::Mat &depth_image, cv::Mat &rgb_image, sensor_msgs::Imu &imu_, cv::Mat &red_image) {
    // Save IMU data to imu_data.txt
    std::ofstream imu_file("/home/lmk/imgs/imu_data.txt", std::ios_base::app);
    if (imu_file.is_open()) {
        // Get current ROS time stamp
        std::stringstream time_ss;
        time_ss << imu_.header.stamp.sec << "." << std::setw(9) << std::setfill('0') << imu_.header.stamp.nsec;
        std::string timestamp = time_ss.str();

        imu_file << "IMU Data (Timestamp: " << timestamp << "):\n";
        imu_file << "Orientation: " << imu_.orientation.x << ", " << imu_.orientation.y << ", " << imu_.orientation.z << ", " << imu_.orientation.w << "\n";
        imu_file << "Angular Velocity: " << imu_.angular_velocity.x << ", " << imu_.angular_velocity.y << ", " << imu_.angular_velocity.z << "\n";
        imu_file << "Linear Acceleration: " << imu_.linear_acceleration.x << ", " << imu_.linear_acceleration.y << ", " << imu_.linear_acceleration.z << "\n\n";

        ROS_INFO("IMU data appended to imu_data.txt");
        imu_file.close();
    } else {
        ROS_ERROR("Unable to open file for appending IMU data!");
    }

    // Get current ROS time stamp for image filenames
    std::stringstream time_ss;
    time_ss << imu_.header.stamp.sec << "." << std::setw(9) << std::setfill('0') << imu_.header.stamp.nsec;
    std::string timestamp = time_ss.str();

    // Save depth image as PNG with timestamp as filename
    cv::imwrite("/home/lmk/imgs/depth_images/" + timestamp + ".png", depth_image);

    // Save RGB image as PNG with timestamp as filename
    cv::imwrite("/home/lmk/imgs/rgb_images/" + timestamp + ".png", rgb_image);

    // Save red image as PNG with timestamp as filename
    cv::imwrite("/home/lmk/imgs/red_images/" + timestamp + ".png", red_image);

    ROS_INFO("Dataset saved to local files.");
}

  // Solve all of perception here...
    // cout << "时间同步后depth时间戳是:  " << depth_msg->header.stamp << endl;
    // cout << "时间同步后rgb时间戳是:    " << rgb_msg->header.stamp << endl;
    // cout << "时间同步后imu时间戳是:    " << imu_msg->header.stamp << endl;
    // cout << "时间同步后redimg时间戳是:    " << red_img_msg->header.stamp << endl;
    //     // // 打印 IMU 数据
    // ROS_INFO("IMU data - Acceleration: [%f, %f, %f], Angular velocity: [%f, %f, %f]",
    //          imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z,
    //          imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);

    // // 转换深度图像为 OpenCV 格式并显示
    // cv_bridge::CvImagePtr depth_cv_ptr;
    // try
    // {
    //     depth_cv_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
    //     cv::imshow("Depth Image", depth_cv_ptr->image);
    //     cv::waitKey(1);
    // }
    // catch (cv_bridge::Exception& e)
    // {
    //     ROS_ERROR("cv_bridge exception: %s", e.what());
    //     return;
    // }

    // // 转换 RGB 图像为 OpenCV 格式并显示
    // cv_bridge::CvImagePtr rgb_cv_ptr;
    // try
    // {
    //     rgb_cv_ptr = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
    //     cv::imshow("RGB Image", rgb_cv_ptr->image);
    //     cv::waitKey(1);
    // }
    // catch (cv_bridge::Exception& e)
    // {
    //     ROS_ERROR("cv_bridge exception: %s", e.what());
    //     return;
    // }

    // // 转换红外图像为 OpenCV 格式并显示
    // cv_bridge::CvImagePtr infrared_cv_ptr;
    // try
    // {
    //     infrared_cv_ptr = cv_bridge::toCvCopy(red_img_msg, sensor_msgs::image_encodings::MONO8);
    //     cv::imshow("Infrared Image", infrared_cv_ptr->image);
    //     cv::waitKey(1);
    // }
    // catch (cv_bridge::Exception& e)
    // {
    //     ROS_ERROR("cv_bridge exception: %s", e.what());
    //     return;
    // }
