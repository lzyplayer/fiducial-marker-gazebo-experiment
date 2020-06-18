//
// Created by vickylzy on 2020/6/10.
//

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

//ugly global var
ros::Publisher pub;
ros::Subscriber sub;
sensor_msgs::CameraInfoPtr cam_info;

void infoCallback(const sensor_msgs::ImageConstPtr &im_msg) {
    cam_info->header = im_msg->header;
    pub.publish(cam_info);
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "fake_info_checker");
    ros::NodeHandle n;
    std::string indo_topic = "/rrbot/camera1/camera_info";
    std::string camera_topic = "/rrbot/camera1/image_raw";

    pub = n.advertise<sensor_msgs::CameraInfo>(indo_topic,10);
    sub = n.subscribe(camera_topic,10,infoCallback);
    sensor_msgs::CameraInfoConstPtr info_msg =  ros::topic::waitForMessage<sensor_msgs::CameraInfo>(indo_topic);
    cam_info = boost::make_shared<sensor_msgs::CameraInfo>(*info_msg);

    ros::spin();
    return 0;

}
