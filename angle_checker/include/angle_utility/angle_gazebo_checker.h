//
// Created by vickylzy on 2020/4/15.
//

#ifndef ANGLE_UTILITY_ANGLE_GAZEBO_CHECKER_H
#define ANGLE_UTILITY_ANGLE_GAZEBO_CHECKER_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include <cmath>
#include <utility>

#include "angle_utility/angle_calculate_utility.h"
#include <gazebo_msgs/ModelStates.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/circular_buffer.hpp>
//#include <boost/accumulators/statistics/moment.hpp>
#include <boost/accumulators/statistics/variance.hpp>

#include <gazebo_msgs/GetModelState.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Image.h>

namespace check_ns {

    struct Init_param{
        const int gt_hz_ = 1000;
        const std::string link_name_ = "textured_shapes_my";
        const std::string target_frame_ = "ar_marker";
        const std::string source_frame_ = "world";
        const std::string model_name_ = "animated_tag";
        const std::string tag_detect_pose_topic_ = "/tag_detections/pose";
        Eigen::Matrix4d camera_opti2world;
        Eigen::Matrix4d model2apriltag;
    };

//    using
    /**
     * calculate err Rotation
     * @param GroundTruth
     * @param sample data matrix
     * @return
     */
    Eigen::Vector3d calculate_err_R(const Eigen::Matrix4d &GroundTruth, const Eigen::Matrix4d &sample);

    /**
     * calculate err translation
     * @param GroundTruth
     * @param sample
     * @return
     */
    Eigen::Vector3d calculate_err_T(const Eigen::Matrix4d &GroundTruth, const Eigen::Matrix4d &sample);

    /**
     * get eigen Matrix with link name from gazebo moddelstate msg
     * @param modelStates
     * @param linkname
     * @return
     */
    Eigen::Matrix4d
    get_matrix_tar_world_from_modelstate(const gazebo_msgs::ModelStates &modelStates, const std::string &linkname);

    /**
     * geometry_msg pose 2 eigen::Matrix4d
     * @param pose
     * @return
     */
    Eigen::Matrix4d geome2eigenM(const geometry_msgs::Pose &pose);

    /**
     * tf::transform 2 eigen::Matrix4d
     * @param tform
     * @return
     */
    Eigen::Matrix4d tf2eigenM(const tf::StampedTransform &tform);



//    Eigen::Matrix4d M_GT_tb;

    class GazeboSrvCaller{

    public:

        GazeboSrvCaller() {}

        ~GazeboSrvCaller() {}
        void init();
        void check_callback(const geometry_msgs::PoseStampedConstPtr &ps_SCP);
        void gt_callback(const ros::TimerEvent& event);
//        void image_timestamp_callback(const sensor_msgs::ImageConstPtr& im){
//            ros::Duration duration = ros::Time::now() - im->header.stamp;
//            std::cout<<duration.toSec()<<std::endl;
//            last_im_time = im->header.stamp;
//        }

    public:
        ros::NodeHandle n_;
        ros::ServiceClient model_getter_;
        ros::ServiceClient sim_pauser_;
        ros::ServiceClient sim_unpauser_;

        ros::Time last_im_time;

    private:
        // communicator
        ros::Timer gt_timer;
        ros::Subscriber pose_suber;
        // param

        Init_param initParam;
        boost::circular_buffer<geometry_msgs::PoseStamped> gt_pose_stack;
    };

}


#endif //ANGLE_UTILITY_ANGLE_GAZEBO_CHECKER_H
