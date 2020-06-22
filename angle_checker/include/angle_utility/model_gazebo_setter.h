//
// Created by vickylzy on 2020/6/18.
//
#ifndef SRC_MODEL_GAZEBO_SETTER_H
#define SRC_MODEL_GAZEBO_SETTER_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>



namespace check_ns{
    class ModelStatusControler{
    public:

        ModelStatusControler(Eigen::Matrix4d initState, Eigen::Matrix4d finalState);

        ModelStatusControler() = delete;
        ~ModelStatusControler() = default;
        void init();

        geometry_msgs::PoseStampedConstPtr step();

    private:
        Eigen::Matrix4d init_state_;
        Eigen::Matrix4d final_state_;
        Eigen::Quaterniond init_quat_;
        Eigen::Quaterniond final_quat_;

        long curr_count_ = 0;
        double single_step_ = 1e-4; //0~1  1e5 motions
        Eigen::Vector3d translation_step_;
        long steps_total_ = (long)(1.0f / single_step_);


        ros::NodeHandle n_;
        ros::ServiceClient setter_;
        ros::ServiceClient getter_;



    };
}
#endif //SRC_MODEL_GAZEBO_SETTER_H
