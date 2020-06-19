//
// Created by vickylzy on 2020/6/18.
//

#include <angle_utility/model_gazebo_setter.h>

#include <utility>

#include "angle_utility/transform_utility.hpp"



namespace check_ns{

    using namespace Eigen;

    void ModelStatusControler::init() {
        setter_ = n_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
        getter_ = n_.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
        init_quat_ = rotm2quat(init_state_);
        final_quat_ = rotm2quat(final_state_);
        translation_step_ = (final_state_.block(0,3,3,1)-init_state_.block(0,3,3,1)) * single_step_;

    }

    ModelStatusControler::ModelStatusControler(Eigen::Matrix4d initState, Eigen::Matrix4d finalState) : init_state_(std::move(
            initState)), final_state_(std::move(finalState)) {}

    geometry_msgs::PoseStampedConstPtr ModelStatusControler::step() {
        Quaterniond curr_quat = init_quat_.slerp((double)(curr_count_ % steps_total_) * single_step_, final_quat_);
        curr_quat.normalize();
        Vector3d curr_pos = init_state_.block(0,3,3,1) + (double)(curr_count_ % steps_total_) *translation_step_;
        //
        geometry_msgs::PoseStamped poseStamped;
        poseStamped.pose.position.x = curr_pos.x();
        poseStamped.pose.position.y = curr_pos.y();
        poseStamped.pose.position.z = curr_pos.z();
        poseStamped.pose.orientation.w = curr_quat.w();
        poseStamped.pose.orientation.x = curr_quat.x();
        poseStamped.pose.orientation.y = curr_quat.y();
        poseStamped.pose.orientation.z = curr_quat.z();

        gazebo_msgs::SetModelState setModelState_srv;
        setModelState_srv.request.model_state.reference_frame="world";
        setModelState_srv.request.model_state.model_name="static_tag";
        setModelState_srv.request.model_state.pose = poseStamped.pose;
        setter_.call(setModelState_srv);
        if(setModelState_srv.response.success){
            poseStamped.header.frame_id="world";
            poseStamped.header.stamp = ros::Time::now();
            geometry_msgs::PoseStampedConstPtr poseStampedConstPtr = boost::make_shared<geometry_msgs::PoseStamped>(poseStamped);
            ++curr_count_;
            return poseStampedConstPtr;
        }else{
            std::cerr<<"model_state set failed"<<std::endl;
            return nullptr;
        }

//        return geometry_msgs::PoseStampedConstPtr();

    }

//    ModelStatus::ModelStatus() = delete;
//    ModelStatus::~ModelStatus() = default;

}