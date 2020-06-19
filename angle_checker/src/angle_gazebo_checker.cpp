//
// Created by vickylzy on 2020/4/15.
//

#include "angle_utility/angle_gazebo_checker.h"

#include "angle_utility/model_gazebo_setter.h"
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <memory>



using namespace std;
using namespace Eigen;
namespace check_ns {
    Eigen::Vector3d calculate_err_R(const Eigen::Matrix4d &GroundTruth, const Eigen::Matrix4d &sample) {
        Eigen::Vector3d x(1, 0, 0), y(0, 1, 0), z(0, 0, 1);
        Eigen::Matrix3d t_a_m = GroundTruth.block(0, 0, 3, 3);
        Eigen::Matrix3d t_b_m = sample.block(0, 0, 3, 3);
        Eigen::Vector3d x_a = t_a_m * x;
        Eigen::Vector3d y_a = t_a_m * y;
        Eigen::Vector3d z_a = t_a_m * z;
        Eigen::Vector3d x_b = t_b_m * x;
        Eigen::Vector3d y_b = t_b_m * y;
        Eigen::Vector3d z_b = t_b_m * z;
        Eigen::Vector3d result(angleBetweenVectors(x_a, x_b) / M_PI * 180, angleBetweenVectors(y_a, y_b) / M_PI * 180,
                               angleBetweenVectors(z_a, z_b) / M_PI * 180);
        return result;
    }

    Eigen::Vector3d calculate_err_T(const Eigen::Matrix4d &GroundTruth, const Eigen::Matrix4d &sample) {
        Eigen::Vector3d gt_t = GroundTruth.block(0, 3, 3, 1);
        Eigen::Vector3d t = sample.block(0, 3, 3, 1);
        Eigen::Vector3d result = gt_t - t;
        return result;
    }

    Eigen::Matrix4d
    get_matrix_tar_world_from_modelstate(const gazebo_msgs::ModelStates &modelStates, const std::string &linkname) {
        int index = -1;
        Eigen::Matrix4d formatted_rm = Eigen::Matrix4d::Identity();
        for (int i = 0; i < modelStates.name.size(); ++i) {
            if (modelStates.name[i] == linkname) {
                index = i;
                break;
            }
        }
        if (index != -1) {
            formatted_rm = geome2eigenM(modelStates.pose[index]);
        } else {
            std::cout << "WARNING: cannot find given link, return Identity" << std::endl;
            return formatted_rm;
        }

    }

    Eigen::Matrix4d geome2eigenM(const geometry_msgs::Pose &pose) {
        Eigen::Matrix4d result_matrix = Eigen::Matrix4d::Identity();
        Eigen::Quaterniond quaterniond(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
//        Eigen::Matrix3d rotm  = quaterniond.toRotationMatrix();
        result_matrix.block(0, 0, 3, 3) = quaterniond.toRotationMatrix();
        Eigen::Vector3d trans(pose.position.x, pose.position.y, pose.position.z);
        result_matrix.block(0, 3, 3, 1) = trans;
//        std::cout << "result_matrix\n" << result_matrix << std::endl;
        return result_matrix;
    }

    Eigen::Matrix4d tf2eigenM(const tf::StampedTransform &tform) {
        Eigen::Matrix4d result_matrix = Eigen::Matrix4d::Identity();
        Eigen::Quaterniond quaterniond(tform.getRotation().w(), tform.getRotation().x(), tform.getRotation().y(),
                                       tform.getRotation().z());
//        Eigen::Matrix3d rotm  = quaterniond.toRotationMatrix();
        result_matrix.block(0, 0, 3, 3) = quaterniond.toRotationMatrix();
        Eigen::Vector3d trans(tform.getOrigin().getX(), tform.getOrigin().getY(), tform.getOrigin().getZ());
        result_matrix.block(0, 3, 3, 1) = trans;
//        std::cout << "result_matrix\n" << result_matrix << std::endl;
        return result_matrix;
    }

    void GazeboSrvCaller::init() {
        // param
//        hz_ = check_ns::fps_;
        initParam.camera_opti2world << 0.0000, 0.0000, 1.0000, 0,
                                    -1.0000, 0, 0.0000, 0,
                                    0, -1.0000, 0.0000, 0,
                                    0, 0, 0, 1;
        initParam.model2apriltag << 0, -1, 0, 0,
                                    0, 0, 1, -1,
                                    -1, 0, 0, 0,
                                    0, 0, 0, 1;
        double hz_duration = 1.0f / (double) initParam.gt_hz_;
        gt_pose_stack.set_capacity(5 * initParam.gt_hz_);
        cv::FileStorage fsetting(initParam.default_yaml_path_,cv::FileStorage::READ);

        cv::Mat start_cvmat, final_cvmat;
        Eigen::Matrix4d target_pose = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d start_pose = Eigen::Matrix4d::Identity();
        fsetting["start_motion"] >> start_cvmat;
        fsetting["target_motion"] >> final_cvmat;
        cv::cv2eigen(start_cvmat,start_pose);
        cv::cv2eigen(final_cvmat,target_pose);

        modelstat_ptr = std::make_unique<ModelStatusControler>(start_pose, target_pose);
        modelstat_ptr->init();
        //
        model_getter_ = n_.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
//        pose_suber = n_.subscribe(initParam.tag_detect_pose_topic_, 5, &GazeboSrvCaller::check_callback, this);
        gt_timer = n_.createTimer(ros::Duration(hz_duration), &GazeboSrvCaller::gt_callback, this);
    }

    void GazeboSrvCaller::gt_callback(const ros::TimerEvent &event) {

        geometry_msgs::PoseStampedConstPtr  curr_gt = modelstat_ptr->step();
        geometry_msgs::PoseStampedPtr  curr_pose;
        while (true){
            geometry_msgs::PoseStampedConstPtr  get_pose = ros::topic::waitForMessage<geometry_msgs::PoseStamped>(initParam.tag_detect_pose_topic_,n_);
            if(get_pose->header.stamp>curr_gt->header.stamp){
                curr_pose.reset(new geometry_msgs::PoseStamped(*get_pose));
                break;
            }

        }
        Matrix4d groudT = geome2eigenM(curr_gt->pose);
        Matrix4d sample = geome2eigenM(curr_pose->pose);
        Matrix4d transformed_sample = initParam.camera_opti2world * sample*initParam.model2apriltag;
//            std::cout<<groudT<<"\n\n"<<transformed_sample<<std::endl;
        Vector3d r_err = calculate_err_R(groudT, transformed_sample);
        Vector3d t_err = calculate_err_T(groudT, transformed_sample);
        cout << "position_err:\nx: " << t_err[0] << "\ty: " << t_err[1] << "\tz: " << t_err[2] << "\norirition:\n"
             << "dx: " << r_err[0] << "\tdy: " << r_err[1] << "\tdz: " << r_err[2] << endl;
//        gazebo_msgs::GetModelState getModelSrv;
//        getModelSrv.request.model_name = initParam.model_name_;
//        model_getter_.call(getModelSrv);
//        //stack gt
//        geometry_msgs::PoseStamped poseStamped;
//        poseStamped.header = getModelSrv.response.header;
//        poseStamped.pose = getModelSrv.response.pose;
//        gt_pose_stack.push_back(poseStamped);
//        if (gt_pose_stack.size()==5*initParam.gt_hz_){
//            for (auto iter=gt_pose_stack.begin();iter!=gt_pose_stack.end();++iter) {
//                std::cout<<std::setprecision(15)<<iter->header.stamp.toSec()<<"\n";
//            }
//        }
//        std::cout<<std::endl;
    }

//    void GazeboSrvCaller::check_callback(const geometry_msgs::PoseStampedConstPtr &ps_SCP) {
//        geometry_msgs::PoseStamped curr_gt;
//        bool found_gt_flag = false;
//        for (auto iter = gt_pose_stack.rbegin(); iter != gt_pose_stack.rend(); iter++) {
//            ros::Duration duration = iter->header.stamp - ps_SCP->header.stamp;
//            if (duration.toSec() < 0) {
//                curr_gt = abs(duration.toSec()) < abs(((iter - 1)->header.stamp - ps_SCP->header.stamp).toSec()) ? *iter : *(iter - 1);
//                found_gt_flag = true;
//                break;
//            }
//        }
//        if (found_gt_flag) {
//            Matrix4d groudT = geome2eigenM(curr_gt.pose);
//            Matrix4d sample = geome2eigenM(ps_SCP->pose);
//            Matrix4d transformed_sample = initParam.camera_opti2world * sample*initParam.model2apriltag;
////            std::cout<<groudT<<"\n\n"<<transformed_sample<<std::endl;
//            Vector3d r_err = calculate_err_R(groudT, transformed_sample);
//            Vector3d t_err = calculate_err_T(groudT, transformed_sample);
//            cout << "position_err:\nx: " << t_err[0] << "\ty: " << t_err[1] << "\tz: " << t_err[2] << "\norirition:\n"
//                 << "dx: " << r_err[0] << "\tdy: " << r_err[1] << "\tdz: " << r_err[2] << endl;
//
//        }
//    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "angle_gazebo_checker");
    //new
    check_ns::GazeboSrvCaller gazeboSrvCaller;
    gazeboSrvCaller.init();
    ros::spin();
    return 0;

}
//old
//tf::TransformListener transformListener;
//Eigen::Matrix4d M_GT_tb;
//M_GT_tb << 0, -1, 0, -1.5, \
//             0, 0, 1, 0, \
//             -1, 0, 0, 0, \
//             0, 0, 0, 1;
//std::string error_msg;
//ros::Time curr_time;
////    check_ns::former_movment = Eigen::Matrix4d::Identity();
//while (ros::ok()) {
//
//auto ms_p = ros::topic::waitForMessage<gazebo_msgs::ModelStates>("/gazebo/model_states");
//curr_time = ros::Time::now();
//Eigen::Matrix4d M_GT_wb = check_ns::get_matrix_tar_world_from_modelstate(*ms_p, check_ns::link_name);
//
//if (transformListener.canTransform(check_ns::target_frame, check_ns::source_frame, ros::Time(0))) {
///*
// * check relative
// */
//
//tf::StampedTransform tf_M_tw;
//
//try {
//transformListener.waitForTransform(check_ns::target_frame, check_ns::source_frame, curr_time,
//        ros::Duration(1));
//transformListener.lookupTransform(check_ns::target_frame, check_ns::source_frame, curr_time, tf_M_tw);
//}
//catch (tf::TransformException ex) {
//ROS_ERROR("cannot get tf . %s", ex.what());
////                rate.sleep();
//}
//Eigen::Matrix4d M_tw = check_ns::tf2eigenM(tf_M_tw);
//Eigen::Matrix4d M_GT_tw = M_GT_tb * M_GT_wb.inverse();
//Eigen::Vector3d delta_rot_vector = check_ns::calculate_err_R(M_GT_tw, M_tw);
//Eigen::Vector3d delta_pos_vector = check_ns::calculate_err_T(M_GT_tw, M_tw);
//// ROS_INFO(
////         "-----------------------------------------------------------------\ndifference between transforms euler(degree): x: %f\t, y: %f\t, z: %f\t\ndifference between transforms position(meter): x: %f\t, y: %f\t, z: %f",
////         delta_rot_vector.x(),
////         delta_rot_vector.y(),
////         delta_rot_vector.z(),
////         delta_pos_vector.x(),
////         delta_pos_vector.y(),
////         delta_pos_vector.z());
//printf(
//"\n%f\t%f\t%f\t\n%f\t %f\t%f\n",
//delta_rot_vector.x(),
//        delta_rot_vector.y(),
//        delta_rot_vector.z(),
//        delta_pos_vector.x(),
//        delta_pos_vector.y(),
//        delta_pos_vector.z());
////            {// get difference on axis
////                Eigen::Vector3d delta_vector = check_ns::minus_angle(M_april_ar);
////                ROS_INFO("difference between transforms euler: x: %f\t, y: %f\t, z: %f\t",
////                         delta_vector.x(),
////                         delta_vector.y(),
////                         delta_vector.z());
////            }
//////            {// get shake stats
//////                check_ns::get_shake_stats(M_april_ar);
//////            }
//}
//
////        rate.sleep();
//
//}