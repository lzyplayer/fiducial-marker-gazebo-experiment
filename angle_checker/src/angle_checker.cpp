//
// Created by vickylzy on 20-3-24.
//

#include "angle_utility/angle_checker.h"

namespace check_ns {
    using namespace tf;
    using namespace std;



    Eigen::Vector3d minus_angle(tf::StampedTransform &t_a) {
        Eigen::Vector3d x(1, 0, 0), y(0, 1, 0), z(0, 0, 1);
//        std::cout<<"t_a"<<std::endl<<t_a<<std::endl;
        Eigen::Matrix4d t_a_m = Eigen::Matrix4d::Identity();
        Eigen::Quaterniond t_a_e(t_a.getRotation().w(), t_a.getRotation().x(), t_a.getRotation().y(),
                                 t_a.getRotation().z());
//        std::cout<<"t_a_e"<<std::endl<<t_a_e.w()<<" "<<t_a_e.x()<<" "<<t_a_e.y()<<" "<<t_a_e.z()<<" "<<std::endl;
        t_a_m.block(0, 0, 3, 3) = t_a_e.toRotationMatrix();
//        std::cout<<"t_a_m"<<std::endl<<t_a_m<<std::endl;
        t_a_m(0, 3) = t_a.getOrigin().x();
        t_a_m(1, 3) = t_a.getOrigin().y();
        t_a_m(2, 3) = t_a.getOrigin().z();
        Eigen::Matrix3d t_a_m_3 = t_a_m.block(0, 0, 3, 3);

        Eigen::Vector3d x_a = t_a_m_3 * x;
        Eigen::Vector3d y_a = t_a_m_3 * y;
        Eigen::Vector3d z_a = t_a_m_3 * z;

        Eigen::Vector3d result(angleBetweenVectors(x_a, x) / M_PI * 180,
                               angleBetweenVectors(y_a, y) / M_PI * 180,
                               angleBetweenVectors(z_a, z) / M_PI * 180);
//      check rotation angle instead
//        Eigen::Vector3d t_a_euler = t_a_m_3.eulerAngles(0,1,2);
//        {
//            result[2] = sqrt(pow(t_a.getOrigin().getX(),2)+pow(t_a.getOrigin().getY(),2));
//        }
        return result;
    }

    void get_shake_stats(tf::StampedTransform &t_a) {
        Eigen::Vector3d x(1, 0, 0), y(0, 1, 0), z(0, 0, 1);
        // get trans matrix
        Eigen::Matrix4d t_a_m = Eigen::Matrix4d::Identity();
        Eigen::Quaterniond t_a_e(t_a.getRotation().w(), t_a.getRotation().x(), t_a.getRotation().y(),
                                 t_a.getRotation().z());
//        std::cout<<"t_a_e"<<std::endl<<t_a_e.w()<<" "<<t_a_e.x()<<" "<<t_a_e.y()<<" "<<t_a_e.z()<<" "<<std::endl;
        t_a_m.block(0, 0, 3, 3) = t_a_e.toRotationMatrix();
//        std::cout<<"t_a_m"<<std::endl<<t_a_m<<std::endl;
        t_a_m(0, 3) = t_a.getOrigin().x();
        t_a_m(1, 3) = t_a.getOrigin().y();
        t_a_m(2, 3) = t_a.getOrigin().z();
        Eigen::Matrix3d t_a_m_3 = t_a_m.block(0, 0, 3, 3);

        // sys walks
        if (former_movment!=t_a_m) {
            ++frame_num;
            x_pos_acc(t_a_m(0, 3));
            y_pos_acc(t_a_m(1, 3));
            Eigen::Vector3d z_a = t_a_m_3 * z;
            double z_axis_deg = angleBetweenVectors(z_a, z) / M_PI * 180;
            z_axis_acc(z_axis_deg);
            //get differ
            double x_dis = fabs(t_a_m(0, 3) - boost::accumulators::mean(x_pos_acc));
            double y_dis = fabs(t_a_m(1, 3) - boost::accumulators::mean(y_pos_acc));
            double pos_dis = sqrt((pow(x_dis, 2)) + (pow(y_dis, 2)));
            double axis_z_dis = fabs(z_axis_deg - boost::accumulators::mean(z_axis_acc));

            max_differ_center_dis = max_differ_center_dis > pos_dis ? max_differ_center_dis : pos_dis;
            max_differ_z_axis = max_differ_z_axis > axis_z_dis ? max_differ_z_axis : axis_z_dis;
            std::cout << "-------------------------------------------------------\n"\
 << "posx(m): " << t_a_m(0, 3) << "\tposy(m): " << t_a_m(1, 3) << "\taxis_z(deg): " << z_axis_deg << "\n"\
 << "distance_to_mean: " << pos_dis << "\tz_axis: " << axis_z_dis << "\n"\
 << "max_differ_center_dis: " << max_differ_center_dis <<"\t max_differ_z_axis: "<<max_differ_z_axis<< "\n"\
 << "Standard deviation_x(m):" << sqrt(boost::accumulators::variance(x_pos_acc)) << "\tStandard deviation_y(m):"
                      << sqrt(boost::accumulators::variance(y_pos_acc)) << "\tStandard deviation_axis_z(deg):"
                      << sqrt(boost::accumulators::variance(z_axis_acc)) << std::endl;
//  <<"diff_to_mean"<<std::endl;
            former_movment = t_a_m;
        }
    }

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "angle_checker");
    ros::NodeHandle n;
    tf::TransformListener transformListener;
    ros::Rate rate(check_ns::fps_);//camera fps
    std::string error_msg;
    ros::Time curr_time;
    //
    check_ns::former_movment = Eigen::Matrix4d::Identity();
    while (ros::ok()) {
        curr_time = ros::Time(0);
        if (transformListener.canTransform(check_ns::april_frame, check_ns::ar_frame, curr_time)) {
            /*
             * check relative
             */
            tf::StampedTransform M_april_ar;
            try {
                transformListener.lookupTransform(check_ns::april_frame, check_ns::ar_frame, curr_time, M_april_ar);
            }
            catch (tf::TransformException ex) {
                ROS_ERROR("cannot get april_tag-ar %s", ex.what());
                rate.sleep();
            }
      {// get difference on axis
        Eigen::Vector3d delta_vector = check_ns::minus_angle(M_april_ar);
        ROS_INFO("difference between transforms euler: x: %f\t, y: %f\t, z: %f\t",
                 delta_vector.x(),
                 delta_vector.y(),
                 delta_vector.z());
      }
//            {// get shake stats
//                check_ns::get_shake_stats(M_april_ar);
//            }
        }

        rate.sleep();

    }
}

/*
 * check pose to camera
 */
//        tf::StampedTransform April_transform;
//        tf::StampedTransform ar_transform;
//        ros::Time curr_time =ros::Time(0);
//        try{
//            transformListener.lookupTransform(april_frame,camera_frame,curr_time,April_transform);
//        }
//        catch (tf::TransformException ex){
//            ROS_ERROR("cannot get april_tag-camera %s",ex.what());
//            ros::Duration(1.0).sleep();
//        }
//        try{
//            transformListener.lookupTransform(ar_frame,camera_frame,curr_time,ar_transform);
////            transformListener.lookupTransform(camera_frame,ar_frame,curr_time,ar_transform);
//        }
//        catch (tf::TransformException ex){
//            ROS_ERROR("cannot get ar_tag-camera %s",ex.what());
//            ros::Duration(1.0).sleep();
//        }
//Eigen::Vector3d minus_angle(tf::StampedTransform &t_a, tf::StampedTransform &t_b ){
//        Eigen::Vector4d x(1,0,0,1),y(0,1,0,1),z(0,0,1,1);
//        Eigen::Quaterniond t_a_e(t_a.getRotation().w(),t_a.getRotation().x(),t_a.getRotation().y(),t_a.getRotation().z());
//        Eigen::Quaterniond t_b_e(t_b.getRotation().w(),t_b.getRotation().x(),t_b.getRotation().y(),t_b.getRotation().z());
//        Eigen::Matrix4d t_a_m = Eigen::Matrix4d::Identity();
//        Eigen::Matrix4d t_b_m = Eigen::Matrix4d::Identity();
//        t_a_m.block(0,0,3,3) = t_a_e.toRotationMatrix();
//        t_b_m.block(0,0,3,3) = t_b_e.toRotationMatrix();
//        t_a_m(0,3)= t_a.getOrigin().x();t_a_m(1,3)= t_a.getOrigin().y();t_a_m(2,3)= t_a.getOrigin().z();
//        t_b_m(0,3)= t_b.getOrigin().x();t_b_m(1,3)= t_b.getOrigin().y();t_b_m(2,3)= t_b.getOrigin().z();
////        std::cout<<"t_a_m"<<std::endl<<t_a_m<<std::endl;
////        std::cout<<"t_b_m"<<std::endl<<t_b_m<<std::endl;
//        Eigen::Matrix3d t_a_m_3= t_a_m.block(0,0,3,3);
//        Eigen::Matrix3d t_b_m_3= t_b_m.block(0,0,3,3);
//
////        std::cout<<"euler_a "<<std::endl<<t_a_m_3.eulerAngles(0,1,2)<<std::endl;
////        std::cout<<"euler b"<<std::endl<<t_b_m_3.eulerAngles(0,1,2)<<std::endl;
////      check rotation angle instead
//        Eigen::Vector3d t_a_euler = t_a_m_3.eulerAngles(0,1,2);
//        Eigen::Vector3d t_b_euler = t_b_m_3.eulerAngles(0,1,2);
//        Eigen::Vector3d result = t_a_euler-t_b_euler;
//
////      problem need to fix on check angle between angles
////        Eigen::Vector4d x_a = t_a_m*x;
////        Eigen::Vector4d y_a = t_a_m*y;
////        Eigen::Vector4d z_a = t_a_m*z;
////        Eigen::Vector4d x_b = t_b_m*x;
////        Eigen::Vector4d y_b = t_b_m*y;
////        Eigen::Vector4d z_b = t_b_m*z;
////        Eigen::Vector3d x_a_p = x_a.segment(0,3);
////        Eigen::Vector3d y_a_p = y_a.segment(0,3);
////        Eigen::Vector3d z_a_p = z_a.segment(0,3);
////        Eigen::Vector3d x_b_p = x_b.segment(0,3);
////        Eigen::Vector3d y_b_p = y_b.segment(0,3);
////        Eigen::Vector3d z_b_p = z_b.segment(0,3);
////
////        Eigen::Vector3d result(angleBetweenVectors(x_a_p,x_b_p), angleBetweenVectors(y_a_p,y_b_p), angleBetweenVectors(z_a_p,z_b_p));
//        return result;
//
//    }