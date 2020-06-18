//
// Created by vickylzy on 20-3-24.
//

#ifndef SRC_ANGLE_CHECKER_H
#define SRC_ANGLE_CHECKER_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include <cmath>

#include "angle_utility/angle_calculate_utility.h"

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/max.hpp>
//#include <boost/accumulators/statistics/moment.hpp>
#include <boost/accumulators/statistics/variance.hpp>

namespace check_ns {

using namespace tf;
using namespace boost::accumulators;

int fps_ = 64;
std::string camera_frame = "rgb_camera_link";
std::string april_frame = "ar_marker";
std::string ar_frame = "ar_marker_2";

//shake param
unsigned long int frame_num = 0;
double max_differ_z_axis = 0;
double max_differ_center_dis = 0;
double average_centerx=-1;
double average_centery=-1;
Eigen::Matrix4d former_movment;

bool first_flag = false;

boost::accumulators::accumulator_set<double, boost::accumulators::stats<tag::variance> > y_pos_acc;
boost::accumulators::accumulator_set<double, boost::accumulators::stats<tag::variance> > x_pos_acc;
boost::accumulators::accumulator_set<double, boost::accumulators::stats<tag::variance> > z_axis_acc;

void get_shake_stats(tf::StampedTransform &t_a);

/**
 * check axes change in degree
 *
 * @param t_a transform between
 * @return Eigen::Vector3d [delta_x, delta_y, delta_z]
 */
Eigen::Vector3d minus_angle(tf::StampedTransform &t_a);
}

#endif //SRC_ANGLE_CHECKER_H
