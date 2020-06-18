//
// Created by vickylzy on 2020/4/15.
//

#ifndef ANGLE_UTILITY_ANGLE_CALCULATE_UTILITY_H
#define ANGLE_UTILITY_ANGLE_CALCULATE_UTILITY_H

namespace check_ns{
    /**
 * check angle Between Vectors
 *
 * @param a vec a
 * @param b vac b
 * @return
 */
    inline double angleBetweenVectors(Eigen::Vector3d &a, Eigen::Vector3d &b) {
        return std::atan2(a.cross(b).norm(), a.dot(b));
    }


}
#endif //ANGLE_UTILITY_ANGLE_CALCULATE_UTILITY_H
