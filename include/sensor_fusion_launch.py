#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <Eigen/Dense>  // Eigen library for matrices and vectors

// Camera intrinsic parameters
const double fx = 1527.58; // Focal length (x)
const double fy = 1550.17; // Focal length (y)
const double cx = 970.83;  // Principal point (x)
const double cy = 731.92;  // Principal point (y)

// Extrinsic parameters
const Eigen::Matrix3d R_lc = (Eigen::Matrix3d() << 0, 1, 0,
                                                    0, 0, -1,
                                                    1, 0, 0).finished(); // Rotation matrix
const Eigen::Vector3d t(0.0, 0.0, 0.05); // Translation vector

#endif // PARAMETERS_H