#ifndef ARUN_REG_H
#define ARUN_REG_H

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>


/**
 * Example of Arun registration
 * Applied Robotics
 * Author: Garrison Johnston
 */
class ArunReg : public rclcpp::Node
{
public:
    /**
     * Constructor.
     */
    ArunReg();

    /**
     * Constructor.
     */
    ~ArunReg() = default;

    /**
     * Computes the Arun registration
     */
    void registerPoints();

private:
    /**
     * Computes the centroid of a list of points
     * @param points Array of points to compute centroid of
     * @return The centroid 
     */
    Eigen::Vector3d computeCentroid(std::array<Eigen::Vector3d,3> & points);

    /**
     * Computes the H matrix from Arun registration
     */
    void createH();

    /**
     * Computes the Fiducial Registration Error (FRE)
     * @return The FRE 
     */
    double computeFRE();

    /**
     * Computes the Target Registration Error (TRE)
     * @return The TRE 
     */
    double computeTRE();

    /**
     * Print the rotation matrix, TRE, and FRE
     */
    void printResults();

    int N_ = 3; // Number of points 
    std::array<Eigen::Vector3d,3> p_w_; // points in workpiece frame
    std::array<Eigen::Vector3d,3> p_r_; // points in robot frame

    std::array<Eigen::Vector3d,3> q_w_; // points in workpiece frame
    std::array<Eigen::Vector3d,3> q_r_; // points in robot frame

    Eigen::Matrix3d H_ = Eigen::Matrix3d::Zero(); // H Matrix from Arun Reg
    Eigen::Matrix3d R_rw_ = Eigen::Matrix3d::Zero(); // the rotation matrix of {w} wrt {r}

    Eigen::Vector3d p_rw_; // The position of {w} wrt {r}
    Eigen::Vector3d p_r_new_; // New point for TRE in {r}
    Eigen::Vector3d p_w_new_; // New point for TRE in {w}
};

#endif // EIGEN_EXAMPLE_H