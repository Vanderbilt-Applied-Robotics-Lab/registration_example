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

    // destructor
    ~ArunReg() = default;

    // publishes the rotated vector
    void registerPoints();

private:
    Eigen::Vector3d computeCentroid(std::array<Eigen::Vector3d,3> & points);
    void createH();
    double computeFRE();
    double computeTRE();
    
    int N_ = 3;
    std::array<Eigen::Vector3d,3> p_w_; // points in workpiece frame
    std::array<Eigen::Vector3d,3> p_r_; // points in robot frame

    std::array<Eigen::Vector3d,3> q_w_; // points in workpiece frame
    std::array<Eigen::Vector3d,3> q_r_; // points in robot frame

    Eigen::Matrix3d H_ = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d R_ = Eigen::Matrix3d::Zero();

    Eigen::Vector3d p_rw_;
    Eigen::Vector3d p_r_new_;
    Eigen::Vector3d p_w_new_;
};

#endif // EIGEN_EXAMPLE_H