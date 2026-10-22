#include <arun_reg/arun_reg.h>

ArunReg::ArunReg() : Node("arun_reg")
{
    // setup robot points
    
    // setup workpiece points
    p_w_[0] << 1.0001, -0.0002, 2.0002;
    p_w_[1] << -0.0001, 0.7072, 2.7070;
    p_w_[2] << 0.001, -0.7070, 2.7072;

    // setup new point
    p_r_new_ << 0.7071, 0.7071, 0;
    p_w_new_ << 0.7071, 0.5, 2.5;
}

void ArunReg::registerPoints()
{
    // compute Centroids

    // offset from centroids

    // create H

    // Compute SVD
    // compute R

    // check determinate
        // compute frame offset
        // print TRE and FRE
}

Eigen::Vector3d ArunReg::computeCentroid(std::array<Eigen::Vector3d,3> & points)
{

}

void ArunReg::createH()
{

}

double ArunReg::computeFRE()
{

}

double ArunReg::computeTRE()
{
    
}

int main(int argc, char * argv[])
{
    // initialize the node
    rclcpp::init(argc, argv);

    // create instance of class
    auto node = std::make_shared<ArunReg>();

    // MAIN LOOP
    node->registerPoints();
    rclcpp::spin_some(node); // updates publishers and subscribers
    rclcpp::shutdown();
    return 0;
}