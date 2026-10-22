#include <arun_reg/arun_reg.h>

ArunReg::ArunReg() : Node("arun_reg")
{
    // setup robot points
    p_r_[0] << 1, 0, 0;
    p_r_[1] << 0, 1, 0;
    p_r_[2] << 0, 0, 1;
    
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
    Eigen::Vector3d w_centroid = computeCentroid(p_w_);
    Eigen::Vector3d r_centroid = computeCentroid(p_r_);

    for (int i = 0; i< N_; ++i)
    {
        q_w_[i] = p_w_[i] -  w_centroid;
        q_r_[i] = p_r_[i] -  r_centroid;
    }

    createH();

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H_, Eigen::ComputeFullU | Eigen::ComputeFullV);
    R_ = svd.matrixV()*svd.matrixU().transpose();

    if (std::abs(R_.determinant() - 1) < 0.001)
    {
        p_rw_ = r_centroid - R_*w_centroid;
        RCLCPP_INFO(this->get_logger(), "FRE: %0.4f, TRE: %0.4f", computeFRE(), computeTRE());
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Determinate is incorrect: %0.4f", R_.determinant());
    }
}

Eigen::Vector3d ArunReg::computeCentroid(std::array<Eigen::Vector3d,3> & points)
{
    // int N = points.length();
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (int i = 0; i< N_; ++i)
    {
        centroid = centroid + points[i];
    }
    return centroid/N_;
}

void ArunReg::createH()
{
    // int N = points.length();
    for (int i = 0; i< N_; ++i)
    {
        H_ = H_ + q_w_[i]*(q_r_[i].transpose());
    }
}

double ArunReg::computeFRE()
{
    // int N = points.length();
    double FRE = 0;
    for (int i = 0; i< N_; ++i)
    {
        FRE = FRE + std::pow((p_r_[i] - (R_*p_w_[i] + p_rw_)).norm(), 2);
    }
    return std::sqrt(FRE);
}

double ArunReg::computeTRE()
{
    // int N = points.length();
    return (p_r_new_ - (R_*p_w_new_ + p_rw_)).norm();
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