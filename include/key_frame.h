#pragma once
#include <Eigen/Eigen>
#include <iostream>

namespace apa_slam {

class KeyFrame {
public:
    KeyFrame();
    KeyFrame(long long timestamp, const Eigen::VectorXd& pose, const Eigen::MatrixXd& covariance);
    KeyFrame(const KeyFrame& other);
    
    long long GetTimestamp() const;
    const Eigen::VectorXd& GetPose() const;
    const Eigen::MatrixXd& GetCovariance() const;
    
    void SetTimestamp(long long timestamp);
    void SetPose(const Eigen::VectorXd& pose);
    void SetCovariance(const Eigen::MatrixXd& covariance);
    void SetKeyFrameData(long long timestamp, const Eigen::VectorXd& pose, const Eigen::MatrixXd& covariance);

private:
    long long timestamp_;
    Eigen::VectorXd pose_;
    Eigen::MatrixXd covariance_;
};

} // namespace apa_slam