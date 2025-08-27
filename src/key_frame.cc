#include "key_frame.h"

namespace apa_slam {

KeyFrame::KeyFrame() : timestamp_(0) {}

KeyFrame::KeyFrame(long long timestamp, const Eigen::VectorXd& pose, const Eigen::MatrixXd& covariance)
    : timestamp_(timestamp), pose_(pose), covariance_(covariance) {}

KeyFrame::KeyFrame(const KeyFrame& other)
    : timestamp_(other.timestamp_), pose_(other.pose_), covariance_(other.covariance_) {}

long long KeyFrame::GetTimestamp() const {
    return timestamp_;
}

const Eigen::VectorXd& KeyFrame::GetPose() const {
    return pose_;
}

const Eigen::MatrixXd& KeyFrame::GetCovariance() const {
    return covariance_;
}

void KeyFrame::SetTimestamp(long long timestamp) {
    timestamp_ = timestamp;
}

void KeyFrame::SetPose(const Eigen::VectorXd& pose) {
    pose_ = pose;
}

void KeyFrame::SetCovariance(const Eigen::MatrixXd& covariance) {
    covariance_ = covariance;
}

void KeyFrame::SetKeyFrameData(long long timestamp, const Eigen::VectorXd& pose, const Eigen::MatrixXd& covariance) {
    timestamp_ = timestamp;
    pose_ = pose;
    covariance_ = covariance;
}
} // namespace apa_slam