/*
 * Filename:
 * /home/yukan/Documents/work/local_mapping/include/semantic_landmark.h Path:
 * /home/yukan/Documents/work/local_mapping/include Created Date: Thursday, May
 * 8th 2025, 3:50:36 pm Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */
#pragma once
#include <memory>

#include "local_mapping_define.h"
#include "semantic_measurement.h"
// #include "ekf_estimator.h"
namespace apa_slam {
class SemanticLandmark {
 public:
  typedef std::shared_ptr<SemanticLandmark> Ptr;
  SemanticLandmark(const SensorType& type, const int id);
  virtual ~SemanticLandmark() = default;
  SensorType GetSemanticType();

  int GetId();

  virtual Eigen::MatrixXd GetLandmarkData() = 0;

  virtual void AddSemanticMea(const long long timestmap, const Pose& mea_pose,
                              const SemanticMea::Ptr mea) = 0;

  bool Initialized();

  void SetInitializeFlag(const bool& flag);

  bool NeedMargin();

  void SetMarginFlag(const bool& flag);

  bool NeedUpdate();

  void SetUpdateFlag(const bool& flag);

  bool NeedInitialize();

  void SetNeedInitializeFlag(const bool& flag);

  void SetCov(const Eigen::MatrixXd& cov);

  virtual void SetMean(const Eigen::VectorXd& mean) = 0;

  Eigen::MatrixXd GetCov();

  const SemanticMea::Ptr GetLatestMea();

  void TagMarginalization(const double timestamp);

  virtual void InitializeLandmark(const Eigen::VectorXd& state,
                                  const Eigen::MatrixXd& P,
                                  Eigen::MatrixXd& Jx) = 0;

  virtual Eigen::VectorXd GetVectorizedData() = 0;

  virtual void GetResidualAndJacobian(const SemanticMea::Ptr& mea,
                                      const Eigen::VectorXd& v_state,
                                      Eigen::VectorXd& residual,
                                      Eigen::MatrixXd& J_v,
                                      Eigen::MatrixXd& J_lm) = 0;

  void GetLatestResidualAndJacobian(const Eigen::VectorXd& v_state,
                                    Eigen::VectorXd& residual,
                                    Eigen::MatrixXd& J_v,
                                    Eigen::MatrixXd& J_lm);

  virtual Eigen::VectorXd ComputeMatchingResidual(const Eigen::VectorXd& pose,
                                                  const SemanticMea::Ptr& mea) = 0;

 protected:
  std::map<long long, std::pair<Pose, SemanticMea::Ptr>> _meas;

 private:
  SensorType _type;
  int _id;
  bool _initialized;
  bool _margin;
  bool _update;
  bool _need_intialize;

  Eigen::MatrixXd _cov;
};
}  // namespace apa_slam
