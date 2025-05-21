/*
 * Filename:
 * /home/yukan/Documents/work/local_mapping/include/parking_slot_tracker.h Path:
 * /home/yukan/Documents/work/local_mapping/include Created Date: Friday, May
 * 9th 2025, 4:02:29 pm Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */

#include "apa_parameters.h"
#include "hungarian.h"
#include "local_mapping_define.h"
#include "semantic_map.h"
#include "tracker_base.h"
namespace apa_slam {
class ParkingSlotTracker : public TrackerBase {
 public:
  typedef std::shared_ptr<ParkingSlotTracker> Ptr;
  ParkingSlotTracker();
  std::vector<int> HungarianMatching(const std::vector<SemanticMea::Ptr>& meas,
                                     const Pose& pose);

 private:
  std::vector<int> matching_with_last_mea(
      const std::vector<SemanticMea::Ptr>& meas, const Pose& pose);
  std::vector<int> matching_with_local_map(
      const std::vector<SemanticMea::Ptr>& meas, const Pose& pose);
  Eigen::VectorXd compute_matching_distance(const SemanticMea::Ptr mea0,
                                            const Pose& pose0,
                                            const SemanticMea::Ptr mea1,
                                            const Pose& pose1);
  std::vector<int> get_consist_matching(
      const std::vector<int>& matching_mea,
      const std::vector<int>& matching_map = {});

  std::pair<double, double> get_two_smallest(const std::vector<double>& vec);

  std::vector<int> fill_matching(const std::vector<int>& matching);
};
}  // namespace apa_slam
