/*
 * Filename:
 * /home/yukan/Documents/work/local_mapping/include/parking_slot_tracker.h Path:
 * /home/yukan/Documents/work/local_mapping/include Created Date: Friday, May
 * 9th 2025, 4:02:29 pm Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */

#include "tracker_base.h"

namespace apa_slam {
class ParkingSlotTracker : public TrackerBase {
 public:
  typedef std::shared_ptr<ParkingSlotTracker> Ptr;
  ParkingSlotTracker();
  std::vector<int> HungarianMatching(const std::vector<SemanticMea::Ptr>& meas,
                                     const Pose& pose);

 private:
  Eigen::VectorXd compute_matching_distance(const SemanticMea::Ptr mea0,
                                            const Pose& pose0,
                                            const SemanticMea::Ptr mea1,
                                            const Pose& pose1);
};
}  // namespace apa_slam
