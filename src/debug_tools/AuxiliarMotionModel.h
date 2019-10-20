
#include <Eigen/Dense>
#include <vector>
#include "sensors/EKF.h"

#ifndef AuxiliarMotionModel_H
#define AuxiliarMotionModel_H

	
class AuxiliarMotionModel{

 public:
  AuxiliarMotionModel(int max_predictions);

  void initialize();
  void restart();

  Eigen::Matrix4d predict_pose(const Eigen::Matrix4d &last_pose);
  void update(const Eigen::Matrix4d &visual_pose, std::vector<double> &measurements);




 private:
  SD_SLAM::EKF* motion_model_;
  Eigen::Matrix4d predicted_pose_;
  int predictions_count_;
  int MAX_PREDICTIONS_;  // until use a visual pose to update model

};

#endif // AuxiliarMotionModel_H