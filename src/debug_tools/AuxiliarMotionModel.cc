
#include "debug_tools/AuxiliarMotionModel.h"
#include <Eigen/Dense>
#include <vector>
#include "sensors/EKF.h"
#include "sensors/IMU.h"


AuxiliarMotionModel::AuxiliarMotionModel(int max_predictions){

  SD_SLAM::Sensor *sensor_model = new SD_SLAM::IMU();
  //SD_SLAM::Sensor *sensor_model = new SD_SLAM::PureIMU();

  motion_model_ = new SD_SLAM::EKF(sensor_model);
  predicted_pose_ = Eigen::Matrix4d::Identity();
  predictions_count_ = 0;
  MAX_PREDICTIONS_ = max_predictions;  // until use a visual pose to update model
}

void AuxiliarMotionModel::restart(){
  motion_model_->Restart();
}

Eigen::Matrix4d AuxiliarMotionModel::predict_pose(const Eigen::Matrix4d &last_pose){
  predicted_pose_ = motion_model_->Predict(last_pose);
  return predicted_pose_;
}

void AuxiliarMotionModel::update(const Eigen::Matrix4d &visual_pose, std::vector<double> &measurements){

  if (predictions_count_ <= MAX_PREDICTIONS_){
    std::cout << "[AUX MOTION MODEL]: UPDATEING WITH SAME POSE" << std::endl;
    motion_model_->Update(predicted_pose_, measurements);
    predictions_count_++;
  }
  else{
    std::cout << "[AUX MOTION MODEL]: UPDATEING WITH SAME VISUAL POSE" << std::endl;
    // motion_model_->Update(visual_pose, measurements);
    restart();
    predictions_count_ = 0;
  }

}

