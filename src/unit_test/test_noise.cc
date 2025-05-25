/*
 * Filename: /home/yukan/Documents/work/local_mapping/unit_test/test_noise.cc
 * Path: /home/yukan/Documents/work/local_mapping/unit_test
 * Created Date: Thursday, May 15th 2025, 4:51:21 pm
 * Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */
#include <Eigen/Eigen>
#include <iostream>
#include <random>
int main() {
  std::default_random_engine generator;
  std::normal_distribution<double> dist(0.0, 1.0);

  // std::cout << "Noise Ps\n" << noise << std::endl;

  for (int i = 0; i < 10000; ++i) {
    Eigen::VectorXd stdDev = Eigen::VectorXd::Zero(2);
    stdDev[0] = 0.1;
    stdDev[1] = 0.1;
    Eigen::Matrix4d noise;
    for (int i = 0; i < 2; ++i) {
      for (int j = 0; j < 2; ++j) {
        noise(i, j) = stdDev(j) * dist(generator);
      }
    }
    std::cout << noise << std::endl;
  }

  return 0;
}