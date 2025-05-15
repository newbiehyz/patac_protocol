#include <Eigen/Eigen>
#include <iostream>
#include "matrix_plot.h"

int main(int argc, char** argv) {
  int n = 20;
  Eigen::MatrixXd X = Eigen::MatrixXd::Random(n, n);
  X = X.array().abs();
  Eigen::MatrixXd cov = X * X.transpose();

  apa_slam::MatrixPlot::GetInstance().PlotCovarianceMatrix(cov);

  // std::cout << cov << std::endl;
  return 0;
}