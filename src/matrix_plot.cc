/*
 * Filename: /home/yukan/Documents/work/local_mapping/src/matrix_plot.cc
 * Path: /home/yukan/Documents/work/local_mapping/src
 * Created Date: Thursday, May 15th 2025, 9:21:04 am
 * Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */
#include "matrix_plot.h"

namespace apa_slam {
MatrixPlot::MatrixPlot() {
  
  _num_colors = sizeof(_colors) / sizeof(_colors[0]);
}

MatrixPlot& MatrixPlot::GetInstance() {
  static MatrixPlot instance;
  return instance;
}

double MatrixPlot::normalize(double val, double min, double max) {
  return (val - min) / (max - min);
}

void MatrixPlot::PlotCovarianceMatrix(const Eigen::MatrixXd& covariance) {
  double min_val = covariance.minCoeff();
  double max_val = covariance.maxCoeff();
  for (int row = 0; row < covariance.rows(); ++row) {
    for (int col = 0; col < covariance.cols(); ++col) {
      double norm = normalize(covariance(row, col), min_val, max_val);
      int color_idx = static_cast<int>(norm * (_num_colors - 1));
      color_idx = std::max(0, std::min(_num_colors - 1, color_idx));
      printf("%s  \033[0m", _colors[color_idx]);

      if (row == col) {
        // 对角线用白色背景+黑色文字
        // printf("\033[48;5;255m\033[38;5;0m%4.2f\033[0m ", covariance(row,
        // col));
      } else {
        // 非对角线元素用颜色块
        // printf("%s  \033[0m", _colors[color_idx]);
      }
    }
    std::cout << "\n";
  }

  // 打印图例
  std::cout << "\nColor Legend:\n";
  for (int i = 0; i < _num_colors; ++i) {
    double legend_val = min_val + (max_val - min_val) * i / (_num_colors - 1);
    printf("%s  \033[0m %.2f", _colors[i], legend_val);
    if (i % 5 == 4) std::cout << std::endl;
  }
  std::cout << std::endl;
}

}  // namespace apa_slam
