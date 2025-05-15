/*
 * Filename: /home/yukan/Documents/work/local_mapping/include/matrix_plot.h
 * Path: /home/yukan/Documents/work/local_mapping/include
 * Created Date: Thursday, May 15th 2025, 9:15:46 am
 * Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */

#pragma once
#include <Eigen/Eigen>
#include <iostream>

namespace apa_slam {
class MatrixPlot {
 public:
  MatrixPlot();
  static MatrixPlot& GetInstance();
  void PlotCovarianceMatrix(const Eigen::MatrixXd& covariance);

 private:
  double normalize(double val, double min, double max);
  const char* _colors[24] = {
      // Red to yellow gradient (9 colors)
      "\033[48;5;196m", "\033[48;5;202m", "\033[48;5;203m", "\033[48;5;208m",
      "\033[48;5;209m", "\033[48;5;214m", "\033[48;5;215m", "\033[48;5;220m",
      "\033[48;5;221m",

      // Yellow-green gradient (6 colors)
      "\033[48;5;226m", "\033[48;5;190m", "\033[48;5;154m", "\033[48;5;118m",
      "\033[48;5;82m", "\033[48;5;46m",

      // Additional blue tones (9 colors)
      "\033[48;5;21m", "\033[48;5;27m", "\033[48;5;33m", "\033[48;5;39m",
      "\033[48;5;45m", "\033[48;5;51m", "\033[48;5;50m", "\033[48;5;49m",
      "\033[48;5;48m"};
  ;
  int _num_colors;
};
}  // namespace apa_slam
