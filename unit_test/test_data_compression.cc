/*
 * Filename:
 * /home/yukan/Documents/work/local_mapping/unit_test/test_data_compression.cc
 * Path: /home/yukan/Documents/work/local_mapping/unit_test
 * Created Date: Saturday, June 7th 2025, 11:08:28 am
 * Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */

#include <Eigen/Dense>
#include <chrono>
#include <iostream>
#include <random>
using namespace Eigen;

// 生成随机正定矩阵
MatrixXd generatePositiveDefiniteMatrix(int size) {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<double> dist(0.0, 1.0);

  MatrixXd A = MatrixXd::Zero(size, size);
  for (int i = 0; i < size; ++i) {
    for (int j = 0; j <= i; ++j) {
      A(i, j) = dist(gen);
      if (i != j) A(j, i) = A(i, j);  // 对称
    }
  }
  return A * A.transpose() + MatrixXd::Identity(size, size);  // 保证正定
}

// 生成稀疏观测矩阵 (m x n)
MatrixXd generateSparseObservationMatrix(int m, int n, double sparsity = 0.9) {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> dist(0.0, 1.0);
  std::normal_distribution<double> noise(0.0, 0.1);

  MatrixXd H = MatrixXd::Zero(m, n);
  for (int i = 0; i < m; ++i) {
    for (int j = 0; j < n; ++j) {
      if (dist(gen) > sparsity) {  // 控制稀疏性
        H(i, j) = noise(gen);
      }
    }
  }
  return H;
}

int main() {
  // 参数设置
  const int n = 20;   // 状态维度 (e.g., [x, y, z, vx, vy, vz])
  const int m = 400;  // 观测维度

  // 1. 生成状态向量 x (n x 1)
  VectorXd x = VectorXd::LinSpaced(n, 0.0, 1.0);  // 线性递增值

  // 2. 生成状态协方差矩阵 P (n x n)
  MatrixXd P = generatePositiveDefiniteMatrix(n);

  // 3. 生成观测矩阵 H (m x n)
  MatrixXd H = generateSparseObservationMatrix(m, n, 0.9);

  // 4. 生成观测噪声矩阵 R (m x m, 对角)
  VectorXd R_diag = VectorXd::Constant(m, 0.01);  // 噪声方差 = 0.01
  MatrixXd R = R_diag.asDiagonal();

  // 5. 生成残差 residual (m x 1)
  VectorXd residual = VectorXd::Random(m) * 0.1;  // 小随机残差

  // 打印关键数据
  std::cout << "State x:\n" << x.transpose() << "\n\n";
  std::cout << "Covariance P (top-left 3x3):\n"
            << P.topLeftCorner(3, 3) << "...\n\n";
  std::cout << "Observation matrix H (top 5x3):\n"
            << H.topLeftCorner(5, 3) << "...\n\n";
  std::cout << "Residual (first 10 elements):\n"
            << residual.head(10).transpose() << "...\n";

  Eigen::MatrixXd S = H * P * H.transpose() + R;
  auto start = std::chrono::steady_clock::now();

  Eigen::MatrixXd Sinv = S.inverse();
  std::cout << "Sinv " << Sinv.rows() << " " << Sinv.cols() << std::endl;

  auto end = std::chrono::steady_clock::now();
  auto duration =
      std::chrono::duration_cast<std::chrono::microseconds>(end - start)
          .count();

  std::cout << "Update Duration: " << duration << " MicroSeconds" << std::endl;
  Eigen::MatrixXd K = P * H.transpose() * Sinv;

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      H, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::MatrixXd U = svd.matrixU();  // m x n
  Eigen::VectorXd z_compressed = U.transpose() * residual;
  Eigen::MatrixXd H_compressed =
      svd.singularValues().asDiagonal() * svd.matrixV().transpose();
  Eigen::MatrixXd R_compressed = U.transpose() * R * U;

  Eigen::MatrixXd S_compressed =
      H_compressed * P * H_compressed.transpose() + R_compressed;
  auto start_compressed = std::chrono::steady_clock::now();
  Eigen::MatrixXd Sinv_compressed = S_compressed.inverse();

  std::cout << "Sinv_compressed " << Sinv_compressed.rows() << " "
            << Sinv_compressed.cols() << std::endl;
  auto end_compressed = std::chrono::steady_clock::now();
  auto duration_compressed =
      std::chrono::duration_cast<std::chrono::microseconds>(end_compressed -
                                                            start_compressed)
          .count();

  std::cout << "Update_compressed Duration: " << duration_compressed
            << " MicroSeconds" << std::endl;
  Eigen::MatrixXd K_compressed = P * H_compressed.transpose() * Sinv_compressed;
  // Eigen::MatrixXd K = P * H_compressed.transpose() * Sinv;

  // std::cout << "dx: " << (K * residual).transpose() << std::endl;

  std::cout << (K * residual).transpose() << std::endl;
  std::cout << (K_compressed * z_compressed).transpose() << std::endl;
  
      // x = x + K * residual;
      // P = P - K * (H * P * H.transpose() + R) * K.transpose();

      return 0;
}