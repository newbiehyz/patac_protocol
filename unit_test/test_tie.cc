/*
 * Filename: /home/yukan/Documents/work/local_mapping/unit_test/test_tie.cc
 * Path: /home/yukan/Documents/work/local_mapping/unit_test
 * Created Date: Thursday, June 5th 2025, 10:44:08 am
 * Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */
#include <vector>
#include <iostream>
#include <tuple>
int main() {
  std::vector<int> x = {1, 2, 3, 5, 8, 9};

  for (size_t i = 0; i < x.size(); ++i) {
    for (size_t j = i + 1; j < x.size(); ++j) {
      auto [a, b] = std::tie(x[i], x[j]);
      std::cout << "Pair: " << a << ", " << b << "\n";
      // 处理关系
    }
  }


  auto [a, b] = std::tie(x[0], x[1]);
  auto [aa, bb] = std::tie(x[1], x[0]);

  std::cout << a << " " << b << std::endl;\
  std::cout << aa << " " << bb << std::endl;


}