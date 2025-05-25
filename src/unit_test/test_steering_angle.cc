/*
 * Filename:
 * /home/yukan/Documents/work/local_mapping/unit_test/test_steering_angle.cc
 * Path: /home/yukan/Documents/work/local_mapping/unit_test
 * Created Date: Monday, May 19th 2025, 10:07:00 am
 * Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */

#include <cmath>
#include <iostream>
#define R_N_557 16
static float RADIUS_557_TAB[4 * R_N_557] = {
    // LF
    4.712f, 5.175f, 5.596f, 6.084f, 6.658f, 7.324f, 8.049f, 8.867f, 9.855f, 11.052f, 12.523f, 14.445f, 16.949f, 20.488f, 25.616f, 0.0f,
    // RF
    4.732f, 5.136f, 5.571f, 6.094f, 6.668f, 7.314f, 8.048f, 8.901f, 9.887f, 11.087f, 12.579f, 14.494f, 17.033f, 20.590f, 25.865f, 0.0f,
    // LR
    4.748f, 5.186f, 5.651f, 6.161f, 6.722f, 7.351f, 8.072f, 8.908f, 9.910f, 11.089f, 12.569f, 14.400f, 16.857f, 20.273f, 25.568f, 0.0f,
    // RR
    4.750f, 5.186f, 5.633f, 6.147f, 6.707f, 7.337f, 8.057f, 8.904f, 9.894f, 11.080f, 12.586f, 14.450f, 16.947f, 20.419f, 25.585f, 0.0f
};



float getRadiusFromSteering(float steering_angle_deg, int wheel_idx) {
  const int N = 16;
    const float radius_angles[N] = {
        540, 510, 480, 450, 420, 390, 360, 330,
        300, 270, 240, 210, 180, 150, 120, 0
    };

    if (steering_angle_deg < 0.0f || steering_angle_deg > 540.0f)
        return 0.0f;

    // 找到 angle 所在的区间
    int i = 0;
    while (i < N - 1 && steering_angle_deg < radius_angles[i + 1]) {
        ++i;
    }

    // 防止越界
    if (i >= N - 1) return RADIUS_557_TAB[wheel_idx * N + N - 1];

    float angle_i = radius_angles[i];
    float angle_ip1 = radius_angles[i + 1];
    float radius_i = RADIUS_557_TAB[wheel_idx * N + i];
    float radius_ip1 = RADIUS_557_TAB[wheel_idx * N + i + 1];

    float t = (steering_angle_deg - angle_i) / (angle_ip1 - angle_i);
    return radius_i + t * (radius_ip1 - radius_i);
}

int main() {
  float angle = 50.0f;
  float radius_lf = getRadiusFromSteering(angle, 0);  // LF = index 0
  float radius_rf = getRadiusFromSteering(angle, 1);  // RF = index 1
  float radius_lr = getRadiusFromSteering(angle, 2);  // LR = index 2
  float radius_rr = getRadiusFromSteering(angle, 3);  // RR = index 3

  std::cout << "Steering angle: " << angle << " deg\n";
  std::cout << "Radius LF: " << radius_lf << " m\n";
  std::cout << "Radius RF: " << radius_rf << " m\n";
  std::cout << "Radius LR: " << radius_lr << " m\n";
  std::cout << "Radius RR: " << radius_rr << " m\n";
  return 0;
}