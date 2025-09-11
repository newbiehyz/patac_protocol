/*
 * Filename: /home/yukan/Documents/work/patac_protocol/data_writer.h
 * Path: /home/yukan/Documents/work/patac_protocol
 * Created Date: Monday, July 14th 2025, 7:11:21 pm
 * Author: yukan
 *
 * Copyright (c) 2025 Your Company
 */

#pragma once
#include <Eigen/Eigen>
#include <memory>
#include <opencv2/opencv.hpp>


namespace apa_slam {

enum DataType : std::uint8_t { DataTypePose = 0, DataTypeDR = 1, DataTypeSLOT = 2, DataTypeFISHEYE = 3, DataTypeIPM = 4,DataTypeSEG = 5,DataTypePTC = 6 };

} // namespace apa_slam