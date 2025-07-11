#include <foundation/message/serializer.h>

#include <iostream>
#include <memory>

#include "interface/parking/ipm_multitask_data.h"
#include "interface/parking/serializer/ipm_multitask_serializer.h"
#include "interface/parking/processor_info_data.h"
#include "interface/parking/serializer/processor_info_serializer.h"
#include "interface/parking/uss_parking_slots_data.h"
#include "ipm_multitask.pb.h"
#include "model/camera_data.h"
#include "model/gnss_data.h"
#include "model/serializer/camera_serializer.h"
#include "processor_info.pb.h"
#include "protocol/camera.pb.h"
#include "protocol/frame.pb.h"
#include "interface/parking/uss_tracking_mode_data.h"
#include "interface/parking/serializer/uss_tracking_mode_serializer.h"
#include "uss_tracking_mode.pb.h"
#include "interface/parking/serializer/fusion_system_serializer.h"
#include "interface/parking/fusion_system_data.h"
#include "protocol/fusion_system.pb.h"


#include "protocol/patac_image.pb.h"
#include "protocol/patac_trajetory.pb.h"
#include "protocol/patac_slot.pb.h"
#include "protocol/patac_dr.pb.h"

using namespace autodrive;
using namespace patac_hpp;

std::string RLECompress(const std::string& input) {
    std::string output;
    size_t i = 0;
    while (i < input.size()) {
        uint8_t current = input[i];
        size_t count = 1;
        while (i + count < input.size() && input[i + count] == current && count < 255) {
            count++;
        }
        output.push_back(static_cast<char>(count));
        output.push_back(static_cast<char>(current));
        i += count;
    }
    return output;
}

std::string RLEDecompress(const std::string& input) {
    std::string output;
    for (size_t i = 0; i + 1 < input.size(); i += 2) {
        uint8_t count = static_cast<uint8_t>(input[i]);
        uint8_t value = static_cast<uint8_t>(input[i + 1]);
        output.append(count, static_cast<char>(value));
    }
    return output;
}


int main() {
  autodrive::ProcessorInfoData data;
  data.GetProcessorInfo();

  autodrive::UssParkingSlotsData data2;
  data2.GetUssParkingSlots();

  autodrive::GnssData data3;
  data3.GetGnss();
  auto& gnss = data3.MutableGnss();
  gnss.utc_time = 1;
  std ::cout << gnss.utc_time << std::endl;

  // Deserialize |proto| -> |class|
  workflow::proto::IpmMultitaskMsg ipm_multitask_msg;
  auto* slot_raw = ipm_multitask_msg.add_slots_raw();
  slot_raw->set_slot_type(
      workflow::proto::ParkingSlotType::kParkingSlotTypeOblique);
  IpmMultitaskData ipm_multitask_data;
  message::DeserializeDataMsg<workflow::proto::IpmMultitaskMsg> proto_msg(
      &ipm_multitask_msg);
  message::SerializeDataMsg<IpmMultitaskData> data_msg(&ipm_multitask_data);
  IpmMultitaskSerializer serializer;
  serializer.Deserialize(&proto_msg, &data_msg);

  // Expect: IpmMultitaskData deserialize success, parking slot size:1, type: 3
  std::cout << "IpmMultitaskData deserialize success, parking slot size:"
            << static_cast<std::uint32_t>(ipm_multitask_data.GetSlotsNum())
            << ", type: "
            << static_cast<std::uint32_t>(
                   ipm_multitask_data.GetSlotsRaw()[0].slot_type)
            << std::endl;

  // Serialize |class| -> |proto|
  ProcessorInfoData processor_info_data;
  auto& process_info = processor_info_data.MutableProcessorInfo();
  process_info.error_code = ProcessorInfoData::ErrorCode::kImageSyncFailed;
  process_info.status = ProcessorInfoData::ProcessorStatus::kError;
  workflow::proto::ProcessorInfoMsg processor_info_msg;
  message::DeserializeDataMsg<workflow::proto::ProcessorInfoMsg> proto_msg1(
      &processor_info_msg);
  message::SerializeDataMsg<ProcessorInfoData> data_msg1(&processor_info_data);
  ProcessorInfoSerializer serializer1;
  serializer1.Serialize(&data_msg1, &proto_msg1);

  // Expect: ProcessorInfoData serialize success, error code: 1, status: 3
  std::cout << "ProcessorInfoData serialize success, error code: "
            << static_cast<std::uint32_t>(processor_info_msg.error_code()) << ", status: "
            << static_cast<std::uint32_t>(processor_info_msg.status())
            << std::endl;

  workflow::proto::FrameMsg frame_msg;
  frame_msg.mutable_frame()->set_topic("FrameTopic");
  // Expected: Frame topic: FrameTopic
  std::cout << "Frame topic: " << frame_msg.frame().topic() << std::endl;

  CameraData camera_data;
  camera_data.MutableImage().height = 1000;
  workflow::proto::CameraMsg camera_data_msg;
  message::DeserializeDataMsg<workflow::proto::CameraMsg> proto_msg2(
      &camera_data_msg);
  message::SerializeDataMsg<CameraData> data_msg2(&camera_data);
  CameraSerializer serializer2;
  serializer2.Serialize(&data_msg2, &proto_msg2);
  // Expected: CameraData serialize success, height: 1000
  std::cout << "CameraData serialize success, height: "
            << camera_data_msg.image().height() << std::endl;
  
  // UssTrackingModeData Serialize/Deserialize
  UssTrackingModeData utm_in_data;
  auto& utm_in = utm_in_data.MutableUssTrackingMode();
  utm_in.plv_mode = PlvState::kPlvStateActive; // 2
  utm_in.psu_mode = PsuState::kPsuStateMax; // 3
  utm_in.uss_ps_clear_distance = 10240;
  utm_in.odo_mode = OdoState::kOdoStateReset; // 1
  auto& ps_correct = utm_in.ps_correct;
  ps_correct.uss_ps_id_ui8 = 3;
  ps_correct.uss_ps_status_en = UssParkingSlotsData::UssPsStatusEn::kUssPsStatusEnSearch; // 1
  auto& uss_point_si16 = ps_correct.uss_ps_boundary_1st_info;
  uss_point_si16.X_si16 = 7;
  uss_point_si16.Y_si16 = 8;
  ps_correct.uss_ps_finish_timestamp = 4096;

  UssTrackingModeDataSerializer utm_serializer;
  // Serialize
  workflow::proto::UssTrackingModeMsg utm_msg;
  message::DeserializeDataMsg<workflow::proto::UssTrackingModeMsg> utm_proto_msg(
      &utm_msg);
  message::SerializeDataMsg<UssTrackingModeData> utm_data_in_msg(&utm_in_data);
  utm_serializer.Serialize(&utm_data_in_msg, &utm_proto_msg);

  // Deserialize
  UssTrackingModeData utm_out_data;
  message::SerializeDataMsg<UssTrackingModeData> utm_data_out_msg(&utm_out_data);
  utm_serializer.Deserialize(&utm_proto_msg, &utm_data_out_msg);
  auto utm_out = utm_out_data.GetUssTrackingMode();
  auto ps_correct_out = utm_out.ps_correct;
  auto uss_point_si16_out = ps_correct_out.uss_ps_boundary_1st_info;
  std::cout << "UssTrackingModeData plv_mode: " <<  static_cast<std::uint32_t>(utm_out.plv_mode)
            << ", psu_mode: " <<  static_cast<std::uint32_t>(utm_out.psu_mode)
            << ", uss_ps_clear_distance: " << static_cast<std::uint32_t>(utm_out.uss_ps_clear_distance)
            << ", odo_mode: " << static_cast<std::uint32_t>(utm_out.odo_mode)
            << ", uss_ps_id_ui8: " << static_cast<std::uint32_t>(ps_correct_out.uss_ps_id_ui8)
            << ", uss_ps_status_en: " << static_cast<std::uint32_t>(ps_correct_out.uss_ps_status_en)
            << ", X_si16: " << static_cast<std::uint32_t>(uss_point_si16_out.X_si16)
            << ", Y_si16: " << static_cast<std::uint32_t>(uss_point_si16_out.Y_si16)
            << ", uss_ps_finish_timestamp: " << ps_correct_out.uss_ps_finish_timestamp
            << std::endl;
  FusionSystemData fusion_system_data;
  auto& fusion_system = fusion_system_data.MutableFusionSystem();
  fusion_system.usr_sel_ps.usersel_parking_slot.num_polygon_points = 4;
  FusionSystemSerializer fusion_system_serializer;
  // Serialize
  workflow::proto::FusionSystemMsg fusion_system_msg;
  message::DeserializeDataMsg<workflow::proto::FusionSystemMsg> fusion_system_proto_msg(
      &fusion_system_msg);
  message::SerializeDataMsg<FusionSystemData> fusion_system_data_in_msg(&fusion_system_data);
  fusion_system_serializer.Serialize(&fusion_system_data_in_msg, &fusion_system_proto_msg);
  
  // ==================================================FIRST TEST
  IpmMultitaskData ipm_multitask_data3;
  ipm_multitask_data3.SetTest(8.8f);
  std::cout << "[Original] confidence = " << ipm_multitask_data3.GetTest() << std::endl;

  // ========== 序列化 C++ → Protobuf ==========
  workflow::proto::IpmMultitaskMsg ipm_multitask_msg3;
  message::SerializeDataMsg<IpmMultitaskData> data_msg3(&ipm_multitask_data3);
  message::DeserializeDataMsg<workflow::proto::IpmMultitaskMsg> proto_msg3(&ipm_multitask_msg3);
  
  IpmMultitaskSerializer serializer3;
  serializer3.Serialize(&data_msg3, &proto_msg3);  // 执行序列化

  // ========== 反序列化 Protobuf → 新 C++ 对象 ==========
  IpmMultitaskData ipm_multitask_data_out;
  message::SerializeDataMsg<IpmMultitaskData> data_msg_out(&ipm_multitask_data_out);
  serializer.Deserialize(&proto_msg3, &data_msg_out);  // 执行反序列化

  std::cout << "[Deserialized] confidence = "
            << ipm_multitask_data_out.GetTest() << std::endl;

            


  std::cout<<std::endl<<"============= PATAC TEST ============="<<std::endl<<std::endl;         

  // ==================================================PATAC TEST1_image
  patac_hpp::Image image;
  std::string raw_data(1880 * 1650 * 3, 0);
  std::string compressed_data = RLECompress(raw_data);
  image.set_data(compressed_data);
  image.set_width(1880);
  image.set_height(1650);
  image.set_pos(patac_hpp::Position::FRONT);
  image.set_type(patac_hpp::Type::RGB);
  image.set_mode(patac_hpp::Mode::CYL);

  // Serialize
  {
        std::ofstream ofs("test_img.bin", std::ios::binary);
        image.SerializeToOstream(&ofs);
  }

  // Deserialize
  patac_hpp::Image image_out;
  {
        std::ifstream ifs("test_img.bin", std::ios::binary);
        if (!image_out.ParseFromIstream(&ifs)) {
            std::cerr << "Failed to parse image." << std::endl;
        }
  }

  // Decompress
  std::string decompressed_data = RLEDecompress(image_out.data());

  // Print
  std::cout << "width: " << image_out.width() << ", height: " << image_out.height() << std::endl;
  std::cout << "type: " << image_out.type() << ", data size: " << decompressed_data.size() << std::endl;



    // ==================================================PATAC TEST2_trajectory
    patac_hpp::Trajectory trajectory;
    trajectory.set_num_trajectory_point(5);
    for (int i = 0; i < 5; ++i) {
        auto* point = trajectory.add_trajectory_point_list();
        point->set_timestamp(i * 10);
        point->set_id(i);
        point->set_x(i * 0.1f);
        point->set_y(i * 0.2f);
        point->set_z(i * 0.3f);
        point->set_qx(i * 0.1f);
        point->set_qy(i * 0.2f);
        point->set_qz(i * 0.3f);
        point->set_qw(i * 0.4f);
        point->set_velocity(i * 0.5f);
        point->set_angular_velocity(i * 0.05f);
    }

    // Serialize
    {
            std::ofstream ofs("test_trajectory.bin", std::ios::binary);
            trajectory.SerializeToOstream(&ofs);
    }

    // Deserialize
    patac_hpp::Trajectory trajectory_out;
    {
            std::ifstream ifs("test_trajectory.bin", std::ios::binary);
            if (!trajectory_out.ParseFromIstream(&ifs)) {
                std::cerr << "Failed to parse trajectory." << std::endl;
            }
    }

    // Print
    std::cout << "num_trajectory_point: " << trajectory_out.num_trajectory_point()
                << ", trajectory_point_list size: " << trajectory_out.trajectory_point_list_size() << std::endl;
    for (const auto& point : trajectory_out.trajectory_point_list()) {
        std::cout << "  Point ID: " << point.id()
                    << ", Timestamp: " << point.timestamp()
                    << ", Position: (" << point.x() << ", " << point.y() << ", " << point.z() << ")"
                    << ", Quaternion: (" << point.qx() << ", " << point.qy() << ", " << point.qz() << ", " << point.qw() << ")"
                    << ", Velocity: " << point.velocity()
                    << ", Angular Velocity: " << point.angular_velocity() << std::endl;
    }


    // ==================================================PATAC TEST3_parking_slot_list
    patac_hpp::ParkingSlotList parking_slot_list;
    parking_slot_list.set_num_parking_slot(3);
    for (int i = 0; i < 3; ++i) {
        auto* slot = parking_slot_list.add_parking_slot_list();
        slot->set_id(i);
        slot->set_type(patac_hpp::ParkingSlotType::SlotTypeVertical);
        slot->set_occupancy(patac_hpp::OccupancyStatus::OccupancyStatusOccupied);
        slot->set_source(patac_hpp::ParkingSource::ParkingSourceIpm);
        slot->set_lock_status(patac_hpp::ParkingLockStatus::ParkingLockStatusOpen);
        for (int j = 0; j < 4; ++j) {
            auto* point = slot->add_points();
            point->set_x(j * 1.0f);
            point->set_y(j * 2.0f);
        }
    }

    // Serialize
    {
        std::ofstream ofs("test_parking_slot_list.bin", std::ios::binary);
        parking_slot_list.SerializeToOstream(&ofs);
    }

    // Deserialize
    patac_hpp::ParkingSlotList parking_slot_list_out;
    {
        std::ifstream ifs("test_parking_slot_list.bin", std::ios::binary);
        if (!parking_slot_list_out.ParseFromIstream(&ifs)) {
            std::cerr << "Failed to parse parking slot list." << std::endl;
        }
    }
    
    // Print
    std::cout << "num_parking_slot: " << parking_slot_list_out.num_parking_slot()
              << ", parking_slot_list size: " << parking_slot_list_out.parking_slot_list_size() << std::endl;
    for (const auto& slot : parking_slot_list_out.parking_slot_list()) {
        std::cout << "  Slot ID: " << slot.id()
                  << ", Type: " << static_cast<int>(slot.type())
                  << ", Occupancy Status: " << static_cast<int>(slot.occupancy())
                  << ", Parking Source: " << static_cast<int>(slot.source())
                  << ", Parking Lock Status: " << static_cast<int>(slot.lock_status())
                  << ", Points: ";
        for (const auto& point : slot.points()) {
            std::cout << "(" << point.x() << ", " << point.y() << ") ";
        }
        std::cout << std::endl;
    }


  // ==================================================PATAC TEST4_DR
  patac_hpp::DRPose dr_pose;
  dr_pose.set_timestamp(1234567890);
  dr_pose.set_x(1.0f);
  dr_pose.set_y(2.0f);
  dr_pose.set_z(3.0f);
  dr_pose.set_yaw(0.1f);
  dr_pose.set_velocity(0.2f);
  dr_pose.set_angular_velocity(0.3f);

  {
    std::ofstream ofs("test_dr_pose.bin", std::ios::binary);
    dr_pose.SerializeToOstream(&ofs);
  }

  patac_hpp::DRPose dr_pose_out;
  {
    std::ifstream ifs("test_dr_pose.bin", std::ios::binary);
    if (!dr_pose_out.ParseFromIstream(&ifs)) {
      std::cerr << "Failed to parse DR pose." << std::endl;
    }
  }

  std::cout << "DRPose - Timestamp: " << dr_pose_out.timestamp()
            << ", Position: (" << dr_pose_out.x() << ", " << dr_pose_out.y() << ", " << dr_pose_out.z() << ")"
            << ", Yaw: " << dr_pose_out.yaw()
            << ", Velocity: " << dr_pose_out.velocity()
            << ", Angular Velocity: " << dr_pose_out.angular_velocity() << std::endl;


  return 1;
}
