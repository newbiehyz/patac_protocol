#include <iostream>
#include <fstream>
#include <string>

#include "protocol/patac_image.pb.h"
#include "protocol/patac_trajetory.pb.h"
#include "protocol/patac_slot.pb.h"
#include "protocol/patac_dr.pb.h"

using namespace patac_hpp;

// ========================== RLE压缩/解压函数 ==========================
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

// ========================== 主函数 ==========================
int main() {
    std::cout << "========== PATAC TEST ==========" << std::endl;

    // --------- PATAC TEST1: Image ---------
    Image image;
    std::string raw_data(1880 * 1650 * 3, 0);
    std::string compressed_data = RLECompress(raw_data);
    image.set_data(compressed_data);
    image.set_width(1880);
    image.set_height(1650);
    image.set_pos(Position::FRONT);
    image.set_type(Type::RGB);
    image.set_mode(Mode::CYL);

    {
        std::ofstream ofs("test_img.bin", std::ios::binary);
        image.SerializeToOstream(&ofs);
    }

    Image image_out;
    {
        std::ifstream ifs("test_img.bin", std::ios::binary);
        image_out.ParseFromIstream(&ifs);
    }

    std::cout << "[Image] width: " << image_out.width()
              << ", height: " << image_out.height()
              << ", type: " << image_out.type()
              << ", decompressed data size: " << RLEDecompress(image_out.data()).size()
              << std::endl;

    // --------- PATAC TEST2: Trajectory ---------
    Trajectory traj;
    traj.set_num_trajectory_point(3);
    for (int i = 0; i < 3; ++i) {
        auto* p = traj.add_trajectory_point_list();
        p->set_id(i);
        p->set_timestamp(i * 100);
        p->set_x(i * 1.0f);
        p->set_y(i * 2.0f);
        p->set_z(i * 3.0f);
    }
    {
        std::ofstream ofs("test_traj.bin", std::ios::binary);
        traj.SerializeToOstream(&ofs);
    }

    Trajectory traj_out;
    {
        std::ifstream ifs("test_traj.bin", std::ios::binary);
        traj_out.ParseFromIstream(&ifs);
    }

    std::cout << "[Trajectory] num: " << traj_out.num_trajectory_point() << std::endl;
    for (const auto& pt : traj_out.trajectory_point_list()) {
        std::cout << "  ID: " << pt.id() << ", Time: " << pt.timestamp()
                  << ", Pos: (" << pt.x() << ", " << pt.y() << ", " << pt.z() << ")" << std::endl;
    }

    // --------- PATAC TEST3: ParkingSlotList ---------
    ParkingSlotList slot_list;
    slot_list.set_num_parking_slot(2);
    for (int i = 0; i < 2; ++i) {
        auto* slot = slot_list.add_parking_slot_list();
        slot->set_id(i);
        slot->set_type(ParkingSlotType::SlotTypeVertical);
        slot->set_occupancy(OccupancyStatus::OccupancyStatusOccupied);
        slot->set_source(ParkingSource::ParkingSourceUserSel);
        slot->set_lock_status(ParkingLockStatus::ParkingLockStatusOpen);
        for (int j = 0; j < 4; ++j) {
            auto* pt = slot->add_points();
            pt->set_x(j);
            pt->set_y(j * 2);
        }
    }
    {
        std::ofstream ofs("test_slot.bin", std::ios::binary);
        slot_list.SerializeToOstream(&ofs);
    }

    ParkingSlotList slot_list_out;
    {
        std::ifstream ifs("test_slot.bin", std::ios::binary);
        slot_list_out.ParseFromIstream(&ifs);
    }

    std::cout << "[ParkingSlotList] num: " << slot_list_out.num_parking_slot() << std::endl;
    for (const auto& s : slot_list_out.parking_slot_list()) {
        std::cout << "  ID: " << s.id() << ", type: " << s.type()
                  << ", occupancy: " << s.occupancy() << ", points: ";
        for (const auto& pt : s.points()) {
            std::cout << "(" << pt.x() << ", " << pt.y() << ") ";
        }
        std::cout << std::endl;
    }

    // --------- PATAC TEST4: DRPose ---------
    DRPose dr;
    dr.set_timestamp(999999);
    dr.set_x(1.1f);
    dr.set_y(2.2f);
    dr.set_z(3.3f);
    dr.set_yaw(0.12f);
    dr.set_velocity(4.4f);
    dr.set_angular_velocity(5.5f);

    {
        std::ofstream ofs("test_dr.bin", std::ios::binary);
        dr.SerializeToOstream(&ofs);
    }

    DRPose dr_out;
    {
        std::ifstream ifs("test_dr.bin", std::ios::binary);
        dr_out.ParseFromIstream(&ifs);
    }

    std::cout << "[DRPose] timestamp: " << dr_out.timestamp()
              << ", pos: (" << dr_out.x() << ", " << dr_out.y() << ", " << dr_out.z() << ")"
              << ", yaw: " << dr_out.yaw()
              << ", velocity: " << dr_out.velocity()
              << ", angular_velocity: " << dr_out.angular_velocity()
              << std::endl;

    return 0;
}
