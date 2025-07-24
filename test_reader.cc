#include <Eigen/Eigen>
#include "data_reader.h"
#include <iostream>

using namespace apa_slam;

void PrintDataSeq(const std::vector<std::pair<int64_t, DataType>>& data_seq) {
    std::cout << "Data Sequence:\n";
    for (const auto& item : data_seq) {
        std::cout << "Timestamp: " << item.first 
                  << ", Type: " << static_cast<int>(item.second) << "\n";
    }
}

void PrintDrPose(const patac_hpp::DRPose& pose) {
    std::cout << "DR Pose - Timestamp: " << pose.timestamp() 
              << ", X: " << pose.x()
              << ", Y: " << pose.y()
              << ", Yaw: " << pose.yaw() << "\n";
}

void PrintParkingSlots(const patac_hpp::ParkingSlotList& slots) {
    std::cout << "Parking Slots (" << slots.parking_slot_list_size() << " slots):\n";
    for (int i = 0; i < slots.parking_slot_list_size(); ++i) {
        const auto& slot = slots.parking_slot_list(i);
        std::cout << "Slot_index " << i << ":\n";
        std::cout << "Slotid: " << slot.id() << ":\n";
        for (int j = 0; j < slot.points_size(); ++j) {
            const auto& pt = slot.points(j);
            std::cout << "    Point " << j << ": (" << pt.x() << ", " << pt.y() << ")\n";
        }
    }
}

int main(int argc, char** argv) {
    // 检查参数
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " ./your.db" << std::endl;
        return 1;
    }
    // 从命令行参数获取数据库文件路径
    std::string db_path = argv[1];
    // 初始化读取器
    apa_slam::DataReader reader(db_path); //读取数据库文件名
    
    // 读取数据序列
    auto data_seq = reader.ReadDataSeq();
    PrintDataSeq(data_seq);
    
    // 读取DR姿态数据
    auto dr_poses = reader.ReadDrPose();
    for (const auto& pose : dr_poses) {
        PrintDrPose(pose);
    }
    
    // 读取车位数据
    auto parking_slots = reader.ReadParkingSlots();
    for (const auto& slots : parking_slots) {
        PrintParkingSlots(slots);
    }
    
    // 读取图片
    auto img_list = reader.ReadImgs();
    std::cout << "Loaded ImageListData with num: " << img_list.size()<< std::endl;
    for (const auto& img : img_list)
    {
        // 处理每个Image
        std::cout << "timestamp: " << img.timestamp()<<" num_image: "<<img.num_image()<< std::endl;
        for (int i = 0; i < img.image_list_size(); ++i) {

            const patac_hpp::Image& img_proto = img.image_list(i);
            std::cout <<"img index: "<<i<< std::endl;
            std::cout <<"wxh: "<<img_proto.width()<<","<<img_proto.height()<< std::endl;
            // 读取BGR原始数据
            // cv::Mat imgdecode(
            //     img_proto.height(),
            //     img_proto.width(),
            //     CV_8UC3,
            //     const_cast<char*>(img_proto.data().data()) 
            // );
            // 读取PNG编码数据
            std::string img_data = img_proto.data(); 
            std::vector<uchar> buffer(img_data.begin(), img_data.end());
            cv::Mat imgdecode = cv::imdecode(buffer, cv::IMREAD_COLOR);

            cv::Mat display_img = imgdecode.clone();
            cv::imshow("BGR Image", display_img);
            cv::waitKey(0); 

        }
    }
    
    return 0;
}