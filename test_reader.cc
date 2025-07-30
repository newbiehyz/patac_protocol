#include <Eigen/Eigen>
#include "data_reader.h"
#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>
#include <cmath>

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
Eigen::Matrix4d get_pose_matrix(double x, double y, double yaw_deg) {
    // 将角度转换为弧度并调整方向 (90 - yaw)
    double theta = (90.0 - yaw_deg) * M_PI / 180.0;
    
    // 创建旋转矩阵
    Eigen::Matrix3d R;
    R << std::cos(theta), -std::sin(theta), 0,
         std::sin(theta),  std::cos(theta), 0,
         0,               0,               1;
    
    // 转换单位 (mm → m) 并创建平移向量
    x *= 0.001;
    y *= 0.001;
    Eigen::Vector3d t(x, y, 0.0);
    
    // 创建 4x4 齐次变换矩阵
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = R;    // 左上角 3x3 部分设为旋转矩阵
    T.block<3,1>(0,3) = t;    // 前三行的第四列设为平移向量
    
    return T;
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
    // auto data_seq = reader.ReadDataSeq();
    // PrintDataSeq(data_seq);
    // while (1)
    // {
    // static int cnt = 0; 
    
    // 读取DR姿态数据
    int dr_rc;
    auto dr_poses = reader.ReadDrPose(dr_rc);
    std::cout <<"pose size: " <<dr_poses.size()<< std::endl;
    std::cout <<"pose start: " <<dr_poses[0].x()*1000<<","<<dr_poses[0].y()*1000<<","<<dr_poses[0].yaw()/3.1415926*180<< std::endl;
    std::cout <<"pose end: " <<dr_poses[dr_poses.size()-1].x()*1000<<","<<dr_poses[dr_poses.size()-1].y()*1000<<","<<dr_poses[dr_poses.size()-1].yaw()/3.1415926*180<< std::endl;

    for (const auto& pose : dr_poses) {
            std::ofstream ofs(std::string("../drpose.csv"), std::ios::app);//校验时间戳对齐
                ofs << pose.timestamp()
                    << ","
                    << pose.x()
                    << ","
                    << pose.y()
                    << ","
                    << pose.yaw()
                    << std::endl;
    }
    // for (const auto& pose : dr_poses) {
    //     // PrintDrPose(pose);
    //     break;
    //     Eigen::Matrix4d pose_matrix;
    //     pose_matrix = get_pose_matrix(pose.x(),pose.y(),pose.yaw());
    //     std::ostringstream drfilename;
    //     // std::string drfilename = "output_" + std::to_string(cnt) + ".txt";
    //         drfilename << "frame-" 
    //          << std::setw(6) << std::setfill('0') << cnt 
    //          << ".pose.txt";
    //     std::ofstream outFile(drfilename.str());
    //     // 检查文件是否成功打开
    //     if (!outFile) {
    //         std::cerr << "无法打开文件！" << std::endl;
    //         return 1;
    //     }
    //     // 设置输出格式：固定小数，7位精度
    //     outFile << std::fixed << std::setprecision(7);
    //     // 以矩阵形式写入数据
    //     for (int i = 0; i < 4; ++i) {
    //         for (int j = 0; j < 4; ++j) {
    //             outFile <<pose_matrix(i,j)<<" "; 
    //         }
    //         outFile << std::endl;
    //     }
    //     // 写入数据
    //     // outFile <<pose.x()<<" "<<pose.y()<<" "<<pose.yaw()<< std::endl;
    //     // 关闭文件
    //     outFile.close();
    // }

    // 读取车位数据
    // auto parking_slots = reader.ReadParkingSlots();
    // for (const auto& slots : parking_slots) {
        // PrintParkingSlots(slots);
    // }

    while (1)
    {
    static int cnt = 0; 
    // 读取图片
    std::cout << "img read start: " << std::endl;
    int img_rc;
    auto img_list = reader.ReadImgs(img_rc);
    if (img_rc != SQLITE_ROW)
    {
        std::cout << "img read finished!" << std::endl;
        break;
    }
    std::cout << "Loaded ImageListData with num: " << img_list.size()<< std::endl;
    for (const auto& img : img_list)
    {
        // 处理每个Image
        std::cout << "timestamp: " << img.timestamp()<<" num_image: "<<img.num_image()<< std::endl;

        auto closest_pose = dr_poses[0];
        auto min_diff = std::max(closest_pose.timestamp(), img.timestamp()) - std::min(closest_pose.timestamp(), img.timestamp());
        
        for (auto pose : dr_poses) {
            auto diff = std::max(pose.timestamp(), img.timestamp()) - std::min(pose.timestamp(), img.timestamp());
            if (diff < min_diff) {
                min_diff = diff;
                closest_pose = pose;
            }
        }

        std::cout << "time min_diff: " <<min_diff<<" cnt: "<<cnt<<" pose.time: "<<closest_pose.timestamp()<<"img.times: "<<img.timestamp()<< std::endl;
        std::ofstream ofs(std::string("../time_check.csv"), std::ios::app);//校验时间戳对齐
                    ofs << min_diff
                    << ","
                    << cnt
                    << ","
                    << img.timestamp()
                    << ","
                    << closest_pose.timestamp()
                    << ","
                    << closest_pose.x()*1000
                    << ","
                    << closest_pose.y()*1000
                    << ","
                    << closest_pose.yaw()/3.1415926*180
                    << std::endl;
                    
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

            // cv::Mat display_img = imgdecode.clone();
            // cv::imshow("BGR Image", imgdecode);
            if(i == 1)
            {   
                std::ostringstream filename;
                    filename << "../data_set/frame-" 
                    << std::setw(6) << std::setfill('0') << cnt 
                    << ".color.jpg";
                cv::imwrite(filename.str(), imgdecode);  // JPEG格式（有损压缩，文件小）
            }
            // cv::waitKey(0); 
        }

        Eigen::Matrix4d pose_matrix;
        pose_matrix = get_pose_matrix(closest_pose.x()*1000,closest_pose.y()*1000,closest_pose.yaw()/3.1415926*180);
        std::ostringstream drfilename;
        // std::string drfilename = "output_" + std::to_string(cnt) + ".txt";
            drfilename << "../data_set/frame-" 
             << std::setw(6) << std::setfill('0') << cnt 
             << ".pose.txt";
        std::ofstream outFile(drfilename.str());
        // 检查文件是否成功打开
        if (!outFile) {
            std::cerr << "无法打开文件！" << std::endl;
            return 1;
        }
        // 设置输出格式：固定小数，7位精度
        outFile << std::fixed << std::setprecision(7);
        // 以矩阵形式写入数据
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                outFile <<pose_matrix(i,j)<<" "; 
            }
            outFile << std::endl;
        }
        // 写入数据
        // outFile <<pose.x()<<" "<<pose.y()<<" "<<pose.yaw()<< std::endl;
        // 关闭文件
        outFile.close();

    }
    
    cnt++;
    }//while
    return 0;
}