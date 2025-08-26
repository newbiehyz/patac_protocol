"""
usage: python3 bin_visualization.py
放在 bin文件同目录下运行
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import os
from datetime import datetime

import patac_protocol.python_gen.patac_slot_pb2 as patac_slot_pb2
import patac_protocol.python_gen.patac_dr_pb2 as patac_dr_pb2
import patac_protocol.python_gen.patac_image_pb2 as patac_image_pb2
import patac_protocol.python_gen.patac_trajetory_pb2 as patac_trajetory_pb2


class ProtobufVisualizer:
    def __init__(self):
        self.point_src = []
        self.data_cache = {}
        
    def load_bin_file(self, file_path, data_type='slot'):
        if not os.path.exists(file_path):
            print(f"File {file_path} does not exist")
            return None
            
        try:
            if data_type == 'slot':
                return self._parse_slot_file(file_path)
                    
            elif data_type == 'trajectory':
                return self._parse_trajectory_file(file_path)
                    
        except Exception as e:
            print(f"Parse failed: {e}")
            return None

    def _parse_slot_file(self, file_path):    
        slot_lists = []
        total_slots = 0
        
        with open(file_path, 'rb') as f:
            while True:
                # 读取4字节长度前缀
                length_bytes = f.read(4)
                if len(length_bytes) < 4:
                    break  # 文件结束
                
                data_length = int.from_bytes(length_bytes, byteorder='little')
                print(f"Read data block length: {data_length} bytes")
                
                # 读取序列化的ParkingSlotList数据
                slot_data = f.read(data_length)
                if len(slot_data) < data_length:
                    print("Warning: incomplete data block")
                    break
                
                # 解析ParkingSlotList
                parking_list = patac_slot_pb2.ParkingSlotList()
                parking_list.ParseFromString(slot_data)
                slot_lists.append(parking_list)
                total_slots += parking_list.num_parking_slot
                
                print(f"Parsed ParkingSlotList: {parking_list.num_parking_slot} slots")
        
        print(f"Total parsed {len(slot_lists)} ParkingSlotLists, containing {total_slots} slots")
        
        # 合并所有车位到一个列表中
        merged_list = self._merge_parking_slot_lists(slot_lists)
        if merged_list:
            self._extract_parking_points(merged_list)
        
        return merged_list
    
    def _parse_trajectory_file(self, file_path):
        with open(file_path, 'rb') as f:
            # 读取4字节长度前缀
            length_bytes = f.read(4)
            if len(length_bytes) < 4:
                return None
            
            data_length = int.from_bytes(length_bytes, byteorder='little')
            print(f"Trajectory data length: {data_length} bytes")
            
            trajectory_data = f.read(data_length)
            if len(trajectory_data) < data_length:
                return None
            
            trajectory = patac_trajetory_pb2.Trajectory()
            trajectory.ParseFromString(trajectory_data)
            self._extract_trajectory_points(trajectory)
            
            print(f"Parsed trajectory: {trajectory.num_trajectory_point} points")
            return trajectory
    
    def _merge_parking_slot_lists(self, slot_lists):
        if not slot_lists:
            return None
        
        merged_list = patac_slot_pb2.ParkingSlotList()
        merged_list.timestamp = slot_lists[0].timestamp
        merged_list.num_parking_slot = 0
        
        for slot_list in slot_lists:
            for slot in slot_list.parking_slot_list:
                new_slot = merged_list.parking_slot_list.add()
                new_slot.CopyFrom(slot)
                merged_list.num_parking_slot += 1
        
        return merged_list
        
    def _extract_parking_points(self, parking_list):
        self.point_src = []
        
        if not parking_list or not parking_list.parking_slot_list:
            return

        for i in range(len(parking_list.parking_slot_list)):
            slot = parking_list.parking_slot_list[i]
            print(f"Slot {i}: ID={slot.id}, valid={slot.valid}, points={len(slot.points)}")
            
            if slot.valid:
                for j in range(len(slot.points)):
                    point = slot.points[j]
                    self.point_src.append([point.x, point.y, 0])
                    print(f"  Point{j}: ({point.x:.3f}, {point.y:.3f})")
                
    def _extract_trajectory_points(self, trajectory):
        self.point_src = []
        
        for i in range(len(trajectory.trajectory_point_list)):
            point = trajectory.trajectory_point_list[i]
            self.point_src.append([point.x, point.y, point.z])

    def _construct_parking_rectangle(self, slot_points, slot_type):
        """根据车位类型构造矩形"""
        if len(slot_points) != 2:
            return slot_points
            
        p1, p2 = slot_points
        p1, p2 = np.array(p1), np.array(p2)
        
        # 计算两点间的距离
        distance = np.linalg.norm(p2 - p1)
        if distance < 1e-6:  # 避免除零错误
            return slot_points
            
        # 计算从p1到p2的单位方向向量
        direction_p1_to_p2 = (p2 - p1) / distance
        
        # 顺时针旋转90度得到"向后"方向
        # 如果原向量是(x, y)，顺时针旋转90度后是(y, -x)
        backward_direction = np.array([direction_p1_to_p2[1], -direction_p1_to_p2[0]])
        
        if slot_type == patac_slot_pb2.SlotTypeVertical:
            # 垂直车位：两点连线作为宽边，向后延伸5.45m作为长边
            depth = 5.45
            
            # 构造矩形的四个顶点
            # p1和p2是车位开口的两端
            # 向后延伸depth距离
            corner1 = p1  # 开口端点1
            corner2 = p2  # 开口端点2
            corner3 = p2 + backward_direction * depth  # 后端点2
            corner4 = p1 + backward_direction * depth  # 后端点1
            
        elif slot_type == patac_slot_pb2.SlotTypeParallel:
            # 水平车位：两点连线作为长边，向后延伸2.25m作为宽边
            width = 2.25
            
            # 构造矩形的四个顶点
            # p1和p2是车位长边的两端
            # 向后延伸width距离
            corner1 = p1  # 前端点1
            corner2 = p2  # 前端点2
            corner3 = p2 + backward_direction * width  # 后端点2
            corner4 = p1 + backward_direction * width  # 后端点1
            
        else:
            # 其他类型车位，使用默认逻辑或原始点
            return slot_points
        
        # 返回矩形的四个顶点
        rectangle_points = [
            [corner1[0], corner1[1]],
            [corner2[0], corner2[1]],
            [corner3[0], corner3[1]],
            [corner4[0], corner4[1]]
        ]
        
        return rectangle_points

    def visualize_combined(self, parking_list=None, trajectory=None):
        plt.figure(figsize=(16, 12))
        
        all_x, all_y = [], []
        
        # 绘制车位
        if parking_list and parking_list.parking_slot_list:
            
            type_colors = {
                patac_slot_pb2.SlotTypeUnknown: 'gray',
                patac_slot_pb2.SlotTypeVertical: 'blue', 
                patac_slot_pb2.SlotTypeParallel: 'green',
                patac_slot_pb2.SlotTypeOblique: 'orange'
            }
            
            occupancy_colors = {
                patac_slot_pb2.OccupancyStatusUnknown: 'lightgray',
                patac_slot_pb2.OccupancyStatusOccupied: 'red',
                patac_slot_pb2.OccupancyStatusNotOccupied: 'lightblue'
            }
            
            valid_slots_drawn = 0
                    
            for i in range(len(parking_list.parking_slot_list)):
                slot = parking_list.parking_slot_list[i]
                
                if not slot.valid:
                    continue
                    
                slot_points = []
                for j in range(len(slot.points)):
                    point = slot.points[j]
                    slot_points.append([point.x, point.y])
                    all_x.append(point.x)
                    all_y.append(point.y)
                
                # 使用新的矩形构造方法
                if len(slot_points) == 2:
                    slot_points = self._construct_parking_rectangle(slot_points, slot.type)
                                
                try:
                    polygon = patches.Polygon(slot_points,
                                             closed=True,
                                             facecolor=occupancy_colors.get(slot.occupancy, 'gray'),
                                             edgecolor=type_colors.get(slot.type, 'black'),
                                             linewidth=2,
                                             alpha=0.6)
                    plt.gca().add_patch(polygon)
                    valid_slots_drawn += 1
                    
                    # 车位ID标记
                    if slot_points:
                        center_x = np.mean([p[0] for p in slot_points])
                        center_y = np.mean([p[1] for p in slot_points])
                        
                        plt.text(center_x, center_y, f'{slot.id}',
                                ha='center', va='center', fontsize=8, fontweight='bold',
                                bbox=dict(boxstyle="round,pad=0.1", facecolor='white', alpha=0.8))
                                           
                except Exception as e:
                    print(f"  Failed to draw slot {slot.id}: {e}")
                    continue
                    
        # 绘制轨迹
        if trajectory and trajectory.trajectory_point_list:
            
            x_coords = []
            y_coords = []
            
            for point in trajectory.trajectory_point_list:
                x_coords.append(point.x)
                y_coords.append(point.y)
                all_x.append(point.x)
                all_y.append(point.y)
            
            x_coords = np.array(x_coords)
            y_coords = np.array(y_coords)
            
            # 画轨迹
            # plt.plot(x_coords, y_coords, 'red', linewidth=3, alpha=0.8, label='Trajectory Path', zorder=10)

            scatter = plt.scatter(x_coords, y_coords, c=range(len(x_coords)), cmap='plasma', 
                       s=30, alpha=0.9, label='Trajectory Points', zorder=11, edgecolors='white', linewidth=0.5)
            
            # 标记起点和终点
            if len(x_coords) > 0:
                plt.scatter(x_coords[0], y_coords[0], color='lime', s=200, 
                           marker='o', label='Start Point', zorder=12, edgecolors='black', linewidth=2)
                plt.scatter(x_coords[-1], y_coords[-1], color='red', s=200, 
                           marker='s', label='End Point', zorder=12, edgecolors='black', linewidth=2)
            
            print(f"Successfully drew trajectory with {len(x_coords)} points")
        
        # 设置图形属性
        if all_x and all_y:
            margin = max(5, (max(all_x) - min(all_x)) * 0.1)
            plt.xlim(min(all_x) - margin, max(all_x) + margin)
            plt.ylim(min(all_y) - margin, max(all_y) + margin)
        else:
            plt.xlim(-10, 10)
            plt.ylim(-10, 10)
        
        plt.xlabel('X Coordinate (m)')
        plt.ylabel('Y Coordinate (m)')
        
        title_parts = []
        if parking_list:
            title_parts.append(f"Parking Slots: {parking_list.num_parking_slot}")
        if trajectory:
            title_parts.append(f"Trajectory Points: {trajectory.num_trajectory_point}")
        
        plt.title(' | '.join(title_parts) if title_parts else 'Map Visualization')
        plt.grid(True, alpha=0.3)
        plt.axis('equal')
        
        legend_elements = []
        
        if trajectory and trajectory.trajectory_point_list:
            legend_elements.extend([
                plt.Line2D([0], [0], marker='o', color='lime', lw=0, markersize=8, label='Start Point'),
                plt.Line2D([0], [0], marker='s', color='red', lw=0, markersize=8, label='End Point')
            ])
        
        if parking_list and parking_list.parking_slot_list:
            type_colors = {
                patac_slot_pb2.SlotTypeUnknown: 'gray',
                patac_slot_pb2.SlotTypeVertical: 'blue', 
                patac_slot_pb2.SlotTypeParallel: 'green',
                patac_slot_pb2.SlotTypeOblique: 'orange'
            }
            
            occupancy_colors = {
                patac_slot_pb2.OccupancyStatusUnknown: 'lightgray',
                patac_slot_pb2.OccupancyStatusOccupied: 'red',
                patac_slot_pb2.OccupancyStatusNotOccupied: 'lightblue'
            }
            
            type_names = {
                patac_slot_pb2.SlotTypeUnknown: 'Unknown Slot',
                patac_slot_pb2.SlotTypeVertical: 'Vertical Slot',
                patac_slot_pb2.SlotTypeParallel: 'Parallel Slot', 
                patac_slot_pb2.SlotTypeOblique: 'Oblique Slot'
            }
            
            occupancy_names = {
                patac_slot_pb2.OccupancyStatusUnknown: 'Unknown Status',
                patac_slot_pb2.OccupancyStatusOccupied: 'Occupied',
                patac_slot_pb2.OccupancyStatusNotOccupied: 'Free'
            }
            
            for slot_type, color in type_colors.items():
                legend_elements.append(
                    plt.Line2D([0], [0], color=color, lw=3, label=type_names.get(slot_type, "Unknown"))
                )
            
            for occupancy, color in occupancy_colors.items():
                legend_elements.append(
                    patches.Patch(color=color, label=occupancy_names.get(occupancy, "Unknown"))
                )
        
        if legend_elements:
            plt.legend(handles=legend_elements, loc='best', bbox_to_anchor=(1.05, 1), borderaxespad=0)
        
        plt.tight_layout()
        plt.show()
            
    def visualize_trajectory(self, trajectory):
        if not trajectory or not trajectory.trajectory_point_list:
            return
            
        plt.figure(figsize=(16, 12))
        
        # 提取轨迹点坐标
        x_coords = []
        y_coords = []
        z_coords = []
        timestamps = []
        
        for point in trajectory.trajectory_point_list:
            x_coords.append(point.x)
            y_coords.append(point.y)
            z_coords.append(point.z)
            timestamps.append(point.timestamp)
        
        x_coords = np.array(x_coords)
        y_coords = np.array(y_coords)
        z_coords = np.array(z_coords)
        timestamps = np.array(timestamps)
        
        # 画轨迹
        # plt.plot(x_coords, y_coords, 'b-', linewidth=3, alpha=0.8, label='Trajectory Path')

        scatter = plt.scatter(x_coords, y_coords, c=range(len(x_coords)), cmap='viridis', s=30, alpha=0.9, label='Trajectory Points', edgecolors='white', linewidth=0.5)
        
        # 标记起点和终点
        if len(x_coords) > 0:
            plt.scatter(x_coords[0], y_coords[0], color='lime', s=200, 
                       marker='o', label='Start Point', zorder=12, edgecolors='black', linewidth=2)
            plt.scatter(x_coords[-1], y_coords[-1], color='red', s=200, 
                       marker='s', label='End Point', zorder=12, edgecolors='black', linewidth=2)
        
        plt.xlabel('X Coordinate (m)')
        plt.ylabel('Y Coordinate (m)')
        plt.title(f'Trajectory Visualization ({len(x_coords)} points)')
        plt.grid(True, alpha=0.3)
        plt.legend()
        plt.axis('equal')
        
        plt.tight_layout()
        plt.show()
                
        if len(x_coords) > 1:
            # 计算轨迹总长度
            distances = np.sqrt(np.diff(x_coords)**2 + np.diff(y_coords)**2 + np.diff(z_coords)**2)
            total_distance = np.sum(distances)
            print(f"Total Distance: {total_distance:.3f} m")
            print(f"Average Point Distance: {total_distance/(len(x_coords)-1):.3f} m")

    def visualize_parking_slots(self, parking_list):
        if not parking_list or not parking_list.parking_slot_list:
            return
            
        plt.figure(figsize=(15, 10))
        
        type_colors = {
            patac_slot_pb2.SlotTypeUnknown: 'gray',
            patac_slot_pb2.SlotTypeVertical: 'blue', 
            patac_slot_pb2.SlotTypeParallel: 'green',
            patac_slot_pb2.SlotTypeOblique: 'orange'
        }
        
        occupancy_colors = {
            patac_slot_pb2.OccupancyStatusUnknown: 'lightgray',
            patac_slot_pb2.OccupancyStatusOccupied: 'red',
            patac_slot_pb2.OccupancyStatusNotOccupied: 'lightblue'
        }
        
        type_names = {
            patac_slot_pb2.SlotTypeUnknown: 'Unknown',
            patac_slot_pb2.SlotTypeVertical: 'Vertical',
            patac_slot_pb2.SlotTypeParallel: 'Parallel', 
            patac_slot_pb2.SlotTypeOblique: 'Oblique'
        }
        
        occupancy_names = {
            patac_slot_pb2.OccupancyStatusUnknown: 'Unknown',
            patac_slot_pb2.OccupancyStatusOccupied: 'Occupied',
            patac_slot_pb2.OccupancyStatusNotOccupied: 'Free'
        }
        
        all_points = []
        valid_slots_drawn = 0
                
        for i in range(len(parking_list.parking_slot_list)):
            slot = parking_list.parking_slot_list[i]
            
            print(f"Slot {i}: ID={slot.id}, valid={slot.valid}, points_count={len(slot.points)}")
            
            if not slot.valid:
                continue
                
            slot_points = []
            for j in range(len(slot.points)):
                point = slot.points[j]
                slot_points.append([point.x, point.y])
                all_points.append([point.x, point.y])
            
            # 使用新的矩形构造方法
            if len(slot_points) == 2:
                slot_points = self._construct_parking_rectangle(slot_points, slot.type)
                            
            try:
                polygon = patches.Polygon(slot_points,
                                         closed=True,
                                         facecolor=occupancy_colors.get(slot.occupancy, 'gray'),
                                         edgecolor=type_colors.get(slot.type, 'black'),
                                         linewidth=2,
                                         alpha=0.7)
                plt.gca().add_patch(polygon)
                valid_slots_drawn += 1
                
                # 画到中心
                if slot_points:
                    center_x = np.mean([p[0] for p in slot_points])
                    center_y = np.mean([p[1] for p in slot_points])
                    
                    # 添加车位ID
                    plt.text(center_x, center_y, f'{slot.id}',
                            ha='center', va='center', fontsize=8, fontweight='bold',
                            bbox=dict(boxstyle="round,pad=0.1", facecolor='white', alpha=0.8))
                                       
            except Exception as e:
                print(f"  Failed to draw slot {slot.id}: {e}")
                continue
                
        # 设置图形属性
        if all_points:
            all_points = np.array(all_points)
            margin = 5
            x_min, x_max = all_points[:, 0].min(), all_points[:, 0].max()
            y_min, y_max = all_points[:, 1].min(), all_points[:, 1].max()
            
            print(f"Coordinate Range: X[{x_min:.2f}, {x_max:.2f}], Y[{y_min:.2f}, {y_max:.2f}]")
            
            plt.xlim(x_min - margin, x_max + margin)
            plt.ylim(y_min - margin, y_max + margin)
        else:
            plt.xlim(-10, 10)
            plt.ylim(-10, 10)
        
        plt.xlabel('X Coordinate (m)')
        plt.ylabel('Y Coordinate (m)')
        plt.title(f'Total {parking_list.num_parking_slot} slots, Drawn {valid_slots_drawn} slots')
        plt.grid(True, alpha=0.3)
        plt.axis('equal')
        
        type_legend_elements = []
        for slot_type, color in type_colors.items():
            type_legend_elements.append(
                plt.Line2D([0], [0], color=color, lw=3,
                          label=f'{type_names.get(slot_type, "Unknown")} Slot')
            )
        
        occupancy_legend_elements = []
        for occupancy, color in occupancy_colors.items():
            occupancy_legend_elements.append(
                patches.Patch(color=color,
                             label=f'{occupancy_names.get(occupancy, "Unknown")} Status')
            )
        
        type_legend = plt.legend(handles=type_legend_elements,
                               loc='upper left', title='Slot Type')
        plt.gca().add_artist(type_legend)
        
        occupancy_legend = plt.legend(handles=occupancy_legend_elements,
                                    loc='upper right', title='Occupancy Status')
        
        plt.tight_layout()
        plt.show()
    
    def print_data_summary(self, data, data_type):
        print(f"\n{'='*50}")
        print(f"Data Type: {data_type.upper()}")
        print(f"{'='*50}")
        
        if data_type == 'slot':
            print(f"Timestamp: {data.timestamp}")
            print(f"Total Slots: {data.num_parking_slot}")
            print(f"Valid Slots: {sum(1 for slot in data.parking_slot_list if slot.valid)}")
                        
        elif data_type == 'trajectory':
            print(f"Trajectory Points: {data.num_trajectory_point}")
            if data.trajectory_point_list:
                first_point = data.trajectory_point_list[0]
                last_point = data.trajectory_point_list[-1]
                print(f"Start Point: ({first_point.x:.3f}, {first_point.y:.3f}, {first_point.z:.3f})")
                print(f"End Point: ({last_point.x:.3f}, {last_point.y:.3f}, {last_point.z:.3f})")
                print(f"Start Timestamp: {first_point.timestamp}")
                print(f"End Timestamp: {last_point.timestamp}")
                            
        print(f"Extracted Points Count: {len(self.point_src)}")

def main():
    visualizer = ProtobufVisualizer()
    
    data_files = {
        'slot': './sematic_map.bin',
        'trajectory': './trajectory.bin',
    }
    
    trajectory_data = None
    parking_data = None
    
    if os.path.exists(data_files['trajectory']):
        trajectory_data = visualizer.load_bin_file(data_files['trajectory'], 'trajectory')
        # if trajectory_data:
            # visualizer.print_data_summary(trajectory_data, 'trajectory')
    
    if os.path.exists(data_files['slot']):
        parking_data = visualizer.load_bin_file(data_files['slot'], 'slot')
        # if parking_data:
            # visualizer.print_data_summary(parking_data, 'slot')
    
    if trajectory_data and parking_data:
        visualizer.visualize_combined(parking_data, trajectory_data)
    elif trajectory_data:
        visualizer.visualize_trajectory(trajectory_data)
    elif parking_data:
        visualizer.visualize_parking_slots(parking_data)
    else:
        print("No valid data files found. Please check file paths and formats.")

if __name__ == "__main__":
    main()