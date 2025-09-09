"""
usage: python3 bin_visualization.py
放在bin文件同目录下运行
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import os
import csv
import math
from datetime import datetime

import patac_protocol.python_gen.patac_slot_pb2 as patac_slot_pb2
import patac_protocol.python_gen.patac_dr_pb2 as patac_dr_pb2
import patac_protocol.python_gen.patac_image_pb2 as patac_image_pb2
import patac_protocol.python_gen.patac_trajetory_pb2 as patac_trajetory_pb2


class ProtobufVisualizer:
    def __init__(self):
        self.point_src = []
        self.data_cache = {}
        
    def quaternion_to_euler(self, qx, qy, qz, qw):
        """将四元数转换为欧拉角(roll, pitch, yaw)，返回弧度制"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw

    def catmull_rom(self, p0, p1, p2, p3, t):
        """Compute position at t in [0,1] using Catmull-Rom spline."""
        t2 = t * t
        t3 = t2 * t
        return 0.5 * (
            (2 * p1) +
            (-p0 + p2) * t +
            (2*p0 - 5*p1 + 4*p2 - p3) * t2 +
            (-p0 + 3*p1 - 3*p2 + p3) * t3
        )

    def catmull_rom_derivative(self, p0, p1, p2, p3, t):
        """Compute velocity at t in [0,1] using derivative of Catmull-Rom."""
        t2 = t * t
        return 0.5 * (
            (-p0 + p2) +
            2*(2*p0 - 5*p1 + 4*p2 - p3)*t +
            3*(-p0 + 3*p1 - 3*p2 + p3)*t2
        )

    def generate_catmull_rom_trajectory(self, ctrl_pts, samples_per_segment=20):
        """Generate interpolated trajectory using Catmull-Rom spline"""
        positions = []
        velocities = []
        headings = []
        ts_all = []
        
        # Loop through each segment with 4 control points
        for i in range(1, len(ctrl_pts) - 2):
            p0, p1, p2, p3 = ctrl_pts[i-1], ctrl_pts[i], ctrl_pts[i+1], ctrl_pts[i+2]
            ts = np.linspace(0, 1, samples_per_segment)
            for t in ts:
                pos = self.catmull_rom(p0, p1, p2, p3, t)
                vel = self.catmull_rom_derivative(p0, p1, p2, p3, t)
                theta = np.arctan2(vel[1], vel[0])
                positions.append(pos)
                velocities.append(vel)
                headings.append(theta)
                ts_all.append(t + i)  # pseudo time
                
        positions = np.array(positions)
        velocities = np.array(velocities)
        headings = np.unwrap(np.array(headings))
        ts_all = np.array(ts_all)
        
        # Use uniform dt for smoother gradient
        dt = 1.0 / samples_per_segment
        angular_velocities = np.gradient(headings, dt)
        
        return positions, velocities, angular_velocities, headings, ts_all

    def interpolate_trajectory_3d(self, trajectory_points, samples_per_segment=20):
        """Apply Catmull-Rom interpolation to 3D trajectory points"""
        if len(trajectory_points) < 4:
            print("Warning: Need at least 4 points for Catmull-Rom interpolation")
            return trajectory_points
        
        # Extract XYZ coordinates
        ctrl_pts_xyz = []
        for point in trajectory_points:
            ctrl_pts_xyz.append(np.array([point.x, point.y, point.z]))
        
        # Apply interpolation to XY coordinates (2D)
        ctrl_pts_xy = [np.array([point.x, point.y]) for point in trajectory_points]
        positions_xy, velocities_xy, angular_velocities, headings, ts_all = self.generate_catmull_rom_trajectory(
            ctrl_pts_xy, samples_per_segment
        )
        
        # Interpolate Z coordinates separately using linear interpolation between control points
        z_coords = []
        timestamps = []
        
        for i in range(1, len(ctrl_pts_xyz) - 2):
            z0, z1, z2, z3 = ctrl_pts_xyz[i-1][2], ctrl_pts_xyz[i][2], ctrl_pts_xyz[i+1][2], ctrl_pts_xyz[i+2][2]
            t0, t1, t2, t3 = (trajectory_points[i-1].timestamp, trajectory_points[i].timestamp, 
                              trajectory_points[i+1].timestamp, trajectory_points[i+2].timestamp)
            
            ts = np.linspace(0, 1, samples_per_segment)
            for t in ts:
                # Linear interpolation for Z between p1 and p2
                z_interp = z1 + (z2 - z1) * t
                z_coords.append(z_interp)
                
                # Linear interpolation for timestamp between p1 and p2
                timestamp_interp = t1 + (t2 - t1) * t
                timestamps.append(int(timestamp_interp))
        
        # Create interpolated trajectory data
        interpolated_data = []
        for i in range(len(positions_xy)):
            data_point = {
                'timestamp': timestamps[i] if i < len(timestamps) else timestamps[-1],
                'x': positions_xy[i][0],
                'y': positions_xy[i][1], 
                'z': z_coords[i] if i < len(z_coords) else z_coords[-1],
                'heading': headings[i],
                'velocity': np.linalg.norm(velocities_xy[i]) if i < len(velocities_xy) else 0,
                'angular_velocity': angular_velocities[i] if i < len(angular_velocities) else 0
            }
            interpolated_data.append(data_point)
        
        return interpolated_data
        
    def save_trajectory_to_csv(self, trajectory, filename, use_interpolation=True, samples_per_segment=20):
        """将轨迹数据保存为CSV文件，可选择是否使用插值"""
        if not trajectory or not trajectory.trajectory_point_list:
            print(f"No trajectory data to save to {filename}")
            return False
            
        try:
            with open(filename, 'w', newline='', encoding='utf-8') as csvfile:
                writer = csv.writer(csvfile)
                
                # 写入表头
                writer.writerow(['timestamp', 'x', 'y', 'z', 'roll', 'pitch', 'yaw', 'velocity', 'angular_velocity'])
                
                if use_interpolation and len(trajectory.trajectory_point_list) >= 4:
                    print(f"Applying Catmull-Rom interpolation with {samples_per_segment} samples per segment...")
                    
                    # 使用插值
                    interpolated_data = self.interpolate_trajectory_3d(
                        trajectory.trajectory_point_list, samples_per_segment
                    )
                    
                    # 写入插值后的数据
                    for data_point in interpolated_data:
                        # 从heading角度计算roll, pitch, yaw (简化处理，假设roll=pitch=0)
                        roll = 0
                        pitch = 0
                        yaw = data_point['heading']
                        
                        writer.writerow([
                            data_point['timestamp'],
                            data_point['x'],
                            data_point['y'], 
                            data_point['z'],
                            roll,
                            pitch,
                            yaw,
                            data_point['velocity'],
                            data_point['angular_velocity']
                        ])
                    
                    print(f"Successfully saved interpolated trajectory data to {filename}")
                    print(f"Original points: {len(trajectory.trajectory_point_list)}")
                    print(f"Interpolated points: {len(interpolated_data)}")
                    
                else:
                    print("Using original trajectory points without interpolation...")
                    
                    # 写入原始数据
                    for point in trajectory.trajectory_point_list:
                        # 将四元数转换为欧拉角
                        roll, pitch, yaw = self.quaternion_to_euler(point.qx, point.qy, point.qz, point.qw)
                        
                        writer.writerow([
                            point.timestamp,
                            point.x,
                            point.y, 
                            point.z,
                            roll,
                            pitch,
                            yaw,
                            point.velocity,
                            point.angular_velocity
                        ])
                    
                    print(f"Successfully saved original trajectory data to {filename}")
                    print(f"Total points saved: {len(trajectory.trajectory_point_list)}")
                    
                return True
                
        except Exception as e:
            print(f"Failed to save CSV file {filename}: {e}")
            return False
        
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

    def visualize_combined_with_dual_trajectory(self, parking_list=None, trajectory_loc=None, trajectory_map=None, save_image=False, image_filename=None):
        """可视化车位和两条轨迹数据"""
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
                    
        # 绘制LOC轨迹
        if trajectory_loc and trajectory_loc.trajectory_point_list:
            
            x_coords = []
            y_coords = []
            
            for point in trajectory_loc.trajectory_point_list:
                x_coords.append(point.x)
                y_coords.append(point.y)
                all_x.append(point.x)
                all_y.append(point.y)
            
            x_coords = np.array(x_coords)
            y_coords = np.array(y_coords)
            
            # 画LOC轨迹
            scatter_loc = plt.scatter(x_coords, y_coords, c=range(len(x_coords)), cmap='viridis', 
                       s=30, alpha=0.9, label='LOC Trajectory', zorder=11, edgecolors='white', linewidth=0.5)
            
            # 标记起点和终点
            if len(x_coords) > 0:
                plt.scatter(x_coords[0], y_coords[0], color='lime', s=200, 
                           marker='o', label='LOC Start', zorder=12, edgecolors='black', linewidth=2)
                plt.scatter(x_coords[-1], y_coords[-1], color='darkgreen', s=200, 
                           marker='s', label='LOC End', zorder=12, edgecolors='black', linewidth=2)
            
            print(f"Successfully drew LOC trajectory with {len(x_coords)} points")
        
        # 绘制MAP轨迹
        if trajectory_map and trajectory_map.trajectory_point_list:
            
            x_coords = []
            y_coords = []
            
            for point in trajectory_map.trajectory_point_list:
                x_coords.append(point.x)
                y_coords.append(point.y)
                all_x.append(point.x)
                all_y.append(point.y)
            
            x_coords = np.array(x_coords)
            y_coords = np.array(y_coords)
            
            # 画MAP轨迹
            scatter_map = plt.scatter(x_coords, y_coords, c=range(len(x_coords)), cmap='plasma', 
                       s=30, alpha=0.9, label='MAP Trajectory', zorder=11, edgecolors='white', linewidth=0.5)
            
            # 标记起点和终点
            if len(x_coords) > 0:
                plt.scatter(x_coords[0], y_coords[0], color='orange', s=200, 
                           marker='o', label='MAP Start', zorder=12, edgecolors='black', linewidth=2)
                plt.scatter(x_coords[-1], y_coords[-1], color='red', s=200, 
                           marker='s', label='MAP End', zorder=12, edgecolors='black', linewidth=2)
            
            print(f"Successfully drew MAP trajectory with {len(x_coords)} points")
        
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
        if trajectory_loc:
            title_parts.append(f"LOC Points: {trajectory_loc.num_trajectory_point}")
        if trajectory_map:
            title_parts.append(f"MAP Points: {trajectory_map.num_trajectory_point}")
        
        plt.title(' | '.join(title_parts) if title_parts else 'Map Visualization')
        plt.grid(True, alpha=0.3)
        plt.axis('equal')
        
        legend_elements = []
        
        # 添加轨迹图例
        if trajectory_loc and trajectory_loc.trajectory_point_list:
            legend_elements.extend([
                plt.Line2D([0], [0], marker='o', color='lime', lw=0, markersize=8, label='LOC Start'),
                plt.Line2D([0], [0], marker='s', color='darkgreen', lw=0, markersize=8, label='LOC End')
            ])
        
        if trajectory_map and trajectory_map.trajectory_point_list:
            legend_elements.extend([
                plt.Line2D([0], [0], marker='o', color='orange', lw=0, markersize=8, label='MAP Start'),
                plt.Line2D([0], [0], marker='s', color='red', lw=0, markersize=8, label='MAP End')
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
        
        # 保存图片
        if save_image and image_filename:
            try:
                plt.savefig(image_filename, dpi=300, bbox_inches='tight')
                print(f"Successfully saved visualization to {image_filename}")
            except Exception as e:
                print(f"Failed to save image {image_filename}: {e}")
        
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
                        
        elif data_type in ['trajectory', 'trajectory_loc', 'trajectory_map']:
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
        'trajectory_loc': './trajectory_loc.bin',
        'trajectory_map': './trajectory_map.bin',
    }
    
    trajectory_loc_data = None
    trajectory_map_data = None
    parking_data = None
    
    # 加载LOC轨迹数据
    if os.path.exists(data_files['trajectory_loc']):
        print(f"Loading LOC trajectory from {data_files['trajectory_loc']}...")
        trajectory_loc_data = visualizer.load_bin_file(data_files['trajectory_loc'], 'trajectory')
        if trajectory_loc_data:
            visualizer.print_data_summary(trajectory_loc_data, 'trajectory_loc')
    
    # 加载MAP轨迹数据
    if os.path.exists(data_files['trajectory_map']):
        print(f"Loading MAP trajectory from {data_files['trajectory_map']}...")
        trajectory_map_data = visualizer.load_bin_file(data_files['trajectory_map'], 'trajectory')
        if trajectory_map_data:
            visualizer.print_data_summary(trajectory_map_data, 'trajectory_map')
            
            # 保存MAP轨迹为CSV文件 (使用插值)
            csv_filename = './trajectory_map_interpolated.csv'
            csv_saved = visualizer.save_trajectory_to_csv(
                trajectory_map_data, 
                csv_filename, 
                use_interpolation=True,  # 启用插值
                samples_per_segment=20  # 每段100个采样点
            )
            
            if csv_saved:
                # 保存可视化图片
                image_filename = './trajectory_map_interpolated.png'
                
                # 单独可视化MAP轨迹并保存
                plt.figure(figsize=(16, 12))
                
                if trajectory_map_data.trajectory_point_list:
                    # 原始轨迹点
                    x_coords_orig = [point.x for point in trajectory_map_data.trajectory_point_list]
                    y_coords_orig = [point.y for point in trajectory_map_data.trajectory_point_list]
                    
                    # 如果有足够的点，生成插值轨迹用于显示
                    if len(x_coords_orig) >= 4:
                        ctrl_pts_xy = [np.array([point.x, point.y]) for point in trajectory_map_data.trajectory_point_list]
                        positions_xy, _, _, _, _ = visualizer.generate_catmull_rom_trajectory(ctrl_pts_xy, 100)
                        
                        # 绘制插值后的轨迹
                        plt.plot(positions_xy[:, 0], positions_xy[:, 1], 'b-', linewidth=2, alpha=0.7, label='Interpolated Trajectory')
                    
                    # 绘制原始轨迹点
                    scatter = plt.scatter(x_coords_orig, y_coords_orig, c='red', s=80, alpha=0.9, 
                               label='Original Control Points', zorder=11, edgecolors='black', linewidth=1)
                    
                    # 标记起点和终点
                    if len(x_coords_orig) > 0:
                        plt.scatter(x_coords_orig[0], y_coords_orig[0], color='lime', s=200, 
                                   marker='o', label='Start Point', zorder=12, edgecolors='black', linewidth=2)
                        plt.scatter(x_coords_orig[-1], y_coords_orig[-1], color='darkred', s=200, 
                                   marker='s', label='End Point', zorder=12, edgecolors='black', linewidth=2)
                    
                    plt.xlabel('X Coordinate (m)', fontsize=12)
                    plt.ylabel('Y Coordinate (m)', fontsize=12)
                    plt.title(f'MAP Trajectory with Catmull-Rom Interpolation\n'
                             f'Original: {len(x_coords_orig)} points', fontsize=14)
                    plt.grid(True, alpha=0.3)
                    plt.legend(fontsize=12)
                    plt.axis('equal')
                    
                    # 设置坐标轴范围
                    all_x = x_coords_orig + (positions_xy[:, 0].tolist() if len(x_coords_orig) >= 4 else [])
                    all_y = y_coords_orig + (positions_xy[:, 1].tolist() if len(x_coords_orig) >= 4 else [])
                    margin = max(5, (max(all_x) - min(all_x)) * 0.1)
                    plt.xlim(min(all_x) - margin, max(all_x) + margin)
                    plt.ylim(min(all_y) - margin, max(all_y) + margin)
                    
                    plt.tight_layout()
                    
                    try:
                        plt.savefig(image_filename, dpi=300, bbox_inches='tight')
                        print(f"Successfully saved interpolated MAP trajectory visualization to {image_filename}")
                    except Exception as e:
                        print(f"Failed to save image {image_filename}: {e}")
                    
                    plt.close()  # 关闭这个图，避免影响后面的可视化
    
    # 加载车位数据
    if os.path.exists(data_files['slot']):
        print(f"Loading parking slots from {data_files['slot']}...")
        parking_data = visualizer.load_bin_file(data_files['slot'], 'slot')
        if parking_data:
            visualizer.print_data_summary(parking_data, 'slot')
    
    # 综合可视化
    if any([trajectory_loc_data, trajectory_map_data, parking_data]):
        print("\nGenerating combined visualization...")
        visualizer.visualize_combined_with_dual_trajectory(
            parking_list=parking_data, 
            trajectory_loc=trajectory_loc_data, 
            trajectory_map=trajectory_map_data
        )
    else:
        print("No valid data files found. Please check file paths and formats.")
        print("Expected files:")
        for key, path in data_files.items():
            print(f"  {key}: {path}")

if __name__ == "__main__":
    main()