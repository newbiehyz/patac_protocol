#!/usr/bin/env python3

# for protobuf bin file check

# Usage
# python3 bin_deserialization.py  ./build/2025-08-08_16-09-09_ekf_marginalization_data.bin

import struct
import argparse
import sys
import os
from typing import List, Union

# Global variables to hold protobuf modules
patac_slot_pb2 = None
patac_dr_pb2 = None

def import_protobuf_modules(proto_path=None):
    """Import protobuf modules from specified path or default locations."""
    global patac_slot_pb2, patac_dr_pb2
    
    # List of paths to try
    paths_to_try = []
    
    if proto_path:
        paths_to_try.append(proto_path)
    
    # Default paths
    paths_to_try.extend([
        'patac_protocol/python_gen',
        '.',  # current directory
        '../patac_protocol/python_gen',  # one level up
    ])
    
    for path in paths_to_try:
        try:
            if path not in sys.path:
                sys.path.insert(0, path)
            
            import patac_slot_pb2 as slot_module
            import patac_dr_pb2 as dr_module
            
            # If successful, assign to global variables
            patac_slot_pb2 = slot_module
            patac_dr_pb2 = dr_module
            
            return True
            
        except ImportError:
            continue
    
    return False


def read_length_prefixed_messages(file_path: str) -> List[bytes]:
    """
    Read length-prefixed messages from a binary file.
    
    Each message is stored with a 4-byte length prefix followed by the message data.
    
    Args:
        file_path: Path to the binary file
        
    Returns:
        List of message bytes
    """
    messages = []
    
    try:
        with open(file_path, 'rb') as f:
            while True:
                # Read 4-byte length prefix
                length_data = f.read(4)
                if len(length_data) < 4:
                    break  # End of file
                
                # Unpack length (little-endian uint32)
                message_length = struct.unpack('<I', length_data)[0]
                
                # Read the message data
                message_data = f.read(message_length)
                if len(message_data) < message_length:
                    print(f"Warning: Expected {message_length} bytes, got {len(message_data)}")
                    break
                
                messages.append(message_data)
                
    except FileNotFoundError:
        print(f"Error: File '{file_path}' not found!")
        return []
    except Exception as e:
        print(f"Error reading file: {e}")
        return []
    
    return messages


def deserialize_parking_slot_list(message_data: bytes):
    """Deserialize ParkingSlotList message."""
    slot_list = patac_slot_pb2.ParkingSlotList()
    slot_list.ParseFromString(message_data)
    return slot_list


def deserialize_dr_pose(message_data: bytes):
    """Deserialize DRPose message."""
    dr_pose = patac_dr_pb2.DRPose()
    dr_pose.ParseFromString(message_data)
    return dr_pose


def format_parking_slot_type(slot_type: int) -> str:
    """Convert parking slot type enum to string."""
    type_map = {
        0: "Unknown",
        1: "Vertical", 
        2: "Parallel",
        3: "Oblique"
    }
    return type_map.get(slot_type, f"Unknown({slot_type})")


def format_occupancy_status(occupancy: int) -> str:
    """Convert occupancy status enum to string."""
    status_map = {
        0: "Unknown",
        1: "Occupied",
        2: "NotOccupied"
    }
    return status_map.get(occupancy, f"Unknown({occupancy})")


def format_parking_source(source: int) -> str:
    """Convert parking source enum to string."""
    source_map = {
        0: "Unspecified",
        1: "Ipm",
        2: "Uss", 
        3: "UserSel"
    }
    return source_map.get(source, f"Unknown({source})")


def print_parking_slot_list(slot_list, index: int):
    print(f"\n=== ParkingSlotList Message {index + 1} ===")
    print(f"Timestamp: {slot_list.timestamp}")
    print(f"Number of parking slots: {slot_list.num_parking_slot}")
    
    for i, slot in enumerate(slot_list.parking_slot_list):
        print(f"\n  Slot {i + 1}:")
        print(f"    ID: {slot.id}")
        print(f"    Valid: {slot.valid}")
        print(f"    Confidence: {slot.conf}")
        print(f"    Type: {format_parking_slot_type(slot.type)}")
        print(f"    Occupancy: {format_occupancy_status(slot.occupancy)}")
        print(f"    Source: {format_parking_source(slot.source)}")
        print(f"    Planning Score: {slot.planning_score}")
        
        if slot.points:
            print(f"    Points:")
            for j, point in enumerate(slot.points):
                print(f"      Point {j + 1}: ({point.x:.3f}, {point.y:.3f})")
        else:
            print(f"    Points: None")


def print_dr_pose(dr_pose, index: int):
    print(f"\n=== DRPose Message {index + 1} ===")
    print(f"Timestamp: {dr_pose.timestamp}")
    print(f"Position: ({dr_pose.x:.6f}, {dr_pose.y:.6f})")
    print(f"Yaw: {dr_pose.yaw:.6f} rad")
    if hasattr(dr_pose, 'velocity'):
        print(f"Velocity: {dr_pose.velocity:.6f} m/s")
    if hasattr(dr_pose, 'angular_velocity'):
        print(f"Angular Velocity: {dr_pose.angular_velocity:.6f} rad/s")


def auto_detect_message_type(filename: str) -> str:
    """Auto-detect message type based on filename patterns."""
    filename_lower = filename.lower()
    
    # Check for specific patterns in filename
    if "marginalization" in filename_lower or "slot_map" in filename_lower:
        return "slot"
    elif "window" in filename_lower or "deleted" in filename_lower:
        return "pose"
    elif "slot" in filename_lower:
        return "slot"
    elif "pose" in filename_lower or "dr" in filename_lower:
        return "pose"
    else:
        return "unknown"


def try_both_types(message_data: bytes, index: int, verbose: bool = False):
    """Try to deserialize as both types and return which one succeeded."""
    
    # Try ParkingSlotList first
    try:
        slot_list = deserialize_parking_slot_list(message_data)
        print_parking_slot_list(slot_list, index)
        return True
    except Exception as e1:
        if verbose:
            print(f"Failed to parse as ParkingSlotList: {e1}")
    
    # Try DRPose
    try:
        dr_pose = deserialize_dr_pose(message_data)
        print_dr_pose(dr_pose, index)
        return True
    except Exception as e2:
        if verbose:
            print(f"Failed to parse as DRPose: {e2}")
    
    return False


def main():
    parser = argparse.ArgumentParser(
        description="Deserialize protobuf files from APA local mapping system",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    
    parser.add_argument("pb_file", help="Path to the .bin file")
    parser.add_argument("--type", choices=["slot", "pose", "auto", "try-both"], 
                       default="auto", help="Message type (default: auto-detect)")
    parser.add_argument("--verbose", "-v", action="store_true", 
                       help="Verbose output")
    parser.add_argument("--proto-path", help="Path to protobuf modules directory")
    
    args = parser.parse_args()
    
    # Import protobuf modules
    if not import_protobuf_modules(args.proto_path):
        print("Error: Could not import protobuf modules!")
        print("Make sure patac_slot_pb2.py and patac_dr_pb2.py are in your Python path")
        print("or specify --proto-path to the directory containing them")
        return 1
    
    if not os.path.exists(args.pb_file):
        print(f"Error: File '{args.pb_file}' does not exist!")
        return 1
    
    print(f"Reading protobuf file: {args.pb_file}")
    
    # Read all messages from the file
    messages = read_length_prefixed_messages(args.pb_file)
    
    if not messages:
        print("No messages found in the file!")
        return 1
    
    print(f"Found {len(messages)} message(s)")
    
    # Process each message
    success_count = 0
    for i, message_data in enumerate(messages):
        if args.verbose:
            print(f"\nMessage {i + 1} size: {len(message_data)} bytes")
        
        # Determine message type
        message_type = args.type
        if message_type == "auto":
            message_type = auto_detect_message_type(args.pb_file)
            if args.verbose:
                print(f"Auto-detected type: {message_type}")
        
        # Deserialize and print based on type
        try:
            if message_type == "slot":
                slot_list = deserialize_parking_slot_list(message_data)
                print_parking_slot_list(slot_list, i)
                success_count += 1
            elif message_type == "pose":
                dr_pose = deserialize_dr_pose(message_data)
                print_dr_pose(dr_pose, i)
                success_count += 1
            elif message_type == "try-both":
                if try_both_types(message_data, i, args.verbose):
                    success_count += 1
                else:
                    print(f"Failed to deserialize message {i + 1} as either type")
            else:
                print(f"Unknown message type '{message_type}' for message {i + 1}")
                print("Trying to parse as both types...")
                if try_both_types(message_data, i, args.verbose):
                    success_count += 1
                else:
                    print(f"Failed to deserialize message {i + 1}")
                    if args.verbose:
                        print(f"Raw bytes (first 50): {message_data[:50].hex()}")
        
        except Exception as e:
            print(f"Error deserializing message {i + 1}: {e}")
            if args.verbose:
                print(f"Raw bytes (first 50): {message_data[:50].hex()}")
    
    print(f"\nSuccessfully processed {success_count}/{len(messages)} messages")
    return 0 if success_count > 0 else 1


if __name__ == "__main__":
    sys.exit(main())