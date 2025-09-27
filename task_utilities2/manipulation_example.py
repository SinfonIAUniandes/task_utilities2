#!/usr/bin/env python3

"""
Example usage of TaskModule with manipulation functionality.

This example demonstrates how to use the new manipulation capabilities
added to the MiscellaneousProxy, including:
- Posture control (stand, sit, rest, etc.)
- Breathing control
- Animation playback

Usage:
    ros2 run task_utilities2 manipulation_example.py
"""

import rclpy
from task_utilities2.task_module.task_module import TaskModule
import time
from threading import Thread

def main():
    """Main function demonstrating manipulation capabilities."""
    
    # Initialize ROS2
    rclpy.init()
    
    # Create TaskModule instance
    task_module = TaskModule(
        node_name="manipulation_example",
        robot_name="demo_robot"
    )
    spin_thread = Thread(target=rclpy.spin, args=(task_module,))
    spin_thread.start()
    
    try:
        # Example 1: Basic posture control
        
        # Make robot stand
        task_module.miscellaneous.go_to_posture("stand")
        
        time.sleep(2)
        
        # Example 2: Breathing control
        
        # Enable breathing for All
        task_module.get_logger().info("Enabling All breathing...")
        if task_module.miscellaneous.toggle_breathing("All",True):
            task_module.get_logger().info("All breathing enabled!")
        else:
            task_module.get_logger().error("Failed to enable All breathing")
        
        time.sleep(3)
        
        # Disable breathing
        task_module.get_logger().info("Disabling breathing...")
        if task_module.miscellaneous.toggle_breathing("All",False):
            task_module.get_logger().info("Breathing disabled!")
        else:
            task_module.get_logger().error("Failed to disable breathing")
        
        # Example 3: Animation playback
        task_module.get_logger().info("=== Animation Demo ===")
        
        # Play a wave animation (if available)
        animation_name = "Stand/Waiting/TakePicture_1"  # Common animation name
        task_module.get_logger().info(f"Playing animation: {animation_name}")
        if task_module.miscellaneous.play_animation(animation_name):
            task_module.get_logger().info("Animation started successfully!")
        else:
            task_module.get_logger().error(f"Failed to play animation: {animation_name}")
        
        time.sleep(3)
        
        task_module.get_logger().info("Demo completed successfully!")
        
    except KeyboardInterrupt:
        task_module.get_logger().info("Demo interrupted by user")
    except Exception as e:
        task_module.get_logger().error(f"Demo failed with error: {e}")
    finally:
        # Cleanup
        task_module.get_logger().info("Shutting down...")
        task_module.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
