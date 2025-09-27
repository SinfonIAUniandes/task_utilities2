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

def main():
    """Main function demonstrating manipulation capabilities."""
    
    # Initialize ROS2
    rclpy.init()
    
    # Create TaskModule instance
    task_module = TaskModule(
        node_name="manipulation_example",
        robot_name="demo_robot"
    )
    
    try:
        # Example 1: Basic posture control
        task_module.get_logger().info("=== Posture Control Demo ===")
        
        # Make robot stand
        task_module.get_logger().info("Making robot stand...")
        if task_module.stand():
            task_module.get_logger().info("Robot is now standing!")
        else:
            task_module.get_logger().error("Failed to make robot stand")
        
        time.sleep(2)
        
        # Make robot sit
        task_module.get_logger().info("Making robot sit...")
        if task_module.sit():
            task_module.get_logger().info("Robot is now sitting!")
        else:
            task_module.get_logger().error("Failed to make robot sit")
        
        time.sleep(2)
        
        # Example 2: Breathing control
        task_module.get_logger().info("=== Breathing Control Demo ===")
        
        # Enable breathing for arms
        task_module.get_logger().info("Enabling arm breathing...")
        if task_module.enable_breathing("Arms"):
            task_module.get_logger().info("Arm breathing enabled!")
        else:
            task_module.get_logger().error("Failed to enable arm breathing")
        
        time.sleep(3)
        
        # Disable breathing
        task_module.get_logger().info("Disabling breathing...")
        if task_module.disable_breathing("All"):
            task_module.get_logger().info("Breathing disabled!")
        else:
            task_module.get_logger().error("Failed to disable breathing")
        
        # Example 3: Animation playback
        task_module.get_logger().info("=== Animation Demo ===")
        
        # Play a wave animation (if available)
        animation_name = "stand_wave_01"  # Common animation name
        task_module.get_logger().info(f"Playing animation: {animation_name}")
        if task_module.play_animation(animation_name):
            task_module.get_logger().info("Animation started successfully!")
        else:
            task_module.get_logger().error(f"Failed to play animation: {animation_name}")
        
        time.sleep(3)
        
        # Example 4: Using direct proxy methods
        task_module.get_logger().info("=== Direct Proxy Methods Demo ===")
        
        # Use the miscellaneous proxy directly for more control
        if task_module.miscellaneous.go_to_posture("SitRelax"):
            task_module.get_logger().info("Robot is now in SitRelax posture!")
        
        time.sleep(2)
        
        # Example 5: Combined functionality
        task_module.get_logger().info("=== Combined Functionality Demo ===")
        
        # Set eye color, stand up, enable breathing, and speak
        task_module.set_eye_color(0, 255, 0, 2.0)  # Green eyes
        task_module.stand()
        task_module.enable_breathing("Body")
        
        # If speech is available
        if hasattr(task_module, 'speech'):
            task_module.speech.say("I am now standing with green eyes and breathing enabled!")
        
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
