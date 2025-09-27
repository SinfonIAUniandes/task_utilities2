#!/usr/bin/env python3

"""
SUPER SIMPLE Robot Agent Example - First Implementation

This is the most basic implementation of a ReAct agent integrated with robotics.
It only has 3 simple robot functionalities:
1. Say something (speech)
2. Take a picture (play animation)
3. Get current time

Usage:
    python3 minimal_robot_agent_example.py
"""

import rclpy
from threading import Thread
import time

# Import TaskModule and LLMAgent
from task_utilities2.task_module.task_module import TaskModule
from task_utilities2.task_module.logic.llm.llm_agent import LLMAgent
from langchain.tools import Tool

def main():
    """Main function with super simple robot agent."""
    
    # Initialize ROS2
    rclpy.init()
    
    # Create TaskModule instance
    task_module = TaskModule(
        node_name="minimal_robot_agent",
        robot_name="nova"
    )
    
    # Run ROS2 spin in separate thread
    spin_thread = Thread(target=rclpy.spin, args=(task_module,))
    spin_thread.start()
    
    print("=== Minimal Robot Agent Example ===")
    print("Starting robot agent with 3 simple tools...")
    
    # Wait for services to be ready
    time.sleep(2)
    
    try:
        # Define 3 super simple robot tools
        def robot_say(text: str) -> str:
            """Make the robot say something."""
            try:
                task_module.speech.say(text)
                return f"Robot said: '{text}'"
            except Exception as e:
                return f"Error saying text: {e}"
        
        def robot_take_picture(query: str = "") -> str:
            """Make the robot take a picture by playing take picture animation."""
            try:
                # Play take picture animation
                success = task_module.miscellaneous.play_animation("Stand/Waiting/TakePicture_1")
                if success:
                    return "Robot took a picture! (played take picture animation)"
                else:
                    return "Could not take picture (animation failed)"
            except Exception as e:
                return f"Error taking picture: {e}"
        
        def get_time(query: str = "") -> str:
            """Get the current time."""
            from datetime import datetime
            return datetime.now().strftime("Current time is %H:%M:%S")
        
        # Create robot tools
        robot_tools = [
            Tool(
                name="robot_say",
                func=robot_say,
                description="Make the robot say something out loud. Input: text to say."
            ),
            Tool(
                name="robot_take_picture", 
                func=robot_take_picture,
                description="Make the robot take a picture by playing animation. No input needed."
            ),
            Tool(
                name="get_time",
                func=get_time,
                description="Get the current time. No input needed."
            )
        ]
        
        # Create the robot agent with simple settings
        robot_agent = LLMAgent(
            initial_settings={
                "model_name": "gpt-4o-mini",
                "temperature": 0.1,
                "context": "You are a helpful robot assistant. You can speak, take pictures, and tell time. Keep responses short and friendly."
            },
            tools=robot_tools
        )
        
        print("Robot agent ready! Available tools:")
        for tool_name in robot_agent.list_tools():
            print(f"  - {tool_name}")
        
        # Simple interactive loop
        print("\n--- Simple Robot Agent Demo ---")
        
        # Test basic functionality
        test_commands = [
            "What time is it?",
            "Say hello to everyone",
            "Take a picture of me"
        ]
        
        for command in test_commands:
            print(f"\nUser: {command}")
            response = robot_agent.get_response(command)
            print(f"Robot Agent: {response}")
            time.sleep(2)
        
        print("\n--- Demo completed successfully! ---")
        
    except KeyboardInterrupt:
        print("Demo interrupted by user")
    except Exception as e:
        print(f"Demo failed with error: {e}")
    finally:
        # Cleanup
        print("Shutting down...")
        task_module.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()