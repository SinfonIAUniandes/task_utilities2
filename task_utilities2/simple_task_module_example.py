#!/usr/bin/env python3

# Example: simple_task_module_example.py
# This shows basic usage of TaskModule similar to simple_speech_example.py

import rclpy
from threading import Thread

# Import the TaskModule class
from .task_module.task_module import TaskModule

def main():
    rclpy.init()

    # Create TaskModule instance (single node for all functionalities)
    task_module = TaskModule(robot_name="nova")

    # Run spin in a separate thread so .call() works synchronously
    spin_thread = Thread(target=rclpy.spin, args=(task_module,))
    spin_thread.start()

    # Load robot context (nova_context.txt)
    task_module.load_robot_context()

    # Basic speech functionality - same API as before but through TaskModule
    task_module.speech.say("Hello, this is a test with TaskModule")
    
    heard = task_module.speech.listen(autocut=False, timeout=5)
    task_module.speech.say(f"I heard: {heard}")
    
    llm_response = task_module.speech.ask(heard)
    task_module.speech.say(llm_response)

    # Future: When navigation is implemented, you could do:
    # task_module.navigation.move_to_position(x=1.0, y=2.0)
    
    # Future: When vision is implemented, you could do:
    # detected_objects = task_module.vision.detect_objects()

    # Graceful shutdown
    task_module.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
